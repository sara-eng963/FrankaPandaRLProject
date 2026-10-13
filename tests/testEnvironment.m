%% Panda RL Environment — Obstacle Placement Diagnostics
% Integration/scenario test: classify candidate-sphere rejections.
% Does not change the RL environment or generate predefined episodes.
%
% Prerequisite: in +environment/resetFunction.m, immediately after
% LoggedSignals.target = targetCandidate; add:
%     LoggedSignals.sampledGoalQ = qGoal;
% This metadata is NOT included in the RL observation or used to control it.
%
% Run from project root: run("tests/testEnvironment.m")

projectRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(projectRoot);
run(fullfile(projectRoot, "Scripts", "create_rl_environment.m"));

% Integration smoke test using the real MATLAB RL Toolbox environment.
initialObs = reset(env);
[nextObs, reward, done, info] = step(env, zeros(7,1));
assert(isequal(size(initialObs), [14 1]) && ...
       isequal(size(nextObs), [14 1]) && ...
       isfinite(reward) && ~done && info.stepCount == 1, ...
       "RL environment integration test failed.");
disp("PASS: RL environment reset/step integration.");

%% 1. Use the exact online-reset configuration from our recent experiment.
scenarioState = initialState;
p = scenarioState.params;
p.workspaceBox = cfg.box;

halfWidths = deg2rad([10 20 10 15 10 15 10]).';
qCenter = qReady(:);
p.goalJointRanges = [qCenter - halfWidths, qCenter + halfWidths];
p.minTargetDistance = 0.12;
p.maxTargetDistance = 0.20;
p.maxTargetAttempts = 200;
p.obstacleFractionRange = [0.4 0.6];
p.obstacleOffsetRange = [0.02 0.08];
p.maxObstacleAttempts = 100;
scenarioState.params = p;

% Temporarily omit sphere randomization to recover exactly the same
% sampled goal configuration for the independent diagnostic replay.
% The placeholder sphere from create_rl_environment.m is far away.
endpointState = scenarioState;
endpointState.params = rmfield(p, ...
    ["obstacleFractionRange", ...
     "obstacleOffsetRange", ...
     "maxObstacleAttempts"]);
nEpisodes = 30;
rejectedTarget = 0;
rejectedStart = 0;
rejectedGoal = 0;
acceptedCount = 0;
blockedEECount = 0;
totalAttempts = 0;
clearances = zeros(nEpisodes, 1);

originalRng = rng;
rng(cfg.trainSeed, "twister");

%% 2. Run the real reset and independently replay its candidate checks.
tic;
for k = 1:nEpisodes
    % Record the RNG state at the beginning of the actual episode.
    episodeRng = rng;
    actualSucceeded = true;

    try
        [~, actual] = environment.resetFunction(scenarioState);
    catch ME
        if contains(ME.message, "no valid obstacle within attempt limit")
            actualSucceeded = false;
        else
            rng(originalRng);
            rethrow(ME);
        end
    end

    afterActualRng = rng;

    % Replay exactly the start/goal draws from the same RNG state,
    % without sphere sampling, to obtain the goal joint configuration.
    rng(episodeRng);
    [~, endpoint] = environment.resetFunction(endpointState);
    assert(isfield(endpoint, "sampledGoalQ"), ...
        "Add LoggedSignals.sampledGoalQ = qGoal after accepting a goal.");

    p0 = endpoint.pEE;
    segment = endpoint.target - p0;
    direction = segment / norm(segment);

    referenceAxis = [0; 0; 1];
    if abs(dot(direction, referenceAxis)) > 0.95
        referenceAxis = [0; 1; 0];
    end
    lateral1 = cross(direction, referenceAxis);
    lateral1 = lateral1 / norm(lateral1);
    lateral2 = cross(direction, lateral1);

    diagnosticSucceeded = false;
    for attempt = 1:p.maxObstacleAttempts
        alpha = p.obstacleFractionRange(1) + ...
                rand * diff(p.obstacleFractionRange);
        rho = p.obstacleOffsetRange(1) + ...
              rand * diff(p.obstacleOffsetRange);
        theta = 2*pi*rand;

        center = p0 + alpha*segment + ...
                 rho*(cos(theta)*lateral1 + sin(theta)*lateral2);
        candidate = collisionSphere(endpoint.obstacle.Radius);
        candidate.Pose = trvec2tform(center.');
        totalAttempts = totalAttempts + 1;

        % Mutually exclusive counters, in the SAME rejection order as reset.
        if norm(center - endpoint.target) <= candidate.Radius
            rejectedTarget = rejectedTarget + 1;
            continue;
        end
        if collision.checkRobotCollision(robot, endpoint.q, ...
                candidate, skipPairs)
            rejectedStart = rejectedStart + 1;
            continue;
        end
        if collision.checkRobotCollision(robot, endpoint.sampledGoalQ, ...
                candidate, skipPairs)
            rejectedGoal = rejectedGoal + 1;
            continue;
        end

        diagnosticSucceeded = true;
        acceptedCenter = center;
        break;
    end

    % Guard against diagnostic code silently diverging from production.
    assert(diagnosticSucceeded == actualSucceeded, ...
        "Diagnostic replay disagrees with resetFunction in episode %d.", k);

    if actualSucceeded
        assert(norm(actual.target - endpoint.target) < 1e-12, ...
            "Target sampling diverged in episode %d.", k);
        assert(norm(actual.obstacle.Pose(1:3,4) - acceptedCenter) < 1e-12, ...
            "Obstacle sampling diverged in episode %d.", k);

        acceptedCount = acceptedCount + 1;
        projectedFraction = dot(acceptedCenter-p0, segment)/dot(segment,segment);
        projectedFraction = max(0, min(1, projectedFraction));
        closest = p0 + projectedFraction*segment;
        clearance = norm(acceptedCenter - closest) - ...
                    actual.obstacle.Radius;
        clearances(acceptedCount) = clearance;
        blockedEECount = blockedEECount + (clearance <= 0);
    end

    % Continue with the RNG sequence from the real reset, not its replay.
    rng(afterActualRng);
end
elapsed = toc;
rng(originalRng);

%% 3. Summary: why did obstacle candidates get rejected?
assert(totalAttempts == rejectedTarget + rejectedStart + ...
       rejectedGoal + acceptedCount, "Invalid diagnostic accounting.");

fprintf("\n=== Obstacle Placement Diagnostics ===\n");
fprintf("Actual successful resets: %d/%d\n", acceptedCount, nEpisodes);
fprintf("Total sphere candidates tested: %d\n", totalAttempts);
fprintf("Rejected: target point overlap  %d\n", rejectedTarget);
fprintf("Rejected: starting robot       %d\n", rejectedStart);
fprintf("Rejected: sampled goal robot  %d\n", rejectedGoal);
fprintf("Accepted sphere candidates:   %d\n", acceptedCount);
fprintf("Straight EE paths obstructed: %d/%d\n", ...
    blockedEECount, acceptedCount);
if acceptedCount > 0
    fprintf("Median accepted EE clearance: %.3f m\n", ...
        median(clearances(1:acceptedCount)));
end
fprintf("Elapsed wall-clock time: %.2f s\n", elapsed);
disp("PASS: Obstacle rejection diagnostics match the production reset.");

%% Visual Scenario Dashboard
% Visualize multiple accepted sampled scenarios in the TA workspace.

dashboardState = initialState;
p = dashboardState.params;

% Use the calibrated target-distance sampling.
halfWidths = deg2rad([10 20 10 15 10 15 10]).';
qCenter = qReady(:);

p.goalJointRanges = [qCenter - halfWidths, qCenter + halfWidths];
p.startJointRanges = [qCenter - halfWidths, qCenter + halfWidths];

p.maxResetAttempts = 200;
p.maxTargetAttempts = 200;
p.minTargetDistance = 0.12;
p.maxTargetDistance = 0.20;

% Obstacle placement under investigation.
p.obstacleFractionRange = [0.4, 0.6];
p.obstacleOffsetRange = [0.02, 0.08];
p.maxObstacleAttempts = 100;

% Enforce TA workspace.
p.workspaceBox = cfg.box;

dashboardState.params = p;

nDesired = 12;
accepted = 0;
scenarios = cell(nDesired, 1);

savedRng = rng;
rng(cfg.trainSeed, "twister");

for k = 1:200
    try
        [~, s] = environment.resetFunction(dashboardState);
        accepted = accepted + 1;
        scenarios{accepted} = s;

        if accepted >= nDesired
            break;
        end
    catch
        % Skip failed reset attempts in the dashboard sampler.
    end
end

rng(savedRng);

assert(accepted > 0, "No valid scenarios available for dashboard visualization.");

scenarios = scenarios(1:accepted);

%% Create figure
figure("Name", "Environment Visual Dashboard");
ax = axes;
hold(ax, "on");

% Show robot in the nominal ready configuration for reference.
show(robot, qReference, ...
    "Visuals", "on", ...
    "Collisions", "off", ...
    "Frames", "off", ...
    "Parent", ax);

%% Draw workspace box
boxLimits = cfg.box;

xmin = boxLimits(1,1); xmax = boxLimits(1,2);
ymin = boxLimits(2,1); ymax = boxLimits(2,2);
zmin = boxLimits(3,1); zmax = boxLimits(3,2);

vertices = [
    xmin ymin zmin
    xmax ymin zmin
    xmax ymax zmin
    xmin ymax zmin
    xmin ymin zmax
    xmax ymin zmax
    xmax ymax zmax
    xmin ymax zmax
];

faces = [
    1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8
];

patch("Parent", ax, ...
    "Vertices", vertices, ...
    "Faces", faces, ...
    "FaceColor", [0.1 0.7 1.0], ...
    "FaceAlpha", 0.06, ...
    "EdgeColor", [0.0 0.5 0.9], ...
    "LineWidth", 1.2);

%% Plot sampled scenarios
[Xs, Ys, Zs] = sphere(20);

allDistances = zeros(accepted,1);
allClearances = zeros(accepted,1);

for i = 1:accepted
    s = scenarios{i};

    p0 = s.pEE;
    pg = s.target;
    center = s.obstacle.Pose(1:3,4);
    radius = s.obstacle.Radius;

    % Start and goal
    scatter3(ax, p0(1), p0(2), p0(3), 45, ...
        "g", "filled");
    scatter3(ax, pg(1), pg(2), pg(3), 45, ...
        "r", "filled");

    % Reference line
    plot3(ax, [p0(1) pg(1)], ...
              [p0(2) pg(2)], ...
              [p0(3) pg(3)], ...
              "-", ...
              "Color", [0.85 0.2 0.2 0.6], ...
              "LineWidth", 1.8);

    % Sphere obstacle
    surf(ax, ...
        radius*Xs + center(1), ...
        radius*Ys + center(2), ...
        radius*Zs + center(3), ...
        "FaceColor", [1.0 0.55 0.1], ...
        "FaceAlpha", 0.18, ...
        "EdgeColor", "none");

    % Distance and straight-line clearance
    d = pg - p0;
    alpha = dot(center - p0, d) / dot(d, d);
    alpha = max(0, min(1, alpha));
    closest = p0 + alpha*d;
    clearance = norm(center - closest) - radius;

    allDistances(i) = norm(pg - p0);
    allClearances(i) = clearance;
end

%% Final formatting
axis(ax, "equal");
grid(ax, "on");
xlabel(ax, "X (m)");
ylabel(ax, "Y (m)");
zlabel(ax, "Z (m)");
view(ax, 135, 25);

title(ax, sprintf( ...
    "Sampled Start-Goal Scenarios (%d accepted)", accepted));

xlim(ax, [-0.25 0.85]);
ylim(ax, [-0.65 0.65]);
zlim(ax, [0 1.0]);

fprintf("\n=== Visual Dashboard Summary ===\n");
fprintf("Accepted scenarios displayed: %d\n", accepted);
fprintf("Distance range [m]: %.3f to %.3f\n", ...
    min(allDistances), max(allDistances));
fprintf("Median distance [m]: %.3f\n", median(allDistances));
fprintf("Clearance range [m]: %.3f to %.3f\n", ...
    min(allClearances), max(allClearances));
fprintf("Median straight-line clearance [m]: %.3f\n", ...
    median(allClearances));