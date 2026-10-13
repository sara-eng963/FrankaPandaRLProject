%% Franka Panda — Environment Integration & Scenario Diagnostics
% Real online resets only: no duplicate obstacle collision replay.
% The current ranges are experimental, NOT official tuned task parameters.
% Run: run("tests/testEnvironment.m")

%% 0. Controls
doVisualize = true;
nEpisodes = 30;
maxVisibleScenarios = 12;
randomizeStarts = true;  % Set false to reproduce the previous fixed-start diagnostic.
diagnosticSeed = 2026;  % Never use cfg.evalSeed for development.

%% 1. RL Toolbox integration
projectRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(projectRoot);
run(fullfile(projectRoot, "Scripts", "create_rl_environment.m"));

obs = reset(env);
[nextObs, reward, done, info] = step(env, zeros(7,1));
assert(isequal(size(obs), [14 1]) && ...
       isequal(size(nextObs), [14 1]) && isfinite(reward) && ...
       ~done && info.stepCount == 1, "RL Toolbox integration failed.");
disp("PASS: RL environment reset/step integration.");

%% 2. One configuration for both statistics and plots
scenarioState = initialState;
p = scenarioState.params;
p.workspaceBox = cfg.box;
% Wider joint-space candidates; FK and collision rejection still enforce
% reachable start/goal positions inside the unchanged TA workspace.
halfWidths = deg2rad([30 40 30 30 30 35 40]).';
qCenter = qReady(:);
jointRanges = [max(qCenter-halfWidths, jointLimits(:,1)), ...
               min(qCenter+halfWidths, jointLimits(:,2))];
assert(all(jointRanges(:,1) < jointRanges(:,2)), ...
    "Invalid calibrated joint-sampling interval.");
p.goalJointRanges = jointRanges;
p.maxTargetAttempts = 200;
p.minTargetDistance = 0.20;
p.maxTargetDistance = 0.30;

if randomizeStarts
    p.startJointRanges = p.goalJointRanges;
    p.maxResetAttempts = 200;
end
p.obstacleFractionRange = [0.4 0.6];
p.obstacleOffsetRange = [0.02 0.08];
p.maxObstacleAttempts = 100;
p.recordScenarioDiagnostics = true;
scenarioState.params = p;

%% 3. Instrumented production resets (no replay)
% RNG restoration is exception-safe, including assertion failures.
previousRng = rng;
rngGuard = onCleanup(@() rng(previousRng));
rng(diagnosticSeed, "twister");

episodes = cell(nEpisodes,1);
resetTimes = zeros(nEpisodes,1);
status = strings(nEpisodes,1);
counts = zeros(nEpisodes,5); % [target, start, goal, workspace, accepted]
distances = nan(nEpisodes,1);
deltaXYZ = nan(nEpisodes,3);
lineClearance = nan(nEpisodes,1);

for k = 1:nEpisodes
    timer = tic;
    [episodeObs, s] = environment.resetFunction(scenarioState);
    resetTimes(k) = toc(timer);
    d = s.scenarioDiagnostics;
    episodes{k} = s;

    assert(d.attempts == d.targetPointRejects + ...
           d.startCollisionRejects + d.goalCollisionRejects + ...
           d.workspaceRejects + double(d.acceptedAttempt > 0), ...
           "Obstacle rejection accounting error at episode %d.", k);

    counts(k,:) = [d.targetPointRejects, d.startCollisionRejects, ...
                   d.goalCollisionRejects, d.workspaceRejects, ...
                   double(d.acceptedAttempt > 0)];
    if ~isempty(episodeObs)
        assert(d.success && isequal(size(episodeObs), [14 1]), ...
            "Accepted reset returned an invalid observation.");
    else
        assert(~d.success && strlength(d.failureStage) > 0, ...
            "Failed diagnostic reset lacks a failure stage.");
    end
    % Start/target data is meaningful only after both endpoints were
    % sampled; an early failure may still contain initialState placeholders.
    if (d.success || d.failureStage == "obstacle") && ...
            all(isfinite(s.pEE)) && all(isfinite(s.target))
        deltaXYZ(k,:) = (s.target - s.pEE).';
        distances(k) = norm(s.target - s.pEE);
    end
    if d.success
        status(k) = "accepted";
        center = s.obstacle.Pose(1:3,4);
        segment = s.target - s.pEE;
        fraction = max(0,min(1,dot(center-s.pEE,segment)/dot(segment,segment)));
        closest = s.pEE + fraction*segment;
        lineClearance(k) = norm(center-closest) - s.obstacle.Radius;
        assert(~collision.checkRobotCollision(robot,s.q,s.obstacle,skipPairs), ...
            "Accepted start configuration is colliding.");
    else
        status(k) = d.failureStage;
    end
end

clear rngGuard; % restores the caller's RNG state

%% 4. Numerical report
nAccepted = nnz(status == "accepted");
fprintf("\n=== Online Obstacle-Placement Diagnostics ===\n");
fprintf("Start sampling: %s | Episodes: %d\n", ...
    string(randomizeStarts), nEpisodes);
fprintf("Accepted: %d/%d | Failed: %d\n", nAccepted,nEpisodes,nEpisodes-nAccepted);
fprintf("Failures: start=%d target=%d obstacle=%d\n", ...
    nnz(status=="start"),nnz(status=="target"),nnz(status=="obstacle"));
fprintf("Sphere candidates: %d\n",sum(counts,"all"));
fprintf("Rejected target/start/goal/workspace: %d / %d / %d / %d\n", ...
    sum(counts(:,1)),sum(counts(:,2)),sum(counts(:,3)),sum(counts(:,4)));
fprintf("Accepted sphere candidates: %d\n",sum(counts(:,5)));
fprintf("Reset time: median %.1f ms | max %.1f ms\n", ...
    median(resetTimes)*1000,max(resetTimes)*1000);
validDistance = distances(isfinite(distances));
if ~isempty(validDistance)
    fprintf("Endpoint separation: median %.1f cm | range %.1f–%.1f cm\n", ...
        100*median(validDistance),100*min(validDistance),100*max(validDistance));
    fprintf("Median displacement XYZ [cm]: [%.1f %.1f %.1f]\n", ...
        100*median(deltaXYZ(isfinite(distances),:),1));
end
validClearance = lineClearance(isfinite(lineClearance));
if ~isempty(validClearance)
    fprintf("Accepted EE line obstructed: %d/%d\n", ...
        nnz(validClearance<=0),numel(validClearance));
    fprintf("Median accepted line clearance: %.1f cm\n",100*median(validClearance));
end
fprintf("NOTE: Endpoint safety and EE line clearance do not establish a feasible whole-arm route.\n");
disp("PASS: Online diagnostic accounting verified.");

%% 5. Nominal whole-arm motion diagnostic (testing ONLY)
% Straight interpolation in JOINT space is a geometric reference, not
% an RL action, demonstration, Cartesian EE path, or motion planner.
% Sample increments are at most the course's 0.01 rad per joint.
acceptedEpisodes = find(status == "accepted");
nominalBlocked = false(nEpisodes,1);
nominalSelfCollision = false(nEpisodes,1);
nominalObstacleOnly = false(nEpisodes,1);
nominalSteps = nan(nEpisodes,1);

for episodeIndex = acceptedEpisodes.'
    episode = episodes{episodeIndex};
    assert(isfield(episode,"sampledGoalQ"), ...
        "Accepted randomized episode lacks its sampled goal configuration.");

    qStart = episode.q;
    qGoal = episode.sampledGoalQ;
    assert(isequal(size(qStart),[1 9]) && isequal(size(qGoal),[1 9]), ...
        "Invalid start or goal configuration at episode %d.",episodeIndex);

    dq = qGoal(1:7) - qStart(1:7);
    nSteps = max(1,ceil(max(abs(dq))/episode.params.maxIncrement));
    nominalSteps(episodeIndex) = nSteps;

    for sampleIndex = 1:nSteps
        qSample = qStart;
        qSample(1:7) = qStart(1:7) + (sampleIndex/nSteps)*dq;

        if collision.checkRobotCollision(episode.robot,qSample, ...
                episode.obstacle,episode.params.skipPairs)
            nominalBlocked(episodeIndex) = true;
            % A self-collision at this sample may coexist with an
            % obstacle collision; only a self-free sample establishes
            % that the obstacle alone obstructs this nominal motion.
            nominalSelfCollision(episodeIndex) = checkCollision( ...
                episode.robot,qSample, ...
                "SkippedSelfCollisions",episode.params.skipPairs);
            nominalObstacleOnly(episodeIndex) = ...
                ~nominalSelfCollision(episodeIndex);
            break;
        end
    end
end

nBlocked = nnz(nominalBlocked(acceptedEpisodes));
nObstacleOnly = nnz(nominalObstacleOnly(acceptedEpisodes));
nSelfCollision = nnz(nominalSelfCollision(acceptedEpisodes));
nDirectClear = numel(acceptedEpisodes)-nBlocked;
nExceedsHorizon = nnz(nominalSteps(acceptedEpisodes)>cfg.maxSteps);
nDirectFeasible = nnz(~nominalBlocked(acceptedEpisodes) & ...
    nominalSteps(acceptedEpisodes)<=cfg.maxSteps);

fprintf("\n=== Whole-Arm Nominal Motion Diagnostic ===\n");
fprintf("Accepted scenarios tested: %d\n",numel(acceptedEpisodes));
fprintf("Direct joint-space interpolation blocked: %d/%d\n", ...
    nBlocked,numel(acceptedEpisodes));
fprintf("First collision: obstacle-only=%d | self (possibly both)=%d\n", ...
    nObstacleOnly,nSelfCollision);
fprintf("Collision-free at sampled configurations: %d/%d\n", ...
    nDirectClear,numel(acceptedEpisodes));
fprintf("Direct interpolation exceeds %d-step horizon: %d\n", ...
    cfg.maxSteps,nExceedsHorizon);
fprintf("Sampled collision-free and within horizon: %d/%d\n", ...
    nDirectFeasible,numel(acceptedEpisodes));
fprintf("NOTE: Discrete checks (<=%.3f rad/joint per sample) do not prove\n", ...
    cfg.maxIncrement);
fprintf("continuous swept-path clearance or existence of another route.\n");
disp("PASS: Whole-arm nominal motion diagnostic completed.");

%% 5A. Obstacle-free reference check (TEST ONLY)
% Use the same joint-space samples. Never provide this reference to RL.
% A trajectory collision-free WITH the sphere must also be self-free
% without it, so only recheck cases blocked in the original test.
obstacleFreeSelfBlocked = false(nEpisodes,1);

for episodeIndex = acceptedEpisodes.'
    if ~nominalBlocked(episodeIndex)
        continue; % Existing full-arm check covered every sample.
    end
    if nominalSelfCollision(episodeIndex)
        obstacleFreeSelfBlocked(episodeIndex) = true;
        continue; % Self-collision was already detected.
    end

    episode = episodes{episodeIndex};
    qStart = episode.q;
    dq = episode.sampledGoalQ(1:7) - qStart(1:7);
    nSteps = nominalSteps(episodeIndex);

    for sampleIndex = 1:nSteps
        qSample = qStart;
        qSample(1:7) = qStart(1:7) + (sampleIndex/nSteps)*dq;

        if any(checkCollision(episode.robot,qSample, ...
                "SkippedSelfCollisions",episode.params.skipPairs))
            obstacleFreeSelfBlocked(episodeIndex) = true;
            break;
        end
    end
end

withSphereBlocked = nominalBlocked(acceptedEpisodes);
withoutSphereBlocked = obstacleFreeSelfBlocked(acceptedEpisodes);
blockedOnlyWithSphere = nnz(withSphereBlocked & ~withoutSphereBlocked);
blockedBothWays = nnz(withSphereBlocked & withoutSphereBlocked);
clearBothWays = nnz(~withSphereBlocked & ~withoutSphereBlocked);

assert(~any(~withSphereBlocked & withoutSphereBlocked), ...
    "Obstacle-free reference contradicts whole-arm collision checks.");
assert(blockedOnlyWithSphere + blockedBothWays + clearBothWays == ...
       numel(acceptedEpisodes), "Reference-motion accounting mismatch.");

fprintf("\n=== Obstacle-Free Whole-Arm Reference ===\n");
fprintf("Accepted scenarios compared: %d\n",numel(acceptedEpisodes));
fprintf("Self-colliding without sphere: %d/%d\n", ...
    nnz(withoutSphereBlocked),numel(acceptedEpisodes));
fprintf("Clear without sphere, blocked with sphere: %d/%d\n", ...
    blockedOnlyWithSphere,numel(acceptedEpisodes));
fprintf("Blocked with and without sphere: %d/%d\n", ...
    blockedBothWays,numel(acceptedEpisodes));
fprintf("Clear with and without sphere: %d/%d\n", ...
    clearBothWays,numel(acceptedEpisodes));
fprintf("NOTE: Discrete checks do not prove continuous swept-path\n");
fprintf("clearance or a feasible alternative route.\n");
disp("PASS: Obstacle-free reference comparison completed.");

%% 6. Two-pane interactive inspection, using the SAME episodes as report
if doVisualize
    available = find(isfinite(distances));  % includes obstacle-placement failures
    if isempty(available)
        warning("No valid endpoint pairs to visualize.");
    else
        available = available(1:min(maxVisibleScenarios,numel(available)));
        fig = figure("Name","Panda | Scenario Diagnostics", ...
            "NumberTitle","off","Color",[0.10 0.12 0.16], ...
            "Position",[100 100 1350 660]);
        overviewAx = axes("Parent",fig,"Position",[0.05 0.13 0.40 0.76]);
        detailAx = axes("Parent",fig,"Position",[0.54 0.13 0.41 0.76]);

        hold(overviewAx,"on");
        drawWorkspaceBox(overviewAx,cfg.box);
        % Call the function explicitly: testStepFunction.m leaves a
        % collisionSphere variable named "sphere" in the base workspace.
        [sphereX,sphereY,sphereZ] = feval("sphere",16);

        for j = 1:numel(available)
            ep = episodes{available(j)};
            startPos = ep.pEE;
            goalPos = ep.target;
            plot3(overviewAx,[startPos(1) goalPos(1)], ...
                [startPos(2) goalPos(2)],[startPos(3) goalPos(3)], ...
                "-","Color",[0.47 0.68 0.92],"LineWidth",1.4);
            scatter3(overviewAx,startPos(1),startPos(2),startPos(3), ...
                32,[0.24 0.82 0.58],"filled");
            scatter3(overviewAx,goalPos(1),goalPos(2),goalPos(3), ...
                32,[0.99 0.36 0.43],"filled");
            if ep.scenarioDiagnostics.success
                c = ep.obstacle.Pose(1:3,4);
                r = ep.obstacle.Radius;
                surf(overviewAx,r*sphereX+c(1),r*sphereY+c(2), ...
                    r*sphereZ+c(3),"EdgeColor","none", ...
                    "FaceColor",[0.99 0.65 0.25],"FaceAlpha",0.22);
            end
        end
        finishAxes(overviewAx,cfg.box,false);
        title(overviewAx,sprintf("Workspace | %d sampled pairs",numel(available)), ...
            "Color","w");

        selection = uicontrol(fig,"Style","popupmenu", ...
            "Units","normalized","Position",[0.58 0.94 0.33 0.045], ...
            "String",cellstr("Episode "+string(available)), ...
            "BackgroundColor",[0.96 0.96 0.96]);
        viewData = struct("axes",detailAx,"figure",fig, ...
            "episodes",{episodes},"indices",available,"robot",robot, ...
            "workspace",cfg.box,"sphereX",sphereX, ...
            "sphereY",sphereY,"sphereZ",sphereZ);
        selection.Callback = @(src,~) renderSelectedScenario(viewData,src.Value);
        renderSelectedScenario(viewData,1);
    end
end

%% Local visualization routines (not called by the RL environment)
function drawWorkspaceBox(ax,limits)
    x=limits(1,:); y=limits(2,:); z=limits(3,:);
    v=[x(1) y(1) z(1);x(2) y(1) z(1);x(2) y(2) z(1); ...
       x(1) y(2) z(1);x(1) y(1) z(2);x(2) y(1) z(2); ...
       x(2) y(2) z(2);x(1) y(2) z(2)];
    f=[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
    patch(ax,"Vertices",v,"Faces",f, ...
        "FaceColor",[0.08 0.67 0.88],"FaceAlpha",0.07, ...
        "EdgeColor",[0.16 0.77 0.98],"LineWidth",1.3);
end

function finishAxes(ax,limits,showRobot)
    axis(ax,"equal");
    view(ax,130,23);
    grid(ax,"on"); box(ax,"on");
    xlabel(ax,"X (m)");ylabel(ax,"Y (m)");zlabel(ax,"Z (m)");
    ax.Color=[0.10 0.12 0.16]; ax.XColor=[0.88 0.90 0.94];
    ax.YColor=ax.XColor;ax.ZColor=ax.XColor;
    if ~showRobot
        pad=0.06;
        xlim(ax,limits(1,:)+[-pad pad]);
        ylim(ax,limits(2,:)+[-pad pad]);
        zlim(ax,limits(3,:)+[-pad pad]);
    else
        % Entire robot + task region; avoid clipping the manipulator.
        xlim(ax,[-0.35 0.82]); ylim(ax,[-0.55 0.55]); zlim(ax,[0 0.95]);
    end
end

function renderSelectedScenario(data,selection)
    ax=data.axes;
    cla(ax);
    ep=data.episodes{data.indices(selection)};
    d=ep.scenarioDiagnostics;
    show(data.robot,ep.q,"Parent",ax,"Visuals","on", ...
        "Collisions","off","Frames","off","PreservePlot",false);
    hold(ax,"on");
    drawWorkspaceBox(ax,data.workspace);
    a=ep.pEE;b=ep.target;
    plot3(ax,[a(1) b(1)],[a(2) b(2)],[a(3) b(3)], ...
        "-","Color",[0.50 0.75 1],"LineWidth",2);
    scatter3(ax,a(1),a(2),a(3),90,[0.24 0.82 0.58],"filled");
    scatter3(ax,b(1),b(2),b(3),90,[0.99 0.36 0.43],"filled");

    % Only a capped subset of rejected candidates is drawn to avoid clutter.
    r=ep.obstacle.Radius;
    n=numel(d.candidateCodes);
    if d.success
        c=ep.obstacle.Pose(1:3,4);
        surf(ax,r*data.sphereX+c(1),r*data.sphereY+c(2), ...
            r*data.sphereZ+c(3),"EdgeColor","none", ...
            "FaceColor",[1 0.67 0.22],"FaceAlpha",0.48);
    end
    rejected=find(d.candidateCodes~=4);
    if ~isempty(rejected)
        subset=rejected(round(linspace(1,numel(rejected), ...
            min(32,numel(rejected)))));
        centers=d.candidateCenters(subset,:);
        cc=d.candidateCodes(subset);
        palette=[0.70 0.70 0.70; 0.88 0.31 0.36; ...
                 0.99 0.64 0.22; 0 0 0; 0.55 0.63 0.78];
        for code=[1 2 3 5]
            m=cc==code;
            if any(m)
                scatter3(ax,centers(m,1),centers(m,2),centers(m,3), ...
                    26,palette(code,:),"filled","MarkerFaceAlpha",0.6);
            end
        end
    end
    finishAxes(ax,data.workspace,true);
    if d.success
        episodeLabel = "accepted";
    else
        episodeLabel = "failed: " + d.failureStage;
    end
    title(ax,sprintf("Episode %d | %s | %.1f cm | %d candidates", ...
        data.indices(selection),char(episodeLabel), ...
        100*norm(b-a),n),"Color","w");
end
