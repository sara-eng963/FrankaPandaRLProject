%% RL Step Function Tests
% Validates kinematic state transitions and failure handling.

%% 1. Initialize Robot
projectRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(projectRoot);

run(fullfile(projectRoot, "Scripts", "robot_setup.m"));

% Verify reference configuration
assert(~checkCollision(robot, qReference, ...
    "SkippedSelfCollisions", skipPairs), ...
    "Reference configuration is self-colliding.");

%% 2. Diagnostic Environment Parameters

envParams = struct();

envParams.jointLimits = jointLimits;
envParams.skipPairs = skipPairs;
envParams.maxIncrement = deg2rad(1);
envParams.positionTolerance = 0.02;
envParams.maxSteps = 20;

% Diagnostic reward weights
envParams.reward.progress = 1;
envParams.reward.step = 0.01;
envParams.reward.success = 10;
envParams.reward.failure = 10;

%% Initialize Test Obstacle and End-Effector Position

% Distant obstacle (diagnostic fixture)
obstacle = collisionSphere(0.05);
obstacle.Pose = trvec2tform([3, 3, 3]);

% Calculate reference end-effector position
T = kinematics.forwardKinematics(robot, qReference, eeFrame);
pEE = T(1:3, 4);

%% 3. Construct LoggedSignals

state.robot = robot;
state.q = qReference;
state.eeFrame = eeFrame;
state.pEE = pEE;
state.target = pEE + [0.10; 0; 0];
state.obstacle = obstacle;
state.params = envParams;
state.stepCount = 0;
state.failureCode = 0;

%% Test 1: Zero Action

[obs, reward, done, next] = ...
    environment.stepFunction(zeros(7,1), state);

assert(isequal(next.q, state.q));
assert(~done);
assert(next.stepCount == 1);
assert(isequal(size(obs), [14,1]));
assert(isfinite(reward));

disp("PASS: Zero action preserves configuration.");

%% Test 2: Valid Joint Increment

action = zeros(7,1);
action(1) = deg2rad(0.1);

[~, ~, done, next] = ...
    environment.stepFunction(action, state);

assert(~done);
assert(next.failureCode == 0);
assert(abs(next.q(1) - state.q(1) - action(1)) < 1e-12);
assert(isequal(next.q(8:9), state.q(8:9)));

disp("PASS: Valid action updates joint configuration.");

%% Test 3: Joint-Limit Violation

nearLimit = state;

% Begin just inside joint 1's upper limit
nearLimit.q(1) = jointLimits(1,2) - deg2rad(0.25);

T = kinematics.forwardKinematics(robot, nearLimit.q, eeFrame);
nearLimit.pEE = T(1:3,4);

assert(~collision.checkRobotCollision( ...
    robot, nearLimit.q, obstacle, skipPairs), ...
    "Near-limit test configuration is colliding.");

action = zeros(7,1);
action(1) = deg2rad(0.5);

[~, ~, done, next] = ...
    environment.stepFunction(action, nearLimit);

assert(done);
assert(next.failureCode == 2);
assert(isequal(next.q, nearLimit.q));

disp("PASS: Joint-limit violation terminates episode.");

%% Test 4: Obstacle Collision

collisionState = state;

% Deliberately intersect robot with oversized sphere
collisionState.obstacle = collisionSphere(2.0);
collisionState.obstacle.Pose = eye(4);

[~, ~, done, next] = ...
    environment.stepFunction(zeros(7,1), collisionState);

assert(done);
assert(next.failureCode == 3);
assert(isequal(next.q, state.q));

disp("PASS: Obstacle collision terminates episode.");

%% Test 5: Successful Target Reaching

successState = state;
successState.target = state.pEE;

[~, reward, done, next] = ...
    environment.stepFunction(zeros(7,1), successState);

assert(done, "Success must terminate the episode.");
assert(next.failureCode == 0);
assert(abs(reward - ...
    (envParams.reward.success - envParams.reward.step)) < 1e-12);

disp("PASS: Target reaching terminates successfully.");


%% Test 6: Episode Timeout

timeoutState = state;
timeoutState.stepCount = envParams.maxSteps - 1;

[~, reward, done, next] = ...
    environment.stepFunction(zeros(7,1), timeoutState);

assert(done, "Timeout must terminate the episode.");
assert(next.failureCode == 4);
assert(next.stepCount == envParams.maxSteps);
assert(abs(reward + envParams.reward.failure + ...
    envParams.reward.step) < 1e-12);

disp("PASS: Episode timeout terminates correctly.");

%% Test 7: Action-Induced Obstacle Collision
% Verify an action causing collision is rejected.
% The large increment is a diagnostic fixture only.

collisionState = state;

% Construct a candidate movement
action = zeros(7,1);
action(1) = deg2rad(45);

% Allow this increment only for the diagnostic test
collisionState.params.maxIncrement = deg2rad(45);

qImpact = collisionState.q;
qImpact(1) = qImpact(1) + action(1);

% Compute where the end effector would move
T = kinematics.forwardKinematics(robot, qImpact, eeFrame);
impactPosition = T(1:3,4);

% Place a sphere at the candidate end-effector position
sphere = collisionSphere(0.025);
sphere.Pose = trvec2tform(impactPosition.');

collisionState.obstacle = sphere;

% Verify the test scenario is correctly constructed
assert(~collision.checkRobotCollision( ...
    robot, collisionState.q, sphere, skipPairs), ...
    "Test setup invalid: initial configuration collides.");

assert(collision.checkRobotCollision( ...
    robot, qImpact, sphere, skipPairs), ...
    "Test setup invalid: candidate does not collide.");

% Execute the action
[~, ~, done, next] = ...
    environment.stepFunction(action, collisionState);

% Verify collision failure and state rejection
assert(done);
assert(next.failureCode == 3);
assert(isequal(next.q, collisionState.q));

disp("PASS: Action-induced collision rejected.");

%% Test 8: Environment Reset
% Verify reset initializes a valid episode state.

% Start with our existing diagnostic scenario
initialState = state;

% Simulate stale episode information
initialState.stepCount = 15;
initialState.failureCode = 3;
initialState.pEE = [NaN; NaN; NaN];

% Reset
[obs, resetState] = ...
    environment.resetFunction(initialState);

% Verify episode state
assert(resetState.stepCount == 0);
assert(resetState.failureCode == 0);

% Verify observation
assert(isequal(size(obs), [14,1]));
assert(all(isfinite(obs)));

% Verify forward-kinematics consistency
assert(norm(obs(8:10) - ...
    (resetState.target - resetState.pEE)) < 1e-12);

% Verify configuration remains valid
assert(isequal(resetState.q, state.q));

disp("PASS: Environment reset initializes valid state.");

%% Test 9: Reject Joint-Limit Violation

invalidState = state;
invalidState.q(1) = jointLimits(1,2) + 0.01;

rejected = false;

try
    environment.resetFunction(invalidState);
catch ME
    rejected = contains(ME.message, "joint limits");
end

assert(rejected, ...
    "FAIL: Reset accepted an invalid joint configuration.");

disp("PASS: Reset rejects joint-limit violations.");


%% Test 10: Reject Colliding Initial Configuration

invalidState = state;

% Deliberately intersect the robot
invalidState.obstacle = collisionSphere(2.0);
invalidState.obstacle.Pose = eye(4);

rejected = false;

try
    environment.resetFunction(invalidState);
catch ME
    rejected = contains(ME.message, "colliding");
end

assert(rejected, ...
    "FAIL: Reset accepted a colliding configuration.");

disp("PASS: Reset rejects colliding configurations.");

%% Test 11: Randomized Reset
% Verify valid sampling and reproducibility.

randomState = state;

% Small diagnostic sampling region around the reference.
% NOT final training ranges.
qCenter = state.q(1:7).';
delta = deg2rad(0.25);

ranges = [qCenter - delta, qCenter + delta];

randomState.params.startJointRanges = ranges;
randomState.params.maxResetAttempts = 100;

% Preserve MATLAB's original random generator state
previousRng = rng;

% First reset
rng(42, "twister");
[obs1, reset1] = environment.resetFunction(randomState);

% Repeat with the same seed
rng(42, "twister");
[obs2, reset2] = environment.resetFunction(randomState);

% Restore original generator state
rng(previousRng);

% Verify reproducibility
assert(isequal(reset1.q, reset2.q));
assert(isequal(obs1, obs2));

% Verify sampled configuration respects the ranges
qArm = reset1.q(1:7).';

assert(all(qArm >= ranges(:,1) & ...
           qArm <= ranges(:,2)));

% Verify no collision
assert(~collision.checkRobotCollision( ...
    robot, reset1.q, reset1.obstacle, skipPairs));

% Verify gripper is unchanged
assert(isequal(reset1.q(8:9), state.q(8:9)));

% Verify observation format
assert(isequal(size(obs1), [14,1]));
assert(all(isfinite(obs1)));

disp("PASS: Randomized reset is valid and reproducible.");

%% Test 12: Randomized Reset Diversity
% Verify that multiple resets generate different valid starts.

nResets = 20;
qSamples = zeros(nResets, 7);

previousRng = rng;
rng(123, "twister");

for k = 1:nResets
    [~, resetState] = environment.resetFunction(randomState);

    % Record sampled joint angles
    qSamples(k,:) = resetState.q(1:7);

    % Verify collision-free configuration
    assert(~collision.checkRobotCollision( ...
        robot, resetState.q, resetState.obstacle, skipPairs));
end

rng(previousRng);

% Verify configurations are not identical
variation = max(qSamples, [], 1) - min(qSamples, [], 1);

assert(any(variation > 1e-10), ...
    "FAIL: Randomized resets produced identical configurations.");

disp("PASS: Randomized resets generate different valid starts.");

%% Test 13: Reset Sampling Exhaustion
% Verify reset fails explicitly when no valid start exists.

invalidState = randomState;

% Deliberately block the entire diagnostic sampling region
invalidState.obstacle = collisionSphere(2.0);
invalidState.obstacle.Pose = eye(4);

% Limit sampling attempts
invalidState.params.maxResetAttempts = 3;

rejected = false;

try
    environment.resetFunction(invalidState);
catch ME
    if contains(ME.message, ...
            "no valid start within attempt limit")
        rejected = true;
    else
        rethrow(ME);
    end
end

assert(rejected, ...
    "FAIL: Reset did not stop after exhausting attempts.");

disp("PASS: Reset sampling exhaustion handled correctly.");

%% Test 14: Randomized Target Generation
% Verify target diversity, distance constraints,
% and reproducibility.
% Workspace and solver limits are diagnostic configuration.

targetState = randomState;

% Position-only IK is configured once and shared by randomized reset tests.
targetState.params.workspaceBox = cfg.box;
targetState.params.ikSolver = inverseKinematics('RigidBodyTree',robot, ...
    'SolverAlgorithm','LevenbergMarquardt');
targetState.params.ikSolver.SolverParameters = struct('MaxIterations',100, ...
    'MaxTime',1,'SolutionTolerance',1e-6,'EnforceJointLimits',true, ...
    'AllowRandomRestart',false);
targetState.params.ikMaxInitialGuesses = 3;
targetState.params.ikPositionTolerance = 0.001;

targetState.params.maxTargetAttempts = 100;
targetState.params.minTargetDistance = 0.03;

% Save random generator state
previousRng = rng;
rng(42, "twister");

nResets = 20;
targets = zeros(3, nResets);

for k = 1:nResets
    [obs, resetState] = ...
        environment.resetFunction(targetState);

    targets(:,k) = resetState.target;

    % Verify minimum target distance
    d = norm(resetState.target - resetState.pEE);

    assert(d >= targetState.params.minTargetDistance);

    % Verify valid observation
    assert(isequal(size(obs), [14,1]));
    assert(all(isfinite(obs)));
end

% Verify target diversity
variation = max(targets, [], 2) - min(targets, [], 2);

assert(any(variation > 1e-6), ...
    "FAIL: Targets did not vary.");

% Verify reproducibility
rng(42, "twister");
[obsRepeat, stateRepeat] = ...
    environment.resetFunction(targetState);

assert(isequal(targets(:,1), stateRepeat.target));

rng(previousRng);

disp("PASS: Randomized targets are valid and reproducible.");

%% Test 15: Randomized Obstacle Placement
% Verify obstacle placement, clearance and reproducibility.
% Parameter values below are diagnostic only.

obstacleState = targetState;

obstacleState.params.obstacleFractionRange = [0.3, 0.7];
obstacleState.params.obstacleOffsetRange = [0.20, 0.30];
obstacleState.params.maxObstacleAttempts = 100;

% Save RNG state
previousRng = rng;

rng(44, "twister");
[obs1, reset1] = environment.resetFunction(obstacleState);

rng(44, "twister");
[obs2, reset2] = environment.resetFunction(obstacleState);

rng(previousRng);

% Reproducibility
assert(isequal(obs1, obs2));
assert(isequal(reset1.obstacle.Pose, reset2.obstacle.Pose));

% Recover geometric placement parameters
startPos = reset1.pEE;
targetPos = reset1.target;
center = reset1.obstacle.Pose(1:3,4);

d = targetPos - startPos;

alpha = dot(center - startPos, d) / dot(d,d);
projection = startPos + alpha*d;
rho = norm(center - projection);

assert(alpha >= 0.3 - 1e-10 && alpha <= 0.7 + 1e-10);
assert(rho >= 0.20 - 1e-10 && rho <= 0.30 + 1e-10);

% Validate starting configuration against final obstacle
assert(~collision.checkRobotCollision( ...
    robot, reset1.q, reset1.obstacle, skipPairs));

disp("PASS: Randomized obstacle placement is valid and reproducible.");

%% Test 16: Batch Randomized Reset Validation
% Test reset reliability and measure execution time.
% Uses diagnostic sampling settings, not course parameters.

nEpisodes = 30;

startSamples = zeros(7, nEpisodes);
targetSamples = zeros(3, nEpisodes);
obstacleSamples = zeros(3, nEpisodes);

previousRng = rng;
rng(2026, "twister");

tic;

for k = 1:nEpisodes

    [obs, s] = environment.resetFunction(obstacleState);

    % Check initial collision safety
    assert(~collision.checkRobotCollision( ...
        robot, s.q, s.obstacle, skipPairs));

    % Check start-to-target distance
    assert(norm(s.target - s.pEE) >= ...
        obstacleState.params.minTargetDistance);

    % Check observation validity
    assert(isequal(size(obs), [14,1]));
    assert(all(isfinite(obs)));

    % Record generated scenarios
    startSamples(:,k) = s.q(1:7).';
    targetSamples(:,k) = s.target;
    obstacleSamples(:,k) = s.obstacle.Pose(1:3,4);
end

elapsed = toc;
rng(previousRng);

% Verify scenario diversity
assert(any(range(startSamples, 2) > 1e-10));
assert(any(range(targetSamples, 2) > 1e-10));
assert(any(range(obstacleSamples, 2) > 1e-10));

fprintf("Validated episodes: %d\n", nEpisodes);
fprintf("Average reset time: %.4f seconds\n", ...
    elapsed / nEpisodes);

disp("PASS: Batch randomized reset validation.");

%% Test 17: Stale End-Effector Position Regression
% Verify target validation uses freshly calculated FK,
% not the incoming cached pEE value.

staleState = state;

% Deliberately provide an incorrect cached position
staleState.pEE = [100; 100; 100];

% Every Cartesian candidate in this tiny test box is too close to the true start.
TTrue = kinematics.forwardKinematics(robot,state.q,eeFrame);
pTrue = TTrue(1:3,4);
staleState.params.workspaceBox = [pTrue-1e-5 pTrue+1e-5];
staleState.params.ikSolver = targetState.params.ikSolver;
staleState.params.ikMaxInitialGuesses = 3;
staleState.params.ikPositionTolerance = 0.001;

staleState.params.maxTargetAttempts = 5;
staleState.params.minTargetDistance = 0.03;

% Confirm the reference goal configuration is collision-free
assert(~collision.checkRobotCollision( ...
    robot, state.q, state.obstacle, skipPairs));

% All sampled goals are too close to the true starting pose.
% Reset must reject them despite the misleading cached pEE.
rejected = false;

try
    environment.resetFunction(staleState);
catch ME
    if contains(ME.message, ...
            "no valid target within attempt limit")
        rejected = true;
    else
        rethrow(ME);
    end
end

assert(rejected, ...
    "FAIL: Target validation may be using stale FK data.");

disp("PASS: Target validation uses current FK.");

%% Test 18: Obstacle Replacement Independence
% When obstacle randomization is enabled, the original
% obstacle position must not affect the generated scenario.

scenarioA = obstacleState;
scenarioB = obstacleState;

% Move the initial placeholder obstacle onto the robot EE.
% Keep its radius unchanged.
placeholder = collisionSphere(scenarioB.obstacle.Radius);
placeholder.Pose = trvec2tform(state.pEE.');

scenarioB.obstacle = placeholder;

savedRng = rng;

rng(909, "twister");
[obsA, ~] = environment.resetFunction(scenarioA);

rng(909, "twister");
[obsB, ~] = environment.resetFunction(scenarioB);

rng(savedRng);

assert(isequal(obsA, obsB), ...
    "Reset depends on an obstacle that should be replaced.");

disp("PASS: Reset independent of discarded obstacle.");


%% Test 19: Cartesian Workspace Constraints
% Workspace bounds apply to episode starts and targets.

workspaceState = state;
workspaceState.params.workspaceBox = cfg.box;

% Valid start and target must be accepted.
[obs, s] = environment.resetFunction(workspaceState);

assert(all(s.pEE >= cfg.box(:,1) & ...
           s.pEE <= cfg.box(:,2)));

assert(all(s.target >= cfg.box(:,1) & ...
           s.target <= cfg.box(:,2)));

assert(all(isfinite(obs)));

% Reject a starting position outside the workspace.
invalidStart = workspaceState;
invalidStart.params.workspaceBox(1,1) = s.pEE(1) + 0.01;

rejected = false;
try
    environment.resetFunction(invalidStart);
catch ME
    if contains(ME.message, "Initial position is outside")
        rejected = true;
    else
        rethrow(ME);
    end
end

assert(rejected, "Out-of-workspace start was accepted.");

% Reject a target outside the workspace.
invalidTarget = workspaceState;
invalidTarget.target(1) = cfg.box(1,2) + 0.05;

rejected = false;
try
    environment.resetFunction(invalidTarget);
catch ME
    if contains(ME.message, "Target position is outside")
        rejected = true;
    else
        rethrow(ME);
    end
end

assert(rejected, "Out-of-workspace target was accepted.");

disp("PASS: Cartesian workspace constraints validated.");


%% Test 20: Randomized Workspace Sampling
% Reject out-of-bounds samples and accept valid start/target pairs.

workspaceState = state;
workspaceState.params.workspaceBox = cfg.box;

% Construct a collision-free configuration whose EE is
% outside the TA's workspace.
qOutside = qReference;
qOutside(1) = deg2rad(35);

TOutside = kinematics.forwardKinematics( ...
    robot, qOutside, eeFrame);
pOutside = TOutside(1:3,4);

assert(pOutside(1) < cfg.box(1,1), ...
    "Invalid test fixture: candidate is inside workspace.");

assert(~collision.checkRobotCollision( ...
    robot, qOutside, obstacle, skipPairs), ...
    "Invalid test fixture: candidate is colliding.");

% Narrow ranges around the invalid configuration.
qCenter = qOutside(1:7).';
delta = 1e-4;
outsideRanges = [qCenter-delta, qCenter+delta];

% A. Randomized starts must reject out-of-bounds samples.
invalidStart = workspaceState;
invalidStart.params.startJointRanges = outsideRanges;
invalidStart.params.maxResetAttempts = 5;

rejected = false;
try
    environment.resetFunction(invalidStart);
catch ME
    rejected = contains(ME.message, ...
        "no valid start within attempt limit");
end
assert(rejected, "Out-of-bounds randomized start accepted.");

% B. An impossible distance shell exhausts bounded Cartesian proposals.
invalidGoal = workspaceState;
invalidGoal.params.ikSolver = targetState.params.ikSolver;
invalidGoal.params.ikMaxInitialGuesses = 3;
invalidGoal.params.ikPositionTolerance = 0.001;
invalidGoal.params.maxTargetAttempts = 5;
invalidGoal.params.minTargetDistance = 2;

rejected = false;
try
    environment.resetFunction(invalidGoal);
catch ME
    rejected = contains(ME.message, ...
        "no valid target within attempt limit");
end
assert(rejected, "Infeasible distance-shell target accepted.");

% C. Verify successful sampling from an admissible region.
validState = workspaceState;
qCenter = qReference(1:7).';

startDelta = 5e-4;
validState.params.startJointRanges = ...
    [qCenter-startDelta, qCenter+startDelta];

validState.params.ikSolver = targetState.params.ikSolver;
validState.params.ikMaxInitialGuesses = 3;
validState.params.ikPositionTolerance = 0.001;
validState.params.maxResetAttempts = 50;
validState.params.maxTargetAttempts = 50;
validState.params.minTargetDistance = 0.03;

for k = 1:10
    [~, s] = environment.resetFunction(validState);

    assert(all(s.pEE >= cfg.box(:,1) & ...
               s.pEE <= cfg.box(:,2)));

    assert(all(s.target >= cfg.box(:,1) & ...
               s.target <= cfg.box(:,2)));
end

disp("PASS: Randomized workspace sampling validated.");


%% Test 21: Opt-in reset diagnostics preserve normal sampling behavior
% With identical RNG seeds, collecting diagnostics must not change the
% accepted episode, observation, or random-number consumption.
diagnosticState = obstacleState;
diagnosticState.params.recordScenarioDiagnostics = true;
savedRng = rng;
rng(44, "twister");
[regularObs, regularReset] = environment.resetFunction(obstacleState);
regularRng = rng;
rng(44, "twister");
[diagnosticObs, diagnosticReset] = environment.resetFunction(diagnosticState);
diagnosticRng = rng;
rng(savedRng);

assert(isequal(regularObs, diagnosticObs) && ...
       isequal(regularReset.obstacle.Pose, diagnosticReset.obstacle.Pose) && ...
       isequal(regularReset.q, diagnosticReset.q) && ...
       isequal(regularRng, diagnosticRng), ...
       "Diagnostics changed reset sampling or RNG consumption.");
diag = diagnosticReset.scenarioDiagnostics;
assert(diag.success && diag.acceptedAttempt >= 1 && ...
       diag.attempts == diag.targetPointRejects + ...
       diag.startCollisionRejects + diag.goalCollisionRejects + ...
       diag.workspaceRejects + diag.rrtRejects + 1 && ...
       numel(diag.candidateCodes) == diag.attempts, ...
       "Opt-in reset diagnostic accounting failed.");
assert(~isfield(regularReset,"scenarioDiagnostics"), ...
       "Ordinary reset unexpectedly retained diagnostics.");
disp("PASS: Optional reset diagnostics preserve normal behavior.");

%% Test 22: Failed diagnostic reset returns rejection data
failedDiagnostic = state;
failedDiagnostic.obstacle = collisionSphere(2.0);
failedDiagnostic.obstacle.Pose = trvec2tform([3,3,3]);
failedDiagnostic.params.obstacleFractionRange = [0.4 0.6];
failedDiagnostic.params.obstacleOffsetRange = [0.02 0.08];
failedDiagnostic.params.maxObstacleAttempts = 3;
failedDiagnostic.params.recordScenarioDiagnostics = true;

[failedObs,failedState] = environment.resetFunction(failedDiagnostic);
failureDiag = failedState.scenarioDiagnostics;
assert(isempty(failedObs) && ~failureDiag.success && ...
       failureDiag.failureStage == "obstacle" && ...
       failureDiag.attempts == 3 && ...
       failureDiag.targetPointRejects + ...
       failureDiag.startCollisionRejects + ...
       failureDiag.goalCollisionRejects == 3, ...
       "Failure diagnostics were lost on obstacle exhaustion.");

ordinaryFailure = rmfield(failedDiagnostic,"params");
ordinaryFailure.params = rmfield(failedDiagnostic.params, ...
    "recordScenarioDiagnostics");
caughtExhaustion = false;
try
    environment.resetFunction(ordinaryFailure);
catch ME
    caughtExhaustion = contains(ME.message, ...
        "no valid obstacle within attempt limit");
end
assert(caughtExhaustion, ...
    "Ordinary RL reset must still throw on obstacle exhaustion.");
disp("PASS: Failed reset diagnostics retained without changing RL errors.");
