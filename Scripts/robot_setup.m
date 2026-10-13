
%% Franka Panda — Robot & Project Setup
% MCT464 | Reinforcement Learning for Obstacle Avoidance
%
% Initializes the Panda model, course parameters, joint limits,
% collision exclusions, and validated reference configuration.
%
% Run once before constructing the RL environment.
% No robot initialization or visualization belongs in the step loop.

%% 1. Course Configuration

cfg = struct();

cfg.task         = "obstacle";
cfg.dt           = 0.05;      % Simulation step [s]
cfg.maxSteps     = 400;       % Episode horizon: 20 s
cfg.maxIncrement = 0.01;      % Maximum joint increment [rad]
cfg.posTol       = 0.02;      % Cartesian success tolerance [m]

% Start and target workspace: [min, max] for x, y, z [m].
cfg.box = [ 0.30  0.65;
           -0.30  0.30;
            0.15  0.60];

cfg.budgetSteps  = 300000;    % Provisional training budget
cfg.numEvalCases = 30;

% User-selected reproducibility seeds.
% The evaluation seed must never be used for training or tuning.
cfg.trainSeed = 2026;
cfg.evalSeed  = 2027;

% Enable for inspection; disable for automated training.
cfg.visualizeRobot = true;

% TA-specified initial actor/critic network size.
netSize = struct( ...
    "hiddenUnits", 128, ...
    "numLayers", 2);

%% 2. Robot Model

robot = loadrobot("frankaEmikaPanda", ...
    "DataFormat", "row");

eeFrame = "panda_hand";

% Home configuration is retained for the assignment's FK demonstration.
qHome = homeConfiguration(robot);

% TA-provided nominal arm configuration [rad].
qReady = [0 -0.785 0 -2.356 0 1.571 0.785];

% Full configuration: seven arm joints and two fixed finger joints.
gripperOpening = 0.04;        % Finger joint position [m]
qReference = [qReady gripperOpening gripperOpening];

%% 3. Joint Limits

% Extract arm limits from the robot model rather than hardcoding them.
jointLimits = zeros(7, 2);

for i = 1:7
    body = getBody(robot, sprintf("panda_link%d", i));
    jointLimits(i,:) = body.Joint.PositionLimits;
end

assert(all(isfinite(jointLimits(:))) && ...
       all(jointLimits(:,1) < jointLimits(:,2)), ...
       "Invalid arm joint limits.");

assert(all(qReady >= jointLimits(:,1).' & ...
           qReady <= jointLimits(:,2).'), ...
       "Ready configuration violates arm joint limits.");

% Validate the fixed finger positions once at initialization.
fingerNames = ["panda_leftfinger", "panda_rightfinger"];

for i = 1:numel(fingerNames)
    body = getBody(robot, fingerNames(i));
    limits = body.Joint.PositionLimits;

    assert(gripperOpening >= limits(1) && ...
           gripperOpening <= limits(2), ...
           "Fixed gripper opening violates finger joint limits.");
end

%% 4. Self-Collision Exclusions

% Exclude adjacent bodies and the known rigid link7-hand overlap.
% Other nonadjacent body pairs remain collision-checked.
skipPairs = cell(robot.NumBodies + 1, 2);

for i = 1:robot.NumBodies
    body = robot.Bodies{i};
    skipPairs(i,:) = {body.Parent.Name, body.Name};
end

skipPairs(end,:) = {'panda_link7', 'panda_hand'};

% Validate the reference pose under the project's collision policy.
assert(~checkCollision(robot, qReference, ...
    "SkippedSelfCollisions", skipPairs), ...
    "Ready configuration is self-colliding.");

%% 5. Forward Kinematics & Workspace Validation

THome = kinematics.forwardKinematics( ...
    robot, qHome, eeFrame);

TReady = kinematics.forwardKinematics( ...
    robot, qReference, eeFrame);

pHome  = THome(1:3,4);
pReady = TReady(1:3,4);

% The ready pose must lie inside the TA-specified workspace.
assert(all(pReady >= cfg.box(:,1) & ...
           pReady <= cfg.box(:,2)), ...
       "Ready end-effector position is outside the workspace.");

%% 6. Setup Summary

fprintf("\n=== Franka Panda Setup ===\n");

fprintf("Home EE position  [m]: [% .4f % .4f % .4f]\n", ...
    pHome);

fprintf("Ready EE position [m]: [% .4f % .4f % .4f]\n", ...
    pReady);

fprintf("Simulation: %.3f s/step | %d steps | %.3f rad max increment\n", ...
    cfg.dt, cfg.maxSteps, cfg.maxIncrement);

disp("PASS: Robot configuration validated.");

%% 7. Visualization

if cfg.visualizeRobot

    fig = figure("Name", "Franka Panda - Ready Configuration");

    ax = show(robot, qReference, ...
        "Visuals", "on", ...
        "Collisions", "off");

    axis(ax, "equal");
    view(ax, 135, 25);
    title(ax, "Franka Panda - Ready Configuration");

    % Color visual meshes only; preserve TF and frame colors.
    purple = [0.67 0.52 0.86];

    robotMeshes = findobj(ax.Children, ...
        "-regexp", "DisplayName", "_mesh$");

    assert(~isempty(robotMeshes), ...
        "Robot visual meshes could not be identified.");

    set(robotMeshes, "FaceColor", purple);
end
