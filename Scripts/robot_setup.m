
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
% Environment creation supplies a temporary override for headless training.
if exist('robotSetupVisualize','var')
    validateattributes(robotSetupVisualize,{'logical'},{'scalar'});
    cfg.visualizeRobot = robotSetupVisualize;
end

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

% The-provided nominal arm configuration [rad].
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



%% 7. Robot Setup & Forward Kinematics Dashboard

if cfg.visualizeRobot

    %% 7.1 Dashboard Figure

    fig = figure( ...
        "Name", "Franka Panda - Robot Setup & FK", ...
        "NumberTitle", "off", ...
        "Color", [0.10 0.11 0.14], ...
        "Position", [80 80 1400 850]);

    annotation(fig, "textbox", [0.10 0.93 0.80 0.045], ...
        "String", "FRANKA PANDA | ROBOT SETUP & FORWARD KINEMATICS", ...
        "FontSize", 17, ...
        "FontWeight", "bold", ...
        "Color", "white", ...
        "HorizontalAlignment", "center", ...
        "EdgeColor", "none");

    %% 7.2 Home and Ready Configurations

    configurations = {qHome, qReference};
    poseNames = {"Home Configuration", "Ready Configuration"};

    for k = 1:2

        ax = axes("Parent", fig, ...
            "Position", [0.05 + (k-1)*0.49, 0.53, 0.42, 0.29]);

        show(robot, configurations{k}, ...
            "Parent", ax, ...
            "Visuals", "on", ...
            "Collisions", "off", ...
            "Frames", "on");

        % Axes formatting
        ax.Color = [0.10 0.11 0.14];
        ax.XColor = [0.85 0.87 0.90];
        ax.YColor = [0.85 0.87 0.90];
        ax.ZColor = [0.85 0.87 0.90];
        ax.GridColor = [0.45 0.48 0.53];
        ax.GridAlpha = 0.25;

        axis(ax, "equal");
        xlim(ax, [-0.4 0.8]);
        ylim(ax, [-0.6 0.6]);
        zlim(ax, [0 1.2]);

        view(ax, 135, 25);
        grid(ax, "on");

        xlabel(ax, "X (m)");
        ylabel(ax, "Y (m)");
        zlabel(ax, "Z (m)");

        title(ax, poseNames{k}, ...
            "FontSize", 13, ...
            "FontWeight", "bold", ...
            "Color", "white");

        % Color only robot meshes, not coordinate frames
        robotMeshes = findobj(ax.Children, ...
            "-regexp", "DisplayName", "_mesh$");

        if ~isempty(robotMeshes)
            set(robotMeshes, ...
                "FaceColor", [0.67 0.52 0.86]);
        end

    end

    %% 7.3 Forward Kinematics Results

    % Cartesian position [m]
    xyz = [pHome.'; pReady.'];

    % ZYX Euler angles [degrees]
    eulerZYX = rad2deg([
        rotm2eul(THome(1:3,1:3), "ZYX");
        rotm2eul(TReady(1:3,1:3), "ZYX")
    ]);

    fkData = [
        {'Home'; 'Ready'}, ...
        num2cell(round([xyz eulerZYX], 4))
    ];

    annotation(fig, "textbox", [0.10 0.365 0.80 0.045], ...
        "String", ...
        "Forward Kinematics | End-Effector Frame: panda_hand", ...
        "FontSize", 12, ...
        "FontWeight", "bold", ...
        "Color", "white", ...
        "EdgeColor", "none");

    uitable("Parent", fig, ...
        "Units", "normalized", ...
        "Position", [0.10 0.235 0.80 0.12], ...
        "Data", fkData, ...
        "ColumnName", { ...
            "Pose", ...
            "X (m)", "Y (m)", "Z (m)", ...
            "Yaw (deg)", "Pitch (deg)", "Roll (deg)"}, ...
        "RowName", []);

    %% 7.4 Joint Configuration Comparison

    jointData = [
        {'Home'; 'Ready'}, ...
        num2cell(round([qHome(1:7); qReady], 4))
    ];

    annotation(fig, "textbox", [0.10 0.185 0.80 0.045], ...
        "String", "Arm Joint Configurations [rad]", ...
        "FontSize", 12, ...
        "FontWeight", "bold", ...
        "Color", "white", ...
        "EdgeColor", "none");

    jointHeaders = { ...
        "Pose", "q1", "q2", "q3", "q4", ...
        "q5", "q6", "q7"};

    uitable("Parent", fig, ...
        "Units", "normalized", ...
        "Position", [0.10 0.055 0.80 0.12], ...
        "Data", jointData, ...
        "ColumnName", jointHeaders, ...
        "RowName", []);

end
