%% Self-Collision Diagnostic
% Purpose:
% Identify colliding robot bodies at a candidate configuration.
% Uses MATLAB's parent-child exclusion policy only.

% Initialize from the project root
projectRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(projectRoot);

run(fullfile(projectRoot, "Scripts", "robot_setup.m"));

% Construct candidate configuration
qCandidate = qHome;
qCandidate(6) = mean(jointLimits(6,:));

% Fix gripper at midpoint opening for this diagnostic
left = getBody(robot, "panda_leftfinger");
right = getBody(robot, "panda_rightfinger");

qCandidate(8) = mean(left.Joint.PositionLimits);
qCandidate(9) = mean(right.Joint.PositionLimits);

%% Self-Collision Policy
% Exclude direct parent-child pairs.
% Also exclude link7-hand: rigidly connected assembly.
% Reference: MoveIt panda.srdf (reason="Adjacent").

skipPairs = cell(robot.NumBodies + 1, 2);

for i = 1:robot.NumBodies
    body = robot.Bodies{i};
    skipPairs(i,:) = {body.Parent.Name, body.Name};
end

skipPairs(end,:) = {'panda_link7', 'panda_hand'};

%% Exhaustive Collision Check

[isColliding, D] = checkCollision(robot, qCandidate, ...
    "SkippedSelfCollisions", "parent", ...
    "Exhaustive", "on");

fprintf("MATLAB release: %s\n", version("-release"));
fprintf("Self-collision detected: %d\n", isColliding);

% Identify colliding body pairs
names = [robot.BodyNames, {robot.Base.Name}];
[row, col] = find(triu(isnan(D), 1));

for k = 1:numel(row)
    fprintf("%s <-> %s\n", names{row(k)}, names{col(k)});
end

isFiltered = checkCollision(robot, qCandidate, ...
    "SkippedSelfCollisions", skipPairs);

fprintf("Collision with documented exclusions: %d\n", ...
    isFiltered);

assert(~isFiltered, ...
    "FAIL: Reference configuration is self-colliding.");

disp("PASS: Reference configuration is collision-free.");



%% Reference Configuration
% Define a reproducible, self-collision-free reference pose.
% Arm joint angles are in radians.
% Gripper positions are in meters.

qReference = qHome;

% Set joint 6 to its range midpoint
qReference(6) = mean(jointLimits(6,:));

% Fixed gripper opening
left = getBody(robot, "panda_leftfinger");
right = getBody(robot, "panda_rightfinger");

qReference(8) = mean(left.Joint.PositionLimits);
qReference(9) = mean(right.Joint.PositionLimits);

%% Self-Collision Policy
% Exclude parent-child pairs and the rigid link7-hand assembly.

skipPairs = cell(robot.NumBodies + 1, 2);

for i = 1:robot.NumBodies
    body = robot.Bodies{i};
    skipPairs(i,:) = {body.Parent.Name, body.Name};
end

skipPairs(end,:) = {'panda_link7', 'panda_hand'};

% Validate the reference configuration
assert(~checkCollision(robot, qReference, ...
    "SkippedSelfCollisions", skipPairs), ...
    "Reference configuration has a self-collision.");
%% Test: Distant Obstacle

% Diagnostic obstacle, NOT an assignment parameter
testSphere = collisionSphere(0.05);
testSphere.Pose = trvec2tform([3, 3, 3]);

isColliding = collision.checkRobotCollision( ...
    robot, qReference, testSphere, skipPairs);

assert(~isColliding, ...
    "FAIL: Unexpected collision with distant obstacle.");

disp("PASS: No collision with distant obstacle.");

%% Test: Positive Robot-Obstacle Collision
% Purpose:
% Verify that collision detection identifies a deliberately
% intersecting obstacle.
%
% The oversized sphere is a test fixture, not a project parameter.

% Create sphere encompassing the robot
testSphere = collisionSphere(2.0);
testSphere.Pose = eye(4);

% Check self-collision and obstacle collision separately
flags = checkCollision(robot, qReference, {testSphere}, ...
    "SkippedSelfCollisions", skipPairs);

assert(~flags(1), ...
    "FAIL: Unexpected self-collision.");

assert(flags(2), ...
    "FAIL: Obstacle collision was not detected.");

% Validate our reusable collision checker
assert(collision.checkRobotCollision( ...
    robot, qReference, testSphere, skipPairs), ...
    "FAIL: Custom collision checker did not detect collision.");

disp("PASS: Robot-obstacle collision detected.");

%% Visualize Tested Configuration

figure;

ax = show(robot, qCandidate, ...
    "Visuals", "off", ...
    "Collisions", "on", ...
    "Frames", "on");

% Select robot mesh patches, excluding coordinate frames
robotMeshes = findall(ax, ...
    "Type", "patch", ...
    "-regexp", "DisplayName", "_mesh$");

assert(~isempty(robotMeshes), ...
    "No mesh patches found. Check graphics object names.");

% Pink robot geometry
pink = [1.0, 0.55, 0.75];

set(robotMeshes, ...
    "FaceColor", pink, ...
    "FaceAlpha", 0.85);

axis equal;
view(3);
title("Franka Panda - Collision Geometry");