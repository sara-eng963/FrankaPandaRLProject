clc; clear; close all;

% Load robot model
robot = loadrobot("frankaEmikaPanda", ...
    "DataFormat", "row");

% Get home configuration
qHome = homeConfiguration(robot);

% Define end-effector frame
eeFrame = "panda_hand";

% Calculate forward kinematics
T = kinematics.forwardKinematics(robot, qHome, eeFrame);

% Extract Cartesian position
p = T(1:3, 4);

disp("End-effector position [m]:");
disp(p);

%% Arm Joint Limits
% Extract position limits for the 7 revolute arm joints.
% Units: radians. Gripper joints are excluded.

nArmJoints = 7;
jointLimits = zeros(nArmJoints, 2);

for i = 1:nArmJoints
    body = getBody(robot, sprintf("panda_link%d", i));
    jointLimits(i, :) = body.Joint.PositionLimits;
end

% Validate limit data
assert(all(isfinite(jointLimits(:))) && ...
       all(jointLimits(:,1) < jointLimits(:,2)), ...
       "Invalid joint limits in robot model.");

disp("Arm joint limits [degrees]:");
disp(rad2deg(jointLimits));

%% Self-Collision Exclusions
% Skip directly connected bodies and the permanent
% geometry overlap between link7 and the hand.
% Do not exclude other wrist collisions.

skipPairs = cell(robot.NumBodies + 1, 2);

for i = 1:robot.NumBodies
    body = robot.Bodies{i};
    skipPairs(i,:) = {body.Parent.Name, body.Name};
end

skipPairs(end,:) = {'panda_link7', 'panda_hand'};
%% Validated Reference Configuration
% Reference configuration for simulation and testing.
% Fixed gripper opening, with self-collision validation.

qReference = qHome;

% Adjust joint 6
qReference(6) = mean(jointLimits(6,:));

% Fixed gripper opening
left = getBody(robot, "panda_leftfinger");
right = getBody(robot, "panda_rightfinger");

qReference(8) = mean(left.Joint.PositionLimits);
qReference(9) = mean(right.Joint.PositionLimits);

% Verify joint limits
assert(all(qReference(1:7) >= jointLimits(:,1).' & ...
           qReference(1:7) <= jointLimits(:,2).'));

% Verify self-collision
assert(~checkCollision(robot, qReference, ...
    "SkippedSelfCollisions", skipPairs), ...
    "Reference configuration is self-colliding.");

% Visualize robot
figure;
show(robot, qReference);
axis equal;
