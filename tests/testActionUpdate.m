%% Test: Joint-Increment Action
% Verify the 7-DoF action updates only the arm joints.
% Units: radians.

robot = loadrobot("frankaEmikaPanda", "DataFormat", "row");
qCurrent = homeConfiguration(robot);

% Diagnostic action: rotate joint 1 by 1 degree
action = zeros(7, 1);
action(1) = deg2rad(1);

% Apply joint increments
qNext = qCurrent;
qNext(1:7) = qCurrent(1:7) + action.';

% Verify action application
assert(all(abs(qNext(1:7) - qCurrent(1:7) ...
    - action.') < 1e-12));

% Verify gripper remains fixed
assert(isequal(qNext(8:9), qCurrent(8:9)));

disp("PASS: Joint-increment action applied correctly.");

%% Test: Joint Limit Violation Detection
% Verify valid configurations are accepted and
% out-of-range configurations are rejected.

% Extract joint limits
jointLimits = zeros(7, 2);

for i = 1:7
    body = getBody(robot, sprintf("panda_link%d", i));
    jointLimits(i,:) = body.Joint.PositionLimits;
end

% Check the valid updated configuration
isValid = all(qNext(1:7) >= jointLimits(:,1).' & ...
              qNext(1:7) <= jointLimits(:,2).');

assert(isValid, "Valid configuration incorrectly rejected.");

% Deliberately exceed joint 1 upper limit
qInvalid = qNext;
qInvalid(1) = jointLimits(1,2) + 0.01;

isValid = all(qInvalid(1:7) >= jointLimits(:,1).' & ...
              qInvalid(1:7) <= jointLimits(:,2).');

assert(~isValid, "Joint-limit violation not detected.");

disp("PASS: Joint-limit violation detected.");