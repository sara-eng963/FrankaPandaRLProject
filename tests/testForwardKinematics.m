%% Test: Forward Kinematics - Joint Perturbation
% Verify that changing joint 1 changes the end-effector position.

robot = loadrobot("frankaEmikaPanda", "DataFormat", "row");
eeFrame = "panda_hand";

% Original configuration
qHome = homeConfiguration(robot);
T_original = kinematics.forwardKinematics(robot, qHome, eeFrame);

% Perturb joint 1 by 10 degrees
qTest = qHome;
qTest(1) = qTest(1) + deg2rad(10);

T_test = kinematics.forwardKinematics(robot, qTest, eeFrame);

% Compare positions
disp("Original position:");
disp(T_original(1:3,4));

disp("New position:");
disp(T_test(1:3,4));

% Automated check
assert(norm(T_test(1:3,4) - T_original(1:3,4)) > 1e-6, ...
    "Test failed: End-effector position did not change.");

disp("PASS: Joint perturbation changes end-effector position.");