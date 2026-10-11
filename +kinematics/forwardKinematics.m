function T_base_ee = forwardKinematics(robot, q, eeFrame)
%FORWARDKINEMATICS Compute the end-effector pose.
%
% Inputs:
%   robot   - rigidBodyTree robot model
%   q       - Full robot joint configuration
%   eeFrame - End-effector body name
%
% Output:
%   T_base_ee - 4x4 homogeneous transformation matrix
%               relative to the robot base
%
% Position units: meters
% Joint angles: radians

    T_base_ee = getTransform(robot, q, eeFrame);

end