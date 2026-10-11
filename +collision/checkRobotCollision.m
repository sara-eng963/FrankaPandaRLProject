function isColliding = checkRobotCollision(robot, q, obstacle, skipPairs)
%CHECKROBOTCOLLISION Check robot self-collision and obstacle collision.
%
% Inputs:
%   robot     - Franka Panda rigidBodyTree model
%   q         - 1x9 full joint configuration
%               Joint angles [rad], finger positions [m]
%   obstacle  - collisionSphere in the robot base frame
%   skipPairs - Nx2 cell array of excluded self-collision pairs
%
% Output:
%   isColliding - Logical true if any collision is detected
%
% Notes:
%   - Checks all robot bodies with collision geometry.
%   - Uses the predefined self-collision exclusion policy.
%   - Does not check joint limits.
%   - Does not perform visualization.

    collisionFlags = checkCollision(robot, q, {obstacle}, ...
        "SkippedSelfCollisions", skipPairs, ...
        "Exhaustive", "off");

    isColliding = any(collisionFlags);
end