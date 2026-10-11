
function [InitialObs, LoggedSignals] = resetFunction(initialState)
%RESETFUNCTION Initialize a Franka Panda RL environment episode.
%
% Input:
%   initialState - Structure containing:
%       robot    : rigidBodyTree model
%       q        : 1x9 robot configuration [rad, m]
%       eeFrame  : End-effector frame name
%       target   : 3x1 target position [m]
%       obstacle : collisionSphere in the base frame
%       params   : Environment and sampling parameters
%
% Outputs:
%   InitialObs   - 14x1 observation vector
%   LoggedSignals - Initialized episode state
%
% Supports deterministic and randomized starts, targets,
% and obstacles. Sampling is bounded and collision-checked.
%
% No inverse kinematics or trajectory planning is performed.

    LoggedSignals = initialState;
    p = LoggedSignals.params;
    q = LoggedSignals.q;

    % Identify enabled randomization modes.
    randomizeStart = isfield(p, "startJointRanges");
    randomizeTarget = isfield(p, "goalJointRanges");
    randomizeObstacle = isfield(p, "obstacleFractionRange");

    % Validate the input configuration before using it.
    assert(isequal(size(q), [1, 9]) && all(isfinite(q)), ...
        "Invalid initial joint configuration.");

    %% 1. Randomized Starting Configuration

    if randomizeStart

        ranges = p.startJointRanges;

        % Validate sampling ranges once, outside the loop.
        assert(isequal(size(ranges), [7, 2]) && ...
               all(isfinite(ranges(:))) && ...
               all(ranges(:,1) < ranges(:,2)) && ...
               all(ranges(:,1) >= p.jointLimits(:,1)) && ...
               all(ranges(:,2) <= p.jointLimits(:,2)), ...
               "Invalid starting joint ranges.");

        assert(isfield(p, "maxResetAttempts") && ...
               isscalar(p.maxResetAttempts) && ...
               isfinite(p.maxResetAttempts) && ...
               p.maxResetAttempts >= 1 && ...
               p.maxResetAttempts == floor(p.maxResetAttempts), ...
               "Invalid maximum reset attempts.");

        accepted = false;

        for attempt = 1:p.maxResetAttempts

            % Uniformly sample seven arm joint angles.
            % Preserve the fixed gripper configuration.
            qCandidate = q;

            qCandidate(1:7) = ranges(:,1).' + ...
                rand(1,7) .* (ranges(:,2) - ranges(:,1)).';

            % If the obstacle will be replaced, check only
            % self-collision at this stage.
            if randomizeObstacle
                isColliding = checkCollision( ...
                    LoggedSignals.robot, qCandidate, ...
                    "SkippedSelfCollisions", p.skipPairs);
            else
                % The existing obstacle is part of the episode.
                isColliding = collision.checkRobotCollision( ...
                    LoggedSignals.robot, qCandidate, ...
                    LoggedSignals.obstacle, p.skipPairs);
            end

            if isColliding
                continue;
            end

            q = qCandidate;
            accepted = true;
            break;
        end

        assert(accepted, ...
            "Reset failed: no valid start within attempt limit.");

    else

        %% Deterministic Starting Configuration

        assert(all(q(1:7) >= p.jointLimits(:,1).') && ...
               all(q(1:7) <= p.jointLimits(:,2).'), ...
               "Initial configuration violates joint limits.");

        if randomizeObstacle
            isColliding = checkCollision( ...
                LoggedSignals.robot, q, ...
                "SkippedSelfCollisions", p.skipPairs);
        else
            isColliding = collision.checkRobotCollision( ...
                LoggedSignals.robot, q, ...
                LoggedSignals.obstacle, p.skipPairs);
        end

        assert(~isColliding, ...
            "Initial configuration is colliding.");
    end

    LoggedSignals.q = q;

    %% 2. Calculate Initial End-Effector Position
    % Must happen after sampling the start configuration.

    T = kinematics.forwardKinematics( ...
        LoggedSignals.robot, q, LoggedSignals.eeFrame);

    LoggedSignals.pEE = T(1:3,4);

    %% 3. Randomized Target Generation

    if randomizeTarget

        ranges = p.goalJointRanges;

        assert(isequal(size(ranges), [7, 2]) && ...
               all(isfinite(ranges(:))) && ...
               all(ranges(:,1) < ranges(:,2)) && ...
               all(ranges(:,1) >= p.jointLimits(:,1)) && ...
               all(ranges(:,2) <= p.jointLimits(:,2)), ...
               "Invalid goal joint ranges.");

        assert(isscalar(p.maxTargetAttempts) && ...
               isfinite(p.maxTargetAttempts) && ...
               p.maxTargetAttempts >= 1 && ...
               p.maxTargetAttempts == floor(p.maxTargetAttempts), ...
               "Invalid target sampling attempt limit.");

        assert(isscalar(p.minTargetDistance) && ...
               isfinite(p.minTargetDistance) && ...
               p.minTargetDistance > p.positionTolerance, ...
               "Invalid minimum target distance.");

        accepted = false;

        for attempt = 1:p.maxTargetAttempts

            % Generate a possible goal configuration.
            qGoal = q;

            qGoal(1:7) = ranges(:,1).' + ...
                rand(1,7) .* (ranges(:,2) - ranges(:,1)).';

            % Check collisions using the appropriate policy.
            if randomizeObstacle
                isColliding = checkCollision( ...
                    LoggedSignals.robot, qGoal, ...
                    "SkippedSelfCollisions", p.skipPairs);
            else
                isColliding = collision.checkRobotCollision( ...
                    LoggedSignals.robot, qGoal, ...
                    LoggedSignals.obstacle, p.skipPairs);
            end

            if isColliding
                continue;
            end

            % Obtain a reachable Cartesian target through FK.
            TGoal = kinematics.forwardKinematics( ...
                LoggedSignals.robot, qGoal, ...
                LoggedSignals.eeFrame);

            targetCandidate = TGoal(1:3,4);

            % Compare with the actual randomized start pose.
            if norm(targetCandidate - LoggedSignals.pEE) < ...
                    p.minTargetDistance
                continue;
            end

            LoggedSignals.target = targetCandidate;
            accepted = true;
            break;
        end

        assert(accepted, ...
            "Reset failed: no valid target within attempt limit.");
    end

    % Target must be a finite Cartesian position.
    assert(isequal(size(LoggedSignals.target), [3, 1]) && ...
           all(isfinite(LoggedSignals.target)), ...
           "Invalid target position.");

    %% 4. Randomized Spherical Obstacle

    if randomizeObstacle

        fractionRange = p.obstacleFractionRange;
        offsetRange = p.obstacleOffsetRange;
        maxAttempts = p.maxObstacleAttempts;

        assert(isequal(size(fractionRange), [1,2]) && ...
               all(isfinite(fractionRange)) && ...
               fractionRange(1) >= 0 && ...
               fractionRange(2) <= 1 && ...
               fractionRange(1) < fractionRange(2), ...
               "Invalid obstacle fraction range.");

        assert(isequal(size(offsetRange), [1,2]) && ...
               all(isfinite(offsetRange)) && ...
               offsetRange(1) >= 0 && ...
               offsetRange(1) <= offsetRange(2), ...
               "Invalid obstacle offset range.");

        assert(isscalar(maxAttempts) && ...
               isfinite(maxAttempts) && ...
               maxAttempts >= 1 && ...
               maxAttempts == floor(maxAttempts), ...
               "Invalid obstacle attempt limit.");

        % Start-to-target segment in Cartesian space.
        segment = LoggedSignals.target - LoggedSignals.pEE;
        segmentLength = norm(segment);

        assert(segmentLength > p.positionTolerance, ...
            "Start and target are too close.");

        direction = segment / segmentLength;

        % Construct an orthonormal perpendicular basis.
        referenceAxis = [0; 0; 1];

        if abs(dot(direction, referenceAxis)) > 0.95
            referenceAxis = [0; 1; 0];
        end

        lateral1 = cross(direction, referenceAxis);
        lateral1 = lateral1 / norm(lateral1);
        lateral2 = cross(direction, lateral1);

        accepted = false;

        for attempt = 1:maxAttempts

            % Position along the start-target segment.
            alpha = fractionRange(1) + ...
                rand * diff(fractionRange);

            % Perpendicular displacement.
            rho = offsetRange(1) + ...
                rand * diff(offsetRange);

            theta = 2*pi*rand;

            lateral = rho * ( ...
                cos(theta)*lateral1 + ...
                sin(theta)*lateral2);

            center = LoggedSignals.pEE + ...
                alpha*segment + lateral;

            % Create the final candidate obstacle.
            candidate = collisionSphere( ...
                LoggedSignals.obstacle.Radius);

            candidate.Pose = trvec2tform(center.');

            % Cheap Cartesian target-point clearance check.
            if norm(center - LoggedSignals.target) <= ...
                    candidate.Radius
                continue;
            end

            % Validate final obstacle against starting robot.
            if collision.checkRobotCollision( ...
                    LoggedSignals.robot, q, ...
                    candidate, p.skipPairs)
                continue;
            end

            % If a goal configuration was sampled, verify
            % the final obstacle also clears that configuration.
            if randomizeTarget
                if collision.checkRobotCollision( ...
                        LoggedSignals.robot, qGoal, ...
                        candidate, p.skipPairs)
                    continue;
                end
            end

            LoggedSignals.obstacle = candidate;
            accepted = true;
            break;
        end

        assert(accepted, ...
            "Reset failed: no valid obstacle within attempt limit.");
    end

    %% 5. Initialize Episode State

    LoggedSignals.stepCount = 0;
    LoggedSignals.failureCode = 0;

    %% 6. Construct Initial Observation

    obstaclePosition = LoggedSignals.obstacle.Pose(1:3,4);

    InitialObs = [
        q(1:7).';
        LoggedSignals.target - LoggedSignals.pEE;
        obstaclePosition;
        LoggedSignals.obstacle.Radius
    ];

end
