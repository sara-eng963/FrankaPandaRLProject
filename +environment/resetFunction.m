function [InitialObs, LoggedSignals] = resetFunction(initialState)
%RESETFUNCTION Initialize a Panda obstacle-avoidance RL episode.
% Samples optional start, target and sphere configurations. The Cartesian
% workspace constrains episode starts and targets, not intermediate motion.
% No IK or trajectory planning is used during reset.

    LoggedSignals = initialState;
    p = LoggedSignals.params;
    q = LoggedSignals.q;

    randomizeStart = isfield(p, "startJointRanges");
    randomizeTarget = isfield(p, "goalJointRanges");
    randomizeObstacle = isfield(p, "obstacleFractionRange");
    useWorkspaceBox = isfield(p, "workspaceBox");

    % Optional diagnostics are strictly opt-in. Standard RL resets retain
    % their original exception behavior and observation layout.
    recordDiagnostics = isfield(p, "recordScenarioDiagnostics") && ...
        isequal(p.recordScenarioDiagnostics, true);
    if recordDiagnostics
        diag = struct( ...
            "success", false, ...
            "failureStage", "", ...
            "attempts", 0, ...
            "targetPointRejects", 0, ...
            "startCollisionRejects", 0, ...
            "goalCollisionRejects", 0, ...
            "acceptedAttempt", 0, ...
            "candidateCenters", zeros(0,3), ...
            "candidateCodes", zeros(0,1,"uint8"));
    end

    assert(isequal(size(q), [1, 9]) && all(isfinite(q)), ...
        "Invalid initial joint configuration.");

    % An omitted workspaceBox preserves unrestricted diagnostic resets.
    if useWorkspaceBox
        workspaceBox = p.workspaceBox;
        assert(isequal(size(workspaceBox), [3, 2]) && ...
               all(isfinite(workspaceBox(:))) && ...
               all(workspaceBox(:,1) < workspaceBox(:,2)), ...
               "Invalid Cartesian workspace bounds.");
    end

    %% 1. Starting Configuration
    if randomizeStart
        ranges = p.startJointRanges;

        % Prevalidate sampling limits; candidates then need collision checks.
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
            qCandidate = q;
            qCandidate(1:7) = ranges(:,1).' + ...
                rand(1,7) .* (ranges(:,2) - ranges(:,1)).';

            % Ignore the placeholder sphere if reset will replace it.
            if randomizeObstacle
                isColliding = checkCollision( ...
                    LoggedSignals.robot, qCandidate, ...
                    "SkippedSelfCollisions", p.skipPairs);
            else
                isColliding = collision.checkRobotCollision( ...
                    LoggedSignals.robot, qCandidate, ...
                    LoggedSignals.obstacle, p.skipPairs);
            end

            if isColliding
                continue;
            end

            TStart = kinematics.forwardKinematics( ...
                LoggedSignals.robot, qCandidate, LoggedSignals.eeFrame);
            pCandidate = TStart(1:3,4);

            % Workspace limits apply to sampled initial EE positions.
            if useWorkspaceBox && ...
               any(pCandidate < workspaceBox(:,1) | ...
                   pCandidate > workspaceBox(:,2))
                continue;
            end

            q = qCandidate;
            pStart = pCandidate;
            accepted = true;
            break;
        end

        if ~accepted
            if recordDiagnostics
                diag.failureStage = "start";
                LoggedSignals.scenarioDiagnostics = diag;
                InitialObs = [];
                return;
            end
            error("Reset failed: no valid start within attempt limit.");
        end
    else
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

        assert(~isColliding, "Initial configuration is colliding.");

        TStart = kinematics.forwardKinematics( ...
            LoggedSignals.robot, q, LoggedSignals.eeFrame);
        pStart = TStart(1:3,4);

        if useWorkspaceBox
            assert(all(pStart >= workspaceBox(:,1) & ...
                       pStart <= workspaceBox(:,2)), ...
                   "Initial position is outside the workspace.");
        end
    end

    % Reuse the accepted FK result instead of calculating it twice.
    LoggedSignals.q = q;
    LoggedSignals.pEE = pStart;

    %% 2. Target Position
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
        % Validate the optional upper distance bound before sampling.
useMaxDistance = isfield(p, "maxTargetDistance");

if useMaxDistance
    assert(isscalar(p.maxTargetDistance) && ...
           isfinite(p.maxTargetDistance) && ...
           p.maxTargetDistance > p.minTargetDistance, ...
           "Invalid maximum target distance.");
end

        accepted = false;

        for attempt = 1:p.maxTargetAttempts
            qGoal = q;
            qGoal(1:7) = ranges(:,1).' + ...
                rand(1,7) .* (ranges(:,2) - ranges(:,1)).';

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

            % Sample joint space, then map to a reachable Cartesian target.
            TGoal = kinematics.forwardKinematics( ...
                LoggedSignals.robot, qGoal, LoggedSignals.eeFrame);
            targetCandidate = TGoal(1:3,4);

            if useWorkspaceBox && ...
               any(targetCandidate < workspaceBox(:,1) | ...
                   targetCandidate > workspaceBox(:,2))
                continue;
            end

        % Enforce the calibrated start-to-target distance range.
        targetDistance = norm(targetCandidate - pStart);
        
        if targetDistance < p.minTargetDistance || ...
                (useMaxDistance && targetDistance > p.maxTargetDistance)
            continue;
        end
            LoggedSignals.target = targetCandidate;

            % Retain the sampled goal configuration for environment diagnostics.
            % It is not included in the agent's observation.
            LoggedSignals.sampledGoalQ = qGoal;
            
            accepted = true;
            break;
        end

        if ~accepted
            if recordDiagnostics
                diag.failureStage = "target";
                LoggedSignals.scenarioDiagnostics = diag;
                InitialObs = [];
                return;
            end
            error("Reset failed: no valid target within attempt limit.");
        end
    end

    assert(isequal(size(LoggedSignals.target), [3, 1]) && ...
           all(isfinite(LoggedSignals.target)), ...
           "Invalid target position.");

    if useWorkspaceBox && ~randomizeTarget
        assert(all(LoggedSignals.target >= workspaceBox(:,1) & ...
                   LoggedSignals.target <= workspaceBox(:,2)), ...
               "Target position is outside the workspace.");
    end

    %% 3. Spherical Obstacle
    if randomizeObstacle
        fractionRange = p.obstacleFractionRange;
        offsetRange = p.obstacleOffsetRange;
        maxAttempts = p.maxObstacleAttempts;

        assert(isequal(size(fractionRange), [1, 2]) && ...
               all(isfinite(fractionRange)) && ...
               fractionRange(1) >= 0 && ...
               fractionRange(2) <= 1 && ...
               fractionRange(1) < fractionRange(2), ...
               "Invalid obstacle fraction range.");

        assert(isequal(size(offsetRange), [1, 2]) && ...
               all(isfinite(offsetRange)) && ...
               offsetRange(1) >= 0 && ...
               offsetRange(1) <= offsetRange(2), ...
               "Invalid obstacle offset range.");

        assert(isscalar(maxAttempts) && ...
               isfinite(maxAttempts) && ...
               maxAttempts >= 1 && ...
               maxAttempts == floor(maxAttempts), ...
               "Invalid obstacle attempt limit.");

        if recordDiagnostics
            % Codes: 1=target overlap, 2=start robot, 3=goal robot,
            % 4=accepted. Store centers so failures can be visualized.
            diag.candidateCenters = zeros(maxAttempts, 3);
            diag.candidateCodes = zeros(maxAttempts, 1, "uint8");
        end

        segment = LoggedSignals.target - pStart;
        segmentLength = norm(segment);

        assert(segmentLength > p.positionTolerance, ...
            "Start and target are too close.");

        direction = segment / segmentLength;

        % Orthonormal basis perpendicular to the start-target segment.
        referenceAxis = [0; 0; 1];
        if abs(dot(direction, referenceAxis)) > 0.95
            referenceAxis = [0; 1; 0];
        end

        lateral1 = cross(direction, referenceAxis);
        lateral1 = lateral1 / norm(lateral1);
        lateral2 = cross(direction, lateral1);

        accepted = false;

        for attempt = 1:maxAttempts
            alpha = fractionRange(1) + rand * diff(fractionRange);
            rho = offsetRange(1) + rand * diff(offsetRange);
            theta = 2*pi*rand;

            center = pStart + alpha*segment + ...
                rho*(cos(theta)*lateral1 + sin(theta)*lateral2);

            candidate = collisionSphere(LoggedSignals.obstacle.Radius);
            candidate.Pose = trvec2tform(center.');

            if recordDiagnostics
                diag.attempts = attempt;
                diag.candidateCenters(attempt,:) = center.';
            end

            % Reject spheres overlapping the goal point or endpoint poses.
            if norm(center - LoggedSignals.target) <= candidate.Radius
                if recordDiagnostics
                    diag.targetPointRejects = diag.targetPointRejects + 1;
                    diag.candidateCodes(attempt) = 1;
                end
                continue;
            end

            if collision.checkRobotCollision( ...
                    LoggedSignals.robot, q, candidate, p.skipPairs)
                if recordDiagnostics
                    diag.startCollisionRejects = diag.startCollisionRejects + 1;
                    diag.candidateCodes(attempt) = 2;
                end
                continue;
            end

            if randomizeTarget && collision.checkRobotCollision( ...
                    LoggedSignals.robot, qGoal, candidate, p.skipPairs)
                if recordDiagnostics
                    diag.goalCollisionRejects = diag.goalCollisionRejects + 1;
                    diag.candidateCodes(attempt) = 3;
                end
                continue;
            end

            LoggedSignals.obstacle = candidate;
            if recordDiagnostics
                diag.candidateCodes(attempt) = 4;
                diag.acceptedAttempt = attempt;
            end
            accepted = true;
            break;
        end

        if recordDiagnostics
            diag.candidateCenters = diag.candidateCenters(1:diag.attempts,:);
            diag.candidateCodes = diag.candidateCodes(1:diag.attempts);
        end
        if ~accepted
            if recordDiagnostics
                diag.failureStage = "obstacle";
                LoggedSignals.scenarioDiagnostics = diag;
                InitialObs = [];
                return;
            end
            error("Reset failed: no valid obstacle within attempt limit.");
        end
    end

    if recordDiagnostics
        diag.success = true;
        LoggedSignals.scenarioDiagnostics = diag;
    end

    %% 4. Episode State and Observation
    LoggedSignals.stepCount = 0;
    LoggedSignals.failureCode = 0;

    obstaclePosition = LoggedSignals.obstacle.Pose(1:3,4);
    InitialObs = [
        q(1:7).';
        LoggedSignals.target - pStart;
        obstaclePosition;
        LoggedSignals.obstacle.Radius
    ];
end
