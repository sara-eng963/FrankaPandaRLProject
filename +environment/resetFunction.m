function [InitialObs, LoggedSignals, status] = resetFunction(initialState)
%RESETFUNCTION Initialize a Panda obstacle-avoidance RL episode.
% Samples optional start, target and sphere configurations. The Cartesian
% workspace constrains episode starts and targets, not intermediate motion.
% Cartesian goals are validated with position-only IK; RRT validates scenarios.
% Neither solver supplies actions to the RL agent or runs in stepFunction.
% Optional third output reports expected bounded sampling exhaustion without
% throwing or enabling detailed diagnostics. Unexpected errors still propagate.
% Existing one/two-output callers retain their original failure behavior.

    LoggedSignals = initialState;
    p = LoggedSignals.params;
    q = LoggedSignals.q;
    recordStatus = nargout >= 3;
    if recordStatus
        status = struct('success',false,'failureStage',"", ...
            'obstacleSamplingMode',"",'obstacleRadius',NaN,'targetDistance',NaN);
    end

    randomizeStart = isfield(p, "startJointRanges");
    randomizeTarget = isfield(p, "ikSolver");
    assert(~isfield(p,"goalJointRanges") || randomizeTarget, ...
        'goalJointRanges no longer enables goal randomization; configure ikSolver.');
    randomizeObstacle = isfield(p, "obstacleFractionRange");
    useWorkspaceBox = isfield(p, "workspaceBox");
    useRRT = isfield(p, "rrt") && p.rrt.enabled;

    % Optional diagnostics are strictly opt-in. Standard RL resets retain
    % their original exception behavior and observation layout.
    recordDiagnostics = isfield(p, "recordScenarioDiagnostics") && ...
        isequal(p.recordScenarioDiagnostics, true);
    if recordDiagnostics
        diag = struct( ...
            "success", false, ...
            "startAttempts", 0, ...
            "startSeconds", 0, ...
            "startSampleCollisionRejects", 0, ...
            "startSampleWorkspaceRejects", 0, ...
            "targetIKRejects", 0, ...
            "ikCalls", 0, ...
            "ikFailures", 0, ...
            "ikSolverNonconverged", 0, ...
            "ikTimeLimits", 0, ...
            "goalPositionError", NaN, ...
            "targetAttempts", 0, ...
            "targetSeconds", 0, ...
            "targetCollisionRejects", 0, ...
            "targetWorkspaceRejects", 0, ...
            "targetDistanceRejects", 0, ...
            "rrtAttempts", 0, ...
            "rrtSeconds", 0, ...
            "rrtSetupSeconds", 0, ...
            "rrtPlanSeconds", 0, ...
            "rrtVerifySeconds", 0, ...
            "rrtCollisionChecks", 0, ...
            "obstacleSamplingMode", "", ...
            "armAnchorsGenerated", 0, ...
            "armAnchorsRejected", 0, ...
            "armAnchorsRetained", 0, ...
            "armNoFeasibleAnchors", false, ...
            "obstacleRadius", NaN, ...
            "failureStage", "", ...
            "attempts", 0, ...
            "targetPointRejects", 0, ...
            "workspaceRejects", 0, ...
            "startCollisionRejects", 0, ...
            "goalCollisionRejects", 0, ...
            "acceptedAttempt", 0, ...
            "candidateCenters", zeros(0,3), ...
            "rrtRejects", 0, ...
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
        if recordDiagnostics
            startTimer = tic;
        end

        for attempt = 1:p.maxResetAttempts
            qCandidate = q;
            qCandidate(1:7) = ranges(:,1).' + ...
                rand(1,7) .* (ranges(:,2) - ranges(:,1)).';
            if recordDiagnostics
                diag.startAttempts = attempt;
            end

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
                if recordDiagnostics
                    diag.startSampleCollisionRejects = diag.startSampleCollisionRejects+1;
                end
                continue;
            end

            TStart = kinematics.forwardKinematics( ...
                LoggedSignals.robot, qCandidate, LoggedSignals.eeFrame);
            pCandidate = TStart(1:3,4);

            % Workspace limits apply to sampled initial EE positions.
            if useWorkspaceBox && ...
               any(pCandidate < workspaceBox(:,1) | ...
                   pCandidate > workspaceBox(:,2))
                if recordDiagnostics
                    diag.startSampleWorkspaceRejects = diag.startSampleWorkspaceRejects+1;
                end
                continue;
            end

            q = qCandidate;
            pStart = pCandidate;
            accepted = true;
            break;
        end

        if recordDiagnostics
            diag.startSeconds = toc(startTimer);
        end

        if ~accepted
            if recordStatus, status.failureStage="start"; end
            if recordDiagnostics
                diag.failureStage = "start";
                LoggedSignals.scenarioDiagnostics = diag;
                InitialObs = [];
                return;
            end
            if recordStatus
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

    %% 2. Cartesian target feasibility (solver configured once by caller)
    if randomizeTarget
        assert(useWorkspaceBox,'Cartesian goal sampling requires workspaceBox.');
        assert(p.minTargetDistance>p.positionTolerance, ...
            'Minimum target distance must exceed the position tolerance.');
        if randomizeObstacle
            [targetCandidate,qGoal,g] = environment.sampleGoalConfiguration( ...
                LoggedSignals.robot,LoggedSignals.eeFrame,q,p);
        else
            [targetCandidate,qGoal,g] = environment.sampleGoalConfiguration( ...
                LoggedSignals.robot,LoggedSignals.eeFrame,q,p,LoggedSignals.obstacle);
        end
        if recordDiagnostics
            diag.targetAttempts = g.targetAttempts;
            diag.targetSeconds = g.seconds;
            diag.targetDistanceRejects = g.distanceRejects;
            diag.targetIKRejects = g.targetIKRejects;
            diag.targetCollisionRejects = g.collisionRejects;
            diag.targetWorkspaceRejects = g.workspaceRejects;
            diag.ikCalls = g.ikCalls;
            diag.ikFailures = g.ikFailures;
            diag.ikSolverNonconverged = g.solverNonconverged;
            diag.ikTimeLimits = g.solverTimeLimits;
            diag.goalPositionError = g.positionError;
        end
        if ~g.success
            if recordStatus, status.failureStage="target"; end
            if recordDiagnostics
                diag.failureStage = "target";
                LoggedSignals.scenarioDiagnostics = diag;
                InitialObs = [];
                return;
            end
            if recordStatus
                InitialObs = [];
                return;
            end
            error("Reset failed: no valid target within attempt limit.");
        end
        LoggedSignals.target = targetCandidate;
        % Feasibility witness only: excluded from the 14D RL observation.
        LoggedSignals.sampledGoalQ = qGoal;
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
        samplingMode = "ee_tube";
        if isfield(p, "obstacleSamplingMode")
            samplingMode = string(p.obstacleSamplingMode);
        end
        assert(isscalar(samplingMode) && ...
            any(samplingMode == ["ee_tube", "arm_sweep", "mixed"]), ...
            "Invalid obstacle sampling mode.");
        armSamplingProbability = 0.5;
        if isfield(p, "armSamplingProbability")
            armSamplingProbability = p.armSamplingProbability;
        end
        validateattributes(armSamplingProbability, {'numeric'}, ...
            {'scalar', 'real', 'finite', '>=', 0, '<=', 1});
        if samplingMode ~= "ee_tube"
            assert(randomizeTarget, ...
                "Arm-sweep sampling requires a sampled IK goal configuration.");
        end

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

       
%% Sample obstacle radius once per episode
% If no radius range is provided, preserve fixed-radius behavior.

obstacleRadius = LoggedSignals.obstacle.Radius;

if isfield(p, "obstacleRadiusRange")
    radiusRange = p.obstacleRadiusRange;

    assert(isnumeric(radiusRange) && ...
           isequal(size(radiusRange), [1, 2]) && ...
           all(isfinite(radiusRange)) && ...
           radiusRange(1) > 0 && ...
           radiusRange(1) < radiusRange(2), ...
           "Invalid obstacle radius range.");

    obstacleRadius = radiusRange(1) + ...
        rand * (radiusRange(2) - radiusRange(1));
end
if recordDiagnostics
    diag.obstacleRadius = obstacleRadius;
end
if recordStatus
    status.obstacleRadius = obstacleRadius;
    status.targetDistance = norm(LoggedSignals.target-pStart);
end

        % Select once, after start/goal and radius sampling. EE-tube consumes
        % exactly its original random draws; only mixed draws a mode choice.
        if samplingMode == "mixed"
            if rand < armSamplingProbability
                samplingMode = "arm_sweep";
            else
                samplingMode = "ee_tube";
            end
        end
        if recordDiagnostics
            diag.obstacleSamplingMode = samplingMode;
        end
        if recordStatus, status.obstacleSamplingMode=samplingMode; end

        if samplingMode == "arm_sweep"
            % Geometric link-frame approximations only, not collision volumes
            % or a feasible route. Never supplied as actions to the RL agent.
            fractions = [0.4 0.5 0.6];
            if isfield(p, "armSweepFractions")
                fractions = p.armSweepFractions;
            end
            validateattributes(fractions, {'double'}, ...
                {'vector', 'nonempty', 'real', 'finite', '>', 0, '<', 1});
            linkNames = ["panda_link4", "panda_link5", ...
                         "panda_link6", "panda_link7"];
            anchorsPerSample = 2*numel(linkNames)-1;
            armAnchors = zeros(3, numel(fractions)*anchorsPerSample);
            for sampleIndex = 1:numel(fractions)
                qNominal = q;
                qNominal(1:7) = q(1:7) + ...
                    fractions(sampleIndex)*(qGoal(1:7)-q(1:7));
                linkPositions = zeros(3, numel(linkNames));
                for linkIndex = 1:numel(linkNames)
                    TLink = getTransform(LoggedSignals.robot, qNominal, ...
                        linkNames(linkIndex));
                    linkPositions(:,linkIndex) = TLink(1:3,4);
                end
                % Origins plus segment midpoints, with no additional FK.
                indices = (sampleIndex-1)*anchorsPerSample+(1:anchorsPerSample);
                armAnchors(:,indices) = [linkPositions, ...
                    (linkPositions(:,1:end-1)+linkPositions(:,2:end))/2];
            end
        end

        if recordDiagnostics
            % Codes: 1=target overlap, 2=start robot, 3=goal robot,
            % 4=accepted, 5=outside workspace, 6=RRT.
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

% Radius-aware bounds for sphere centers.
% The entire sphere must remain inside the workspace.
if useWorkspaceBox
    centerMin = workspaceBox(:,1) + obstacleRadius;
    centerMax = workspaceBox(:,2) - obstacleRadius;

    assert(all(centerMin <= centerMax), ...
        "Sphere is too large to fit inside the workspace.");
end

if samplingMode == "arm_sweep"
    % IK goal sampling already requires workspaceBox. A box intersects the
    % allowed offset shell iff its nearest distance is <= the outer radius
    % and its farthest distance is >= the inner radius. Keep outside anchors
    % whenever they can reach that intersection; no collision approximation.
    armNearest = min(max(armAnchors,centerMin),centerMax);
    armFarthest = centerMin + (centerMax-centerMin).* ...
        (abs(centerMax-armAnchors)>=abs(centerMin-armAnchors));
    nearestDistances = sqrt(sum((armNearest-armAnchors).^2,1));
    farthestDistances = sqrt(sum((armFarthest-armAnchors).^2,1));
    feasibleAnchors = nearestDistances<=offsetRange(2) & ...
        farthestDistances>=offsetRange(1);
    if recordDiagnostics
        diag.armAnchorsGenerated = size(armAnchors,2);
        diag.armAnchorsRetained = nnz(feasibleAnchors);
        diag.armAnchorsRejected = nnz(~feasibleAnchors);
        if isfield(p,"recordObstacleSamplingGeometry") && ...
                isequal(p.recordObstacleSamplingGeometry,true)
            % Opt-in offline inspection only: retain the exact pre-filter data.
            diag.armSamplingGeometry = struct('rawAnchors',armAnchors, ...
                'feasibleMask',feasibleAnchors, ...
                'midpointMask',repmat([false(1,4) true(1,3)],1,numel(fractions)), ...
                'fractions',fractions);
        end
    end
    if ~any(feasibleAnchors)
        if recordStatus, status.failureStage="obstacle"; end
        if recordDiagnostics
            diag.armNoFeasibleAnchors = true;
            diag.failureStage = "obstacle";
            diag.candidateCenters = zeros(0,3);
            diag.candidateCodes = zeros(0,1,"uint8");
            LoggedSignals.scenarioDiagnostics = diag;
            InitialObs = [];
            return;
        end
        if recordStatus
            InitialObs = [];
            return;
        end
        error("Reset failed: no geometrically feasible arm-sweep anchors.");
    end
    armAnchors = armAnchors(:,feasibleAnchors);
    armNearest = armNearest(:,feasibleAnchors);
    armFarthest = armFarthest(:,feasibleAnchors);
end


        accepted = false;


for attempt = 1:maxAttempts
    if recordDiagnostics
        diag.attempts = attempt;
    end

    if samplingMode == "arm_sweep"
        anchorIndex = randi(size(armAnchors,2));
        anchor = armAnchors(:,anchorIndex);
        lateralDir = armSweepDirection(anchor, armNearest(:,anchorIndex), ...
            armFarthest(:,anchorIndex), centerMin, centerMax, offsetRange);
    else
        % 1. Random position along the start-target segment.
        alpha = fractionRange(1) + rand * diff(fractionRange);
        theta = 2*pi*rand;

        anchor = pStart + alpha*segment;

        % Random perpendicular direction.
        lateralDir = cos(theta)*lateral1 + sin(theta)*lateral2;
    end

    % 2. Start with the configured lateral-offset range.
    rhoMin = offsetRange(1);
    rhoMax = offsetRange(2);

    % 3. Restrict the offset using the valid center bounds.
    if useWorkspaceBox
        for axis = 1:3

            if abs(lateralDir(axis)) < 1e-12
                % No movement along this coordinate.
                if anchor(axis) < centerMin(axis) || ...
                   anchor(axis) > centerMax(axis)
                    rhoMin = Inf;
                    break;
                end
            else
                % Allowed lateral offsets along this axis.
                limits = ([centerMin(axis), centerMax(axis)] ...
                          - anchor(axis)) / lateralDir(axis);

                rhoMin = max(rhoMin, min(limits));
                rhoMax = min(rhoMax, max(limits));
            end
        end

        % This direction has no valid lateral offset.
        if rhoMin > rhoMax
            if recordDiagnostics
                diag.workspaceRejects = ...
                    diag.workspaceRejects + 1;
                diag.candidateCodes(attempt) = 5;

                % Record a nominal rejected point for plotting.
                diag.candidateCenters(attempt,:) = ...
                    (anchor + offsetRange(1)*lateralDir).';
            end
            continue;
        end
    end

    % 4. Sample only from the admissible offset interval.
    rho = rhoMin + rand*(rhoMax - rhoMin);
    center = anchor + rho*lateralDir;

    if recordDiagnostics
        diag.candidateCenters(attempt,:) = center.';
    end

    % Defensive validation: every constructed sphere must fit.
    if useWorkspaceBox && any( ...
            center - obstacleRadius < workspaceBox(:,1) - 1e-12 | ...
            center + obstacleRadius > workspaceBox(:,2) + 1e-12)
        error("Radius-aware sampler produced an invalid center.");
    end
            candidate = collisionSphere(obstacleRadius);
            candidate.Pose = trvec2tform(center.');

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

            
%% RRT Feasibility Validation

if useRRT

    assert(randomizeTarget, ...
        "RRT validation requires a sampled goal configuration.");

    if recordDiagnostics
        diag.rrtAttempts = diag.rrtAttempts+1;
        rrtTimer = tic;
        [feasible, routeSteps, rrtStats] = ...
            environment.checkScenarioFeasibility( ...
                LoggedSignals.robot, q, LoggedSignals.sampledGoalQ, ...
                candidate, p.skipPairs, p.maxIncrement, p.maxSteps, p.rrt);
        diag.rrtSeconds = diag.rrtSeconds+toc(rrtTimer);
        diag.rrtSetupSeconds = diag.rrtSetupSeconds+rrtStats.setupSeconds;
        diag.rrtPlanSeconds = diag.rrtPlanSeconds+rrtStats.planningSeconds;
        diag.rrtVerifySeconds = diag.rrtVerifySeconds+rrtStats.verificationSeconds;
        diag.rrtCollisionChecks = diag.rrtCollisionChecks+rrtStats.collisionChecks;
    else
        [feasible, routeSteps] = ...
            environment.checkScenarioFeasibility( ...
                LoggedSignals.robot, q, LoggedSignals.sampledGoalQ, ...
                candidate, p.skipPairs, p.maxIncrement, p.maxSteps, p.rrt);
    end

    if ~feasible
        if recordDiagnostics
            diag.rrtRejects = diag.rrtRejects + 1;
            diag.candidateCodes(attempt) = 6;
        end
        continue;
    end

    % Diagnostic information only.
    % The RL observation does not contain the planned path.
    LoggedSignals.rrtRouteSteps = routeSteps;
end

%% Accept Validated Scenario

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
            if recordStatus, status.failureStage="obstacle"; end
            if recordDiagnostics
                diag.failureStage = "obstacle";
                LoggedSignals.scenarioDiagnostics = diag;
                InitialObs = [];
                return;
            end
            if recordStatus
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
    if recordStatus, status.success=true; end
end

function direction = armSweepDirection(anchor, nearest, farthest, ...
        centerMin, centerMax, offsetRange)
% Choose a direction through a reachable box point, without rejection loops.
% Along the segment from the nearest box point to any other box point,
% distance from the anchor is monotone. Randomize the endpoint and a reachable
% radius; the caller then samples its usual clipped offset along this ray.
% This intentionally biases directions toward the workspace, not uniformly
% over the sphere. Segment interiors preserve spatial variation in the box.
    endpoint = centerMin + rand(3,1).*(centerMax-centerMin);
    nearestDistance = norm(nearest-anchor);
    lowerRadius = max(offsetRange(1),nearestDistance);
    if norm(endpoint-anchor)<lowerRadius
        endpoint = farthest;
    end
    upperRadius = min(offsetRange(2),norm(endpoint-anchor));
    radius = lowerRadius + rand*max(0,upperRadius-lowerRadius);
    segment = endpoint-nearest;
    segmentSquared = dot(segment,segment);
    b = dot(nearest-anchor,segment);
    delta = max(0,radius^2-nearestDistance^2);
    denominator = b + sqrt(max(0,b^2+segmentSquared*delta));
    fraction = 0;
    if denominator>0
        % Stable positive root of the line/sphere intersection equation.
        fraction = min(1,max(0,delta/denominator));
    end
    direction = nearest + fraction*segment - anchor;
    if norm(direction)==0
        direction = farthest-anchor;
        if norm(direction)==0
            % Degenerate zero-offset, point-sized box: any ray admits rho=0.
            direction = [1;0;0];
        end
    end
    direction = direction/norm(direction);
end
