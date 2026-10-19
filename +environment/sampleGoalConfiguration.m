function [targetXYZ,qGoal,info] = sampleGoalConfiguration(robot,eeFrame,qStart,p,obstacle)
%SAMPLEGOALCONFIGURATION Cartesian-first, position-only IK goal validation.
% Configure p.ikSolver ONCE outside reset (inverseKinematics, joint limits
% enforced, automatic random restarts disabled). Required settings:
% workspaceBox, jointLimits, skipPairs, maxTargetAttempts, minTargetDistance,
% ikMaxInitialGuesses, ikPositionTolerance. maxTargetDistance is optional.
% Solver iteration/time limits belong to p.ikSolver.SolverParameters.
%
% Each target is drawn uniformly from workspaceBox, then distance-filtered.
% First IK guess is qStart; subsequent guesses sample the physical arm limits.
% Orientation weights are ZERO. Restored fixed fingers and every IK result
% are checked with FK, actual position error, bounds and exact collisions.
% Optional obstacle preserves static-obstacle endpoint checks in reset.
% Empty outputs mean bounded search exhaustion, NOT proven unreachability.
% Uses the caller's random stream exactly like the existing reset sampler;
% never seeds it. qGoal is a feasibility witness, never an RL action.
    arguments
        robot (1,1) rigidBodyTree
        eeFrame (1,1) string
        qStart (1,9) double {mustBeFinite}
        p (1,1) struct
        obstacle = []
    end
    timer = tic;
    box = p.workspaceBox;
    limits = p.jointLimits;
    validateattributes(box,{'double'},{'size',[3 2],'finite'});
    validateattributes(limits,{'double'},{'size',[7 2],'finite'});
    validateattributes(p.maxTargetAttempts,{'double'},{'scalar','integer','positive'});
    validateattributes(p.ikMaxInitialGuesses,{'double'},{'scalar','integer','positive'});
    validateattributes(p.ikPositionTolerance,{'double'},{'scalar','finite','positive'});
    if isfield(p,'positionTolerance')
        assert(p.ikPositionTolerance<=p.positionTolerance, ...
            'IK tolerance must not exceed the task position tolerance.');
    end
    validateattributes(p.minTargetDistance,{'double'},{'scalar','finite','nonnegative'});
    maxDistance = Inf;
    if isfield(p,'maxTargetDistance'), maxDistance = p.maxTargetDistance; end
    assert(all(box(:,1)<box(:,2)) && all(limits(:,1)<limits(:,2)));
    assert(isscalar(maxDistance) && maxDistance>p.minTargetDistance);
    assert(isa(p.ikSolver,'inverseKinematics'),'Provide a reusable IK solver.');
    settings = p.ikSolver.SolverParameters;
    assert(settings.EnforceJointLimits && ~settings.AllowRandomRestart, ...
        'IK must enforce joint limits and disable internal random restarts.');
    assert(all(qStart(1:7)>=limits(:,1).' & qStart(1:7)<=limits(:,2).'));
    fingerNames = ["panda_leftfinger","panda_rightfinger"];
    for finger = 1:2
        body = getBody(robot,fingerNames(finger));
        fingerLimits = body.Joint.PositionLimits;
        assert(qStart(7+finger)>=fingerLimits(1) && ...
            qStart(7+finger)<=fingerLimits(2),'Invalid fixed gripper position.');
    end
    TStart = kinematics.forwardKinematics(robot,qStart,eeFrame);
    pStart = TStart(1:3,4);
    weights = [0 0 0 1 1 1];
    info = struct('success',false,'targetAttempts',0,'distanceRejects',0, ...
        'targetIKRejects',0,'ikCalls',0,'ikFailures',0, ...
        'solverNonconverged',0,'positionRejects',0,'jointLimitRejects',0, ...
        'workspaceRejects',0,'collisionRejects',0,'positionError',NaN, ...
        'maxIterationsUsed',0,'solverTimeLimits',0,'seconds',0);
    targetXYZ = []; qGoal = [];
    for attempt = 1:p.maxTargetAttempts
        info.targetAttempts = attempt;
        xyz = box(:,1)+rand(3,1).*diff(box,1,2);
        distance = norm(xyz-pStart);
        if distance<p.minTargetDistance || distance>maxDistance
            info.distanceRejects = info.distanceRejects+1;
            continue;
        end
        desired = eye(4);
        desired(1:3,4) = xyz; % Rotation has zero weight; no orientation target.
        for guessIndex = 1:p.ikMaxInitialGuesses
            guess = qStart;
            if guessIndex>1
                guess(1:7) = limits(:,1).'+rand(1,7).*diff(limits,1,2).';
            end
            [candidate,solverInfo] = p.ikSolver(eeFrame,desired,weights,guess);
            info.ikCalls = info.ikCalls+1;
            info.maxIterationsUsed = max(info.maxIterationsUsed,solverInfo.Iterations);
            info.solverNonconverged = info.solverNonconverged+ ...
                ~strcmp(solverInfo.Status,'success');
            % Robotics System Toolbox IK ExitFlag 3: time limit exceeded.
            info.solverTimeLimits = info.solverTimeLimits+(solverInfo.ExitFlag==3);
            info.ikFailures = info.ikFailures+1;
            if ~isequal(size(candidate),[1 9]) || any(~isfinite(candidate)) || ...
                    any(candidate(1:7)<limits(:,1).' | candidate(1:7)>limits(:,2).')
                info.jointLimitRejects = info.jointLimitRejects+1;
                continue;
            end
            candidate(8:9) = qStart(8:9);
            T = kinematics.forwardKinematics(robot,candidate,eeFrame);
            actualXYZ = T(1:3,4);
            error = norm(actualXYZ-xyz);
            if error>p.ikPositionTolerance
                info.positionRejects = info.positionRejects+1;
                continue;
            end
            if any(actualXYZ<box(:,1) | actualXYZ>box(:,2))
                info.workspaceRejects = info.workspaceRejects+1;
                continue;
            end
            actualDistance = norm(actualXYZ-pStart);
            if actualDistance<p.minTargetDistance || actualDistance>maxDistance
                info.positionRejects = info.positionRejects+1;
                continue;
            end
            if isempty(obstacle)
                colliding = checkCollision(robot,candidate, ...
                    'SkippedSelfCollisions',p.skipPairs,'Exhaustive','off');
            else
                colliding = collision.checkRobotCollision(robot,candidate,obstacle,p.skipPairs);
            end
            if colliding
                info.collisionRejects = info.collisionRejects+1;
                continue;
            end
            targetXYZ = xyz;
            qGoal = candidate;
            info.ikFailures = info.ikFailures-1;
            info.positionError = error;
            info.success = true;
            break;
        end
        if info.success, break; end
        info.targetIKRejects = info.targetIKRejects+1;
    end
    info.seconds = toc(timer);
end
