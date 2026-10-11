function [NextObs, Reward, IsDone, LoggedSignals] = ...
    stepFunction(Action, LoggedSignals)
%STEPFUNCTION Execute one kinematic RL environment step.
%
% Inputs:
%   Action        - 7x1 joint increments [rad]
%   LoggedSignals - Robot state and simulation parameters
%
% Outputs:
%   NextObs       - 14x1 observation vector
%   Reward        - Scalar reward
%   IsDone        - Episode termination flag
%   LoggedSignals - Updated environment state
%
% Invalid actions terminate the episode without changing
% the last valid robot configuration.

    p = LoggedSignals.params;
    w = p.reward;
    action = Action(:).';

    assert(numel(action) == 7, ...
        "Expected seven joint increments.");

    LoggedSignals.stepCount = LoggedSignals.stepCount + 1;

    %% 1. Validate Action

    failureCode = 0;

    if any(~isfinite(action)) || ...
       any(abs(action) > p.maxIncrement(:).')
        failureCode = 1;
    end

    %% 2. Calculate Candidate Configuration

    qCandidate = LoggedSignals.q;

    if failureCode == 0
        qCandidate(1:7) = qCandidate(1:7) + action;

        qArm = qCandidate(1:7);

        if any(qArm < p.jointLimits(:,1).') || ...
           any(qArm > p.jointLimits(:,2).')
            failureCode = 2;
        end
    end

    %% 3. Check Collisions

    if failureCode == 0
        if collision.checkRobotCollision( ...
                LoggedSignals.robot, qCandidate, ...
                LoggedSignals.obstacle, p.skipPairs)
            failureCode = 3;
        end
    end

    %% 4. Calculate Reward and Termination

    if failureCode ~= 0
        Reward = -w.failure;
        IsDone = true;
    else
        T = kinematics.forwardKinematics( ...
            LoggedSignals.robot, qCandidate, ...
            LoggedSignals.eeFrame);

        newPosition = T(1:3,4);

        dOld = norm(LoggedSignals.target - LoggedSignals.pEE);
        dNew = norm(LoggedSignals.target - newPosition);

        LoggedSignals.q = qCandidate;
        LoggedSignals.pEE = newPosition;

        success = dNew <= p.positionTolerance;
        timeout = LoggedSignals.stepCount >= p.maxSteps;

        Reward = w.progress * (dOld - dNew) - w.step;

        if success
            Reward = Reward + w.success;
        elseif timeout
            Reward = Reward - w.failure;
            failureCode = 4;
        end

        IsDone = success || timeout;
    end

    LoggedSignals.failureCode = failureCode;

    %% 5. Generate Next Observation

    obstaclePosition = LoggedSignals.obstacle.Pose(1:3,4);

    NextObs = [
        LoggedSignals.q(1:7).';
        LoggedSignals.target - LoggedSignals.pEE;
        obstaclePosition;
        LoggedSignals.obstacle.Radius
    ];
end