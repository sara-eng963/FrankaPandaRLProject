
function [Observation, Reward, IsDone, NextState] = simulinkStep(Action, State)
%SIMULINKSTEP Interface between Simulink and the existing Panda environment.
% Reuses the original step function without duplicating robot algorithms.

    % Get the existing robot model and configuration.
    mdlWks = get_param("PandaRL", "ModelWorkspace");
logged = getVariable(mdlWks, "pandaTemplate");
    % Restore the current numerical episode state.
    logged.q(1:7) = State(1:7).';
    logged.pEE = State(8:10);
    logged.target = State(11:13);

    logged.obstacle = collisionSphere(State(17));
    logged.obstacle.Pose = trvec2tform(State(14:16).');

    logged.stepCount = State(18);
    logged.failureCode = State(19);

    % Execute the original tested environment logic.
    [Observation, Reward, IsDone, logged] = ...
        environment.stepFunction(Action, logged);

    % Return the next numerical state.
    NextState = [
        logged.q(1:7).';
        logged.pEE;
        logged.target;
        logged.obstacle.Pose(1:3,4);
        logged.obstacle.Radius;
        logged.stepCount;
        logged.failureCode
    ];
end
