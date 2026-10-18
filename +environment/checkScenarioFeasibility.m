
function [isFeasible, routeSteps] = checkScenarioFeasibility( ...
    robot, qStart, qGoal, obstacle, skipPairs, ...
    maxIncrement, maxSteps, opts)
%CHECKSCENARIOFEASIBILITY Validate an obstacle-avoidance scenario.
%
% Uses RRT only for scenario validation, never for RL control.
% Keeps the two gripper joints fixed.
%
% A successful result demonstrates a discretely collision-free
% route within the action-increment and episode-step limits.
%
% Outputs:
%   isFeasible - True if a valid route was found and verified
%   routeSteps - Number of required joint-increment steps
%                NaN if validation failed

    isFeasible = false;
    routeSteps = NaN;

    assert(numel(qStart) == 9 && numel(qGoal) == 9, ...
        "Expected nine-joint Panda configurations.");

    assert(isequal(qStart(8:9), qGoal(8:9)), ...
        "Gripper positions must remain fixed.");

    %% 1. Create planner for the current obstacle

    planner = manipulatorRRT(robot, {obstacle});

    planner.SkippedSelfCollisions = skipPairs;
    planner.MaxIterations = opts.maxIterations;
    planner.MaxConnectionDistance = opts.maxConnectionDistance;
    planner.ValidationDistance = opts.validationDistance;

    %% 2. Search for a collision-free path

    [path, info] = plan(planner, qStart, qGoal);

    if ~info.IsPathFound || size(path, 1) < 2
        return;
    end

    %% 3. Enforce fixed gripper positions
    % The Panda model has nine nonfixed joints, but the RL
    % policy controls only the first seven. Project the
    % path onto the fixed-gripper configuration space.
    % Every projected configuration is checked again.

    path(:, 8:9) = repmat(qStart(8:9), size(path, 1), 1);

    %% 4. Validate executable joint increments

    totalSteps = 0;

    for i = 2:size(path, 1)

        qFrom = path(i-1, :);
        qTo = path(i, :);

        dq = qTo(1:7) - qFrom(1:7);

        % Each joint increment must be <= maxIncrement.
        nSteps = max(1, ceil(max(abs(dq)) / maxIncrement));

        totalSteps = totalSteps + nSteps;

        if totalSteps > maxSteps
            return;
        end

        % Recheck the whole arm with the gripper fixed.
        for k = 1:nSteps

            qTest = qFrom;
            qTest(1:7) = qFrom(1:7) + (k/nSteps)*dq;

            if collision.checkRobotCollision( ...
                    robot, qTest, obstacle, skipPairs)
                return;
            end
        end
    end

    %% 5. Accept verified route

    isFeasible = true;
    routeSteps = totalSteps;
end
