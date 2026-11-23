function pandaEpisodeViewer(stateBefore, stateAfter, isDone)
%PANDAEPISODEVIEWER Animate one Franka Panda RL episode in MATLAB.
% =========================================================================
% MCT464 | Franka Panda reinforcement learning obstacle avoidance
%
% PURPOSE
%   Visualize the robot, the stationary spherical obstacle, the Cartesian
%   target and the actual end-effector path while a Simulink episode runs.
%   This is a display-only function: it does NOT calculate robot actions,
%   collision decisions, rewards, or changes to the environment state.
%
% HOW THIS CONNECTS TO SIMULINK
%   1. Save this file in <FrankaPandaRLProject>/simulink/.
%   2. Create a MATLAB Function block named Panda Visualizer and paste:
%
%       function passthrough = fcn(stateBefore,stateAfter,isDone)
%       %#codegen
%       coder.extrinsic('pandaEpisodeViewer');
%       pandaEpisodeViewer(stateBefore,stateAfter,isDone);
%       passthrough = stateAfter;
%       end
%
%   3. Connect stateBefore to the 19-element state BEFORE the step
%      function, stateAfter to its 19-element NextState output, and isDone
%      to its terminal flag. Connect passthrough to a Terminator block.
%   4. Use ordinary (normal-mode) Simulink simulation, not generated code.
%
% STATE VECTOR: THE EXISTING PANDA PROJECT CONTRACT (19 x 1)
%     1:7    seven arm joint positions [rad]
%     8:10   actual end-effector position [m]
%    11:13   target Cartesian position [m]
%    14:16   obstacle sphere center [m]
%    17      obstacle sphere radius [m]
%    18      current episode step count
%    19      failure code (0, 1, 2, 3, 4)
%
% WHY TWO STATES?
%   At the first transition, stateBefore supplies the exact starting EE
%   position, so the visible path includes the initial point. stateAfter
%   supplies the next position, which includes the final terminal step.
%
% TRAINING COST
%   Robot visualization is relatively expensive. Disable/comment out the
%   Panda Visualizer subsystem for large training experiments. Use it for
%   short tests or for evaluation simulations, exactly like Lab 3's
%   visualization subsystem. Display frames are deliberately decimated.
% =========================================================================

    % ---- User-adjustable visualization settings -------------------------
    drawEveryNSteps = 5;  % 1 = every RL step (slow); 5 = every fifth step
    showRateLimited = true;  % true skips redundant renderer refreshes
    % --------------------------------------------------------------------

    persistent robot fig ax previousStep previousState episodeNumber eePath

    % Ignore incomplete/invalid initialisation evaluations by Simulink.
    if ~isnumeric(stateBefore) || ~isnumeric(stateAfter) || ...
            numel(stateBefore) ~= 19 || numel(stateAfter) ~= 19 || ...
            any(~isfinite(stateBefore(:))) || any(~isfinite(stateAfter(:)))
        return
    end

    stateBefore = double(stateBefore(:));
    stateAfter  = double(stateAfter(:));
    stepNumber  = round(stateAfter(18));
    done        = logical(isDone);

    if isempty(robot)
        % Load once rather than re-importing robot meshes at every step.
        % The project uses DataFormat="row" and two finger joints held fixed.
        robot = loadrobot('frankaEmikaPanda', 'DataFormat', 'row');
    end

    if isempty(episodeNumber)
        episodeNumber = 0;
    end

    % A decreasing/repeated step number indicates a new Simulink episode.
    % The step counter resets to zero before an episode, then advances to 1.
    % Simulink may evaluate a block more than once at a single sample.
    % A repeated IDENTICAL state is not a new episode. An independently
    % randomized episode can still begin at the same step number.
    newEpisode = isempty(previousStep) || stepNumber < previousStep || ...
        (stepNumber == previousStep && ~isequal(stateAfter,previousState));
    if newEpisode
        episodeNumber = episodeNumber + 1;
        eePath = stateBefore(8:10).'; % exact starting point
    end
    previousStep = stepNumber;
    previousState = stateAfter;

    % Accumulate the TRUE end-effector path from the accepted simulator
    % states. Do not use an RRT path: RRT only validates scenario feasibility.
    eePosition = stateAfter(8:10).';
    if isempty(eePath) || any(eePosition ~= eePath(end,:))
        eePath(end+1,:) = eePosition;
    end

    % Plot the first frame and every Nth frame. Always plot terminal state.
    if ~(newEpisode || done || mod(stepNumber,drawEveryNSteps) == 0)
        return
    end

    if isempty(fig) || ~isgraphics(fig) || isempty(ax) || ~isgraphics(ax)
        fig = figure('Name','Panda RL | Episode Animation', ...
            'NumberTitle','off','Color','w', ...
            'Tag','PandaRLEpisodeViewer');
        ax = axes('Parent',fig);
        useFastUpdate = false;
    else
        useFastUpdate = ~newEpisode;
    end

    % The underlying RL policy commands 7 arm increments; fingers are
    % intentionally fixed at 0.04 m in the assignment's robot state.
    fullJointConfiguration = [stateAfter(1:7).' 0.04 0.04];

    if useFastUpdate
        show(robot,fullJointConfiguration, ...
            'Parent',ax,'FastUpdate',true,'PreservePlot',false, ...
            'Frames','off','Visuals','on','Collisions','off');
    else
        show(robot,fullJointConfiguration, ...
            'Parent',ax,'PreservePlot',false, ...
            'Frames','off','Visuals','on','Collisions','off');
    end

    % FastUpdate modifies the robot display. The overlays are re-created
    % after each frame to avoid stale/duplicated paths or sphere patches.
    oldOverlays = findobj(ax,'Tag','PandaRLOverlay');
    if ~isempty(oldOverlays)
        delete(oldOverlays);
    end
    hold(ax,'on');

    % The obstacle is a sphere in BASE coordinates, not a joint-space
    % constraint. It stays stationary during the episode.
    sphereCenter = stateAfter(14:16).';
    sphereRadius = stateAfter(17);
    [sx,sy,sz] = sphere(20);
    surf(ax,sphereCenter(1)+sphereRadius*sx, ...
            sphereCenter(2)+sphereRadius*sy, ...
            sphereCenter(3)+sphereRadius*sz, ...
            'FaceColor',[0.9 0.2 0.15],'FaceAlpha',0.35, ...
            'EdgeColor','none','Tag','PandaRLOverlay');

    % Plot the Cartesian trajectory that this policy ACTUALLY executed.
    plot3(ax,eePath(:,1),eePath(:,2),eePath(:,3), ...
        'Color',[0 0.45 0.85],'LineWidth',2, ...
        'Tag','PandaRLOverlay');
    plot3(ax,eePath(1,1),eePath(1,2),eePath(1,3), ...
        's','MarkerSize',8,'MarkerFaceColor',[0.1 0.65 0.3], ...
        'MarkerEdgeColor','k','Tag','PandaRLOverlay');
    plot3(ax,eePosition(1),eePosition(2),eePosition(3), ...
        '.','Color',[0 0.45 0.85],'MarkerSize',20, ...
        'Tag','PandaRLOverlay');

    target = stateAfter(11:13).';
    plot3(ax,target(1),target(2),target(3), ...
        'p','MarkerSize',14,'MarkerFaceColor',[1 0.75 0], ...
        'MarkerEdgeColor','k','Tag','PandaRLOverlay');

    % The last state already carries the failure code. In the repository,
    % code 0 + done = task success, and code 4 = time limit reached.
    if done
        switch round(stateAfter(19))
            case 0, outcome = 'TARGET REACHED';
            case 1, outcome = 'INVALID ACTION';
            case 2, outcome = 'JOINT LIMIT';
            case 3, outcome = 'COLLISION';
            case 4, outcome = 'TIMEOUT';
            otherwise, outcome = 'TERMINATED';
        end
    else
        outcome = 'IN PROGRESS';
    end

    goalError = norm(target-eePosition);
    title(ax,sprintf('Episode %d | Step %d | Goal error %.3f m | %s', ...
        episodeNumber,stepNumber,goalError,outcome), ...
        'Interpreter','none');
    xlabel(ax,'X [m]'); ylabel(ax,'Y [m]'); zlabel(ax,'Z [m]');
    grid(ax,'on'); axis(ax,'equal');
    xlim(ax,[-0.4 0.9]); ylim(ax,[-0.65 0.65]); zlim(ax,[0 1.2]);
    view(ax,135,25);
    hold(ax,'off');

    if showRateLimited
        drawnow limitrate
    else
        drawnow
    end
end
