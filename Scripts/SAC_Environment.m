function varargout = SAC_Environment(operation, varargin)
%SAC_ENVIRONMENT SAC reaching layer over the unchanged validated environment.
% [env,base] = SAC_Environment("build",cfg)
% [obs,reward,done,state] = SAC_Environment("step",action,state)
% [obs,state] = SAC_Environment("reset",base,cfg)
    switch string(operation)
        case "build"
            cfg = varargin{1};
            base = robotTemplate();
            validateattributes(cfg.goalX,{'double'},{'size',[1 2],'finite'});
            validateattributes(cfg.goalY,{'double'},{'size',[1 2],'finite'});
            validateattributes(cfg.goalZ,{'double'},{'scalar','finite'});
            assert(all([diff(cfg.goalX),diff(cfg.goalY)]>=0),'Invalid goal ranges.');
            % Both extrema must stay in the original Cartesian workspace.
            lo = base.pEE + [cfg.goalX(1);cfg.goalY(1);cfg.goalZ];
            hi = base.pEE + [cfg.goalX(2);cfg.goalY(2);cfg.goalZ];
            assert(all(lo>=base.params.workspaceBox(:,1)) && ...
                all(hi<=base.params.workspaceBox(:,2)),'Goals outside workspace.');
            base.target = base.pEE + [cfg.goalX(1);cfg.goalY(1);cfg.goalZ];
            obsInfo = rlNumericSpec([14 1],Name="PandaObservations");
            actInfo = rlNumericSpec([7 1],Name="NormalizedJointIncrements", ...
                LowerLimit=-ones(7,1),UpperLimit=ones(7,1));
            env = rlFunctionEnv(obsInfo,actInfo,@normalizedStep,@() nearbyReset(base,cfg));
            varargout = {env,base};
        case "step"
            [varargout{1:nargout}] = normalizedStep(varargin{:});
        case "reset"
            [varargout{1:nargout}] = nearbyReset(varargin{:});
        otherwise
            error('SAC:Operation','Unknown environment operation.');
    end
end

function base = robotTemplate()
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root);
    robotSetupVisualize = false; %#ok<NASGU> Used by robot_setup script.
    run(fullfile(root,'Scripts','robot_setup.m'));
    p = struct('jointLimits',jointLimits,'skipPairs',{skipPairs}, ...
        'maxIncrement',cfg.maxIncrement,'maxSteps',cfg.maxSteps, ...
        'positionTolerance',cfg.posTol,'workspaceBox',cfg.box);
    p.reward = struct('progress',20,'step',0.01,'success',10,'failure',10, ...
        'jointLimitMarginRad',0.20,'jointLimitWeight',0.02);
    obstacle = collisionSphere(0.03);
    obstacle.Pose = trvec2tform([3 3 3]);
    base = struct('robot',robot,'q',qReference,'eeFrame',eeFrame, ...
        'pEE',pReady,'target',pReady+[0.04;0;0],'obstacle',obstacle, ...
        'params',p,'stepCount',0,'failureCode',0);
end

function [obs,state] = nearbyReset(base,cfg)
    state = base;
    % Same two uniform random draws as the historical nearby reset.
    x = cfg.goalX(1) + rand*diff(cfg.goalX);
    y = cfg.goalY(1) + rand*diff(cfg.goalY);
    state.target = base.pEE + [x;y;cfg.goalZ];
    [obs,state] = environment.resetFunction(state);
end

function [obs,reward,done,state] = normalizedStep(action,state)
    assert(numel(action)==7,'Expected seven normalized joint actions.');
    scale = state.params.maxIncrement;
    if isscalar(scale), scale=repmat(scale,7,1); end
    assert(numel(scale)==7,'Expected scalar or seven-entry increment limit.');
    [obs,reward,done,state] = environment.stepFunction(action(:).*scale(:),state);
    if state.failureCode~=0, return; end
    q = state.q(1:7).';
    limits = state.params.jointLimits;
    nearest = min(q-limits(:,1),limits(:,2)-q);
    w = state.params.reward;
    reward = reward-w.jointLimitWeight*sum(max(0,1-nearest/w.jointLimitMarginRad).^2);
end
