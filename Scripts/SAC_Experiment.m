function result = SAC_Experiment(overrides)
%SAC_EXPERIMENT Evaluate by default; train only with explicit mode="train".
% result = SAC_Experiment(struct('mode',"evaluate"));
% Configuration is local to this function; no base-workspace inputs are used.
    cfg = struct('algorithm',"SAC",'curriculum',5,'mode',"evaluate", ...
        'startCheckpoint',"Training_Results/SAC/C04/SAC_C04_Agent.mat", ...
        'goalX',[0.03 0.04],'goalY',[-0.0125 0.005],'goalZ',0, ...
        'maxEpisodes',20,'evaluateEvery',2,'seed',2026, ...
        'extraGoals',zeros(0,3),'requireBaselineSuccess',true);
    if nargin<1, overrides=struct(); end
    names=fieldnames(overrides);
    for i=1:numel(names)
        assert(isfield(cfg,names{i}),'SAC:Config','Unknown configuration field: %s',names{i});
        cfg.(names{i})=overrides.(names{i});
    end
    assert(cfg.algorithm=="SAC" && any(cfg.mode==["evaluate","train"]),'Invalid algorithm/mode.');
    validateattributes(cfg.curriculum,{'double'},{'scalar','integer','positive'});
    validateattributes(cfg.maxEpisodes,{'double'},{'scalar','integer','positive'});
    validateattributes(cfg.evaluateEvery,{'double'},{'scalar','integer','positive'});
    validateattributes(cfg.seed,{'double'},{'scalar','integer','nonnegative','<=',2^32-1});
    validateattributes(cfg.extraGoals,{'double'},{'ncols',3,'finite'});
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'Scripts'));
    before=rng; guard=onCleanup(@() rng(before));
    rng(cfg.seed,'twister');
    initialRng=rng;
    [env,base]=SAC_Environment("build",cfg);
    provenance=struct('sourceCommit',"unknown",'checkpoint',cfg.startCheckpoint, ...
        'checkpointSHA256',"",'matlabVersion',string(version), ...
        'toolboxes',ver,'sourceHashes',sourceHashes(root));
    [status,revision]=system(sprintf('git -C "%s" rev-parse HEAD',root));
    if status==0, provenance.sourceCommit=string(strtrim(revision)); end
    if strlength(cfg.startCheckpoint)>0
        path=fullfile(root,cfg.startCheckpoint);
        assert(isfile(path),'SAC:Checkpoint','Missing checkpoint: %s',path);
        loaded=load(path,'agent');
        assert(isfield(loaded,'agent') && isa(loaded.agent,'rl.agent.rlSACAgent'), ...
            'Checkpoint must contain an rlSACAgent variable named agent.');
        agent=loaded.agent;
        provenance.checkpointSHA256=fileHash(path);
        verifyAgent(agent,env);
    else
        options=rlSACAgentOptions(SampleTime=0.05,DiscountFactor=0.995, ...
            ExperienceBufferLength=100000,MiniBatchSize=128,TargetSmoothFactor=1e-3);
        options.ActorOptimizerOptions.LearnRate=3e-5;
        co=options.CriticOptimizerOptions;
        for i=1:numel(co), co(i).LearnRate=3e-5; end
        options.CriticOptimizerOptions=co;
        options.EntropyWeightOptions.LearnRate=3e-5;
        options.NumWarmStartSteps=512;
        options.LearningFrequency=1;
        options.NumEpoch=1;
        options.MaxMiniBatchPerEpoch=1;
        options.PolicyUpdateFrequency=5;
        options.InfoToSave=struct('Optimizer',true,'Target',true, ...
            'PolicyState',true,'ExperienceBuffer',true);
        init=rlAgentInitializationOptions(NumHiddenUnit=128,UseRNN=false,Normalization="none");
        agent=rlSACAgent(getObservationInfo(env),getActionInfo(env),init,options);
    end
    cfg.agentOptions=agent.AgentOptions;
    [~,~,local]=evaluateTargets(agent,base,benchmarkGoals("local"));
    [~,~,workspace]=evaluateTargets(agent,base,benchmarkGoals("workspace"));
    [~,~,extra]=evaluateTargets(agent,base,cfg.extraGoals);
    baseline=struct('local',local,'workspace',workspace,'extra',extra);
    result=struct('config',cfg,'provenance',provenance,'baseline',baseline, ...
        'trainingStarted',false,'runDirectory',"");
    fprintf('Local: %d/14; reconstructed workspace: %d/25\n', ...
        sum(local.Success),sum(workspace.Success));
    if cfg.mode=="evaluate", return; end
    if strlength(cfg.startCheckpoint)>0 && cfg.requireBaselineSuccess
        assert(all(local.Success),'SAC:Regression','Starting policy failed the 14-target regression.');
    end
    % This branch is reached only by an explicit mode="train" request.
    parent=fullfile(root,'Training_Results','SAC',sprintf('C%02d',cfg.curriculum));
    runDirectory=reserveRun(parent);
    mkdir(fullfile(runDirectory,'checkpoints'));
    trainingOptions=rlTrainingOptions(MaxEpisodes=cfg.maxEpisodes,MaxStepsPerEpisode=400, ...
        StopTrainingCriteria="None",SaveAgentCriteria="None",UseParallel=false, ...
        StopOnError="on",ScoreAveragingWindowLength=5,Plots="training-progress",Verbose=false);
    trainingRng=rng;
    save(fullfile(runDirectory,'config.mat'),'cfg','provenance','initialRng','trainingRng','trainingOptions','-v7.3');
    save(fullfile(runDirectory,'starting.mat'),'agent','baseline','-v7.3');
    evaluator=rlCustomEvaluator(@(a,e,info) periodicEvaluation(a,e,info,base,runDirectory), ...
        EvaluationFrequency=cfg.evaluateEvery);
    trainingStatistics=train(agent,env,trainingOptions,Evaluator=evaluator);
    finalRng=rng;
    save(fullfile(runDirectory,'final.mat'),'agent','trainingStatistics','finalRng','-v7.3');
    % Select once from baseline + periodic snapshots, using local success rate.
    % Ties retain the earliest snapshot; selection criterion is explicit.
    bestRate=mean(local.Success); selected="starting.mat";
    checkpoints=dir(fullfile(runDirectory,'checkpoints','Evaluation_*.mat'));
    for i=1:numel(checkpoints)
        candidate=load(fullfile(checkpoints(i).folder,checkpoints(i).name),'statistic');
        if candidate.statistic>bestRate
            bestRate=candidate.statistic;
            selected=string(fullfile('checkpoints',checkpoints(i).name));
        end
    end
    [finalRate,~,finalEvaluation]=evaluateTargets(agent,base,benchmarkGoals("local"));
    if finalRate>bestRate, bestRate=finalRate; selected="final.mat"; end
    chosen=load(fullfile(runDirectory,selected),'agent');
    finalAgent=agent; agent=chosen.agent;
    save(fullfile(runDirectory,'best.mat'),'agent','bestRate','selected','-v7.3');
    agent=finalAgent; %#ok<NASGU> Retain the final agent after best selection.
    save(fullfile(runDirectory,'results.mat'),'baseline','finalEvaluation','selected','bestRate','provenance','-v7.3');
    result.trainingStarted=true;
    result.runDirectory=string(runDirectory);
end

function verifyAgent(agent,env)
    oi=getObservationInfo(agent); ai=getActionInfo(agent); o=agent.AgentOptions;
    assert(isequal(oi.Dimension,[14 1]) && isequal(ai.Dimension,[7 1]));
    assert(isequal(ai.LowerLimit,-ones(7,1)) && isequal(ai.UpperLimit,ones(7,1)));
    assert(isequal(getActionInfo(env).Dimension,ai.Dimension));
    assert(o.PolicyUpdateFrequency==5 && o.CriticUpdateFrequency==1);
    assert(o.ActorOptimizerOptions.LearnRate==3e-5 && all([o.CriticOptimizerOptions.LearnRate]==3e-5));
    assert(o.EntropyWeightOptions.LearnRate==3e-5 && o.SampleTime==0.05 && o.DiscountFactor==0.995);
    assert(o.MiniBatchSize==128 && o.ExperienceBufferLength==100000 && o.NumWarmStartSteps==512);
    assert(o.LearningFrequency==1 && o.NumEpoch==1 && o.MaxMiniBatchPerEpoch==1);
    assert(o.TargetSmoothFactor==1e-3 && o.TargetUpdateFrequency==1 && ~o.ResetExperienceBufferBeforeTraining);
    assert(all(structfun(@(x) logical(x),o.InfoToSave)),'Continuation state must be retained.');
end

function goals=benchmarkGoals(name)
    if name=="local"
        [x,y]=ndgrid([0.03 0.035 0.04],[-0.005 0 0.005]);
        goals=[x(:),y(:),zeros(9,1);0.045 -0.005 0;0.045 0.005 0; ...
            0.050 0 0;0.040 -0.010 0;0.040 0.010 0];
    else
        [x,y]=ndgrid([0.03 0.04 0.05 0.06 0.08],[-0.015 -0.01 -0.005 0 0.01]);
        goals=[x(:),y(:),zeros(25,1)];
    end
end

function [statistic,scores,data]=evaluateTargets(agent,base,goals)
    policy=getGreedyPolicy(agent);
    n=size(goals,1); scores=zeros(n,1);
    data=struct('Goals',goals,'Success',false(n,1),'FinalErrors',zeros(n,1), ...
        'FailureCodes',zeros(n,1),'Steps',zeros(n,1));
    for i=1:n
        state=base; state.target=base.pEE+goals(i,:).';
        [obs,state]=environment.resetFunction(state);
        for k=1:400
            action=getAction(policy,{obs});
            [obs,~,done,state]=SAC_Environment("step",action{1},state);
            if done, break; end
        end
        data.FinalErrors(i)=norm(obs(8:10));
        data.FailureCodes(i)=state.failureCode;
        data.Steps(i)=k;
        scores(i)=data.FinalErrors(i)<=0.02 && state.failureCode==0;
    end
    data.Success=logical(scores);
    statistic=mean(scores);
    data.OverallSuccessRate=statistic;
end

function [statistic,scores,data]=periodicEvaluation(agent,~,info,base,runDirectory)
    [statistic,scores,data]=evaluateTargets(agent,base,benchmarkGoals("local"));
    files=dir(fullfile(runDirectory,'checkpoints','Evaluation_*.mat'));
    destination=fullfile(runDirectory,'checkpoints',sprintf('Evaluation_%06d.mat',numel(files)+1));
    assert(~isfile(destination),'Refusing to overwrite checkpoint.');
    rngState=rng;
    save(destination,'agent','statistic','scores','data','info','rngState','-v7.3');
end

function folder=reserveRun(parent)
    if ~isfolder(parent), mkdir(parent); end
    % Java mkdir returns false if the directory already exists (atomic claim).
    for i=1:999999
        folder=fullfile(parent,sprintf('Run_%03d',i));
        directory=java.io.File(folder);
        if directory.mkdir(), return; end
        assert(isfolder(folder),'Cannot create run directory.');
    end
    error('SAC:RunDirectory','No free run directory.');
end

function hashes=sourceHashes(root)
    files=[dir(fullfile(root,'Scripts','SAC_*.m'));dir(fullfile(root,'Scripts','robot_setup.m')); ...
        dir(fullfile(root,'+environment','stepFunction.m'));dir(fullfile(root,'+environment','resetFunction.m')); ...
        dir(fullfile(root,'+kinematics','*.m'));dir(fullfile(root,'+collision','*.m'))];
    hashes=struct('file',{},'sha256',{});
    for i=1:numel(files)
        path=fullfile(files(i).folder,files(i).name);
        hashes(i)=struct('file',string(erase(path,[root filesep])),'sha256',fileHash(path));
    end
end

function hash=fileHash(path)
    fid=fopen(path,'rb'); assert(fid>=0,'Cannot read file.');
    guard=onCleanup(@() fclose(fid));
    digest=java.security.MessageDigest.getInstance('SHA-256');
    while ~feof(fid), digest.update(fread(fid,1048576,'*uint8')); end
    hash=string(lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[])));
end
