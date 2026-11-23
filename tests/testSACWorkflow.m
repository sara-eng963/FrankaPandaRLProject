function report = testSACWorkflow()
%TESTSACWORKFLOW Non-training SAC and protected-policy regression checks.
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'Scripts'));
    oldRng=rng; guard=onCleanup(@() rng(oldRng));
    cfg=struct('goalX',[0.03 0.04],'goalY',[-0.0125 0.005],'goalZ',0);
    [env,base]=SAC_Environment("build",cfg);
    validateEnvironment(env);
    ai=getActionInfo(env);
    assert(isequal(ai.LowerLimit,-ones(7,1)) && isequal(ai.UpperLimit,ones(7,1)));
    % Exercise valid motion, invalid action, hard limit, collision, success,
    % timeout, proximity shaping, and success-vs-timeout precedence.
    cases=repmat({base},1,9);
    actions=zeros(7,9); actions(:,2)=[1;0.5;0;-0.5;0;0;0];
    actions(1,3)=1.01;
    cases{4}.q(1)=base.params.jointLimits(1,2); actions(1,4)=1;
    cases{5}.obstacle=collisionSphere(0.15);
    cases{5}.obstacle.Pose=trvec2tform(base.pEE.');
    cases{6}.target=base.pEE;
    cases{7}.stepCount=399;
    cases{8}.params.reward.jointLimitMarginRad=10; % force a nonzero shaping check
    cases{9}.target=base.pEE; cases{9}.stepCount=399;
    expectedCodes=[0 0 1 2 3 0 4 0 0];
    for i=1:numel(cases)
        state=cases{i}; a=actions(:,i);
        [o,r,d,s]=SAC_Environment("step",a,state);
        [op,rp,dp,sp]=environment.stepFunction(a*0.01,state);
        if sp.failureCode==0
            q=sp.q(1:7).'; lim=sp.params.jointLimits; w=sp.params.reward;
            nearest=min(q-lim(:,1),lim(:,2)-q);
            rp=rp-w.jointLimitWeight*sum(max(0,1-nearest/w.jointLimitMarginRad).^2);
        end
        assert(isequaln(o,op) && abs(r-rp)<1e-12 && isequal(d,dp));
        assert(isequaln(s.q,sp.q) && s.failureCode==expectedCodes(i));
    end
    rng(2026); [o1,s1]=SAC_Environment("reset",base,cfg);
    rng(2026); [o2,s2]=SAC_Environment("reset",base,cfg);
    assert(isequal(o1,o2) && isequal(s1.target,s2.target));
    offsets=s1.target-base.pEE;
    assert(offsets(1)>=cfg.goalX(1) && offsets(1)<=cfg.goalX(2));
    assert(offsets(2)>=cfg.goalY(1) && offsets(2)<=cfg.goalY(2) && offsets(3)==0);
    fixed=cfg; fixed.goalX=[0.04 0.04]; fixed.goalY=[0 0];
    [~,fs]=SAC_Environment("reset",base,fixed);
    assert(norm(fs.target-base.pEE-[0.04;0;0])<1e-12);
    official=fullfile(root,'Training_Results','SAC','C04','SAC_C04_Agent.mat');
    loaded=load(official,'agent'); agent=loaded.agent;
    before=saveobj(agent);
    assert(agent.ExperienceBuffer.Length==1204);
    assert(all(structfun(@logical,agent.AgentOptions.InfoToSave)));
    baseline=SAC_Experiment();
    assert(~baseline.trainingStarted && baseline.runDirectory=="");
    assert(all(baseline.baseline.local.Success));
    after=saveobj(agent);
    compareAgents(before,after);
    % Fresh creation is exercised in evaluate mode only; no learning occurs.
    fresh=SAC_Experiment(struct('startCheckpoint',"",'extraGoals',[0.035 -0.0125 0]));
    assert(~fresh.trainingStarted && isscalar(fresh.baseline.extra.Success));
    original=fullfile(root,'Training_Results','SAC_K_20261120_225730','best_reaching_agent.mat');
    historicalVerified=false;
    if isfile(original)
        old=load(original,'bestReachingAgent');
        compareAgents(saveobj(old.bestReachingAgent),before);
        oldPolicy=getGreedyPolicy(old.bestReachingAgent); policy=getGreedyPolicy(agent);
        goals=baseline.baseline.workspace.Goals;
        % Compare actions at every visited state, including failure rollouts.
        for i=1:size(goals,1)
            s=base; s.target=base.pEE+goals(i,:).';
            [obs,s]=environment.resetFunction(s);
            for k=1:400
                a=getAction(policy,{obs}); b=getAction(oldPolicy,{obs});
                assert(isequaln(a,b),'Original/standardized deterministic action mismatch.');
                [obs,~,done,s]=SAC_Environment("step",a{1},s);
                if done, break; end
            end
        end
        if isfile(fullfile(root,'Scripts','evaluatePandaCurriculum.m'))
            [rate,~,legacy]=evaluatePandaCurriculum(old.bestReachingAgent,[],[],base);
            assert(rate==1 && isequal(legacy.Success,baseline.baseline.local.Success));
            errorDifference=max(abs(legacy.FinalErrors-baseline.baseline.local.FinalErrors));
            fprintf('Historical/new evaluator maximum final-error difference: %.17g m\n',errorDifference);
            % Separate robot/network constructions may differ in last-place arithmetic.
            % This is a 1 nm comparison tolerance, not the task's 2 cm success tolerance.
            assert(errorDifference<1e-9,'Evaluator position error differs beyond numerical tolerance.');
            assert(isequal(legacy.Steps,baseline.baseline.local.Steps));
            assert(isequal(legacy.FailureCodes,baseline.baseline.local.FailureCodes));
        end
        historicalVerified=true;
    end
    report=struct('passed',true,'historicalCheckpointCompared',historicalVerified, ...
        'local',baseline.baseline.local,'reconstructedWorkspace',baseline.baseline.workspace);
    fprintf('PASS: SAC interface/reward/failures/reset, default/fresh evaluation, checkpoint state and policy regression.\n');
end

function compareAgents(a,b)
    assert(isequaln(getLearnableParameters(a.Actor),getLearnableParameters(b.Actor)));
    for i=1:2
        assert(isequaln(getLearnableParameters(a.Critic(i)),getLearnableParameters(b.Critic(i))));
        assert(isequaln(getLearnableParameters(a.TargetCritic(i)),getLearnableParameters(b.TargetCritic(i))));
        assert(isequaln(a.TargetCritic(i).State,b.TargetCritic(i).State));
    end
    assert(isequaln(a.PolicyState,b.PolicyState));
    assert(a.ExperienceBuffer.Length==b.ExperienceBuffer.Length);
    assert(isequaln(allExperiences(a.ExperienceBuffer),allExperiences(b.ExperienceBuffer)));
    % struct exposes private Adam moments/update counts read-only in R2025b.
    w=warning('off','MATLAB:structOnObject'); cleanup=onCleanup(@() warning(w));
    assert(isequaln(struct(a.ActorOptimizer),struct(b.ActorOptimizer)));
    assert(isequaln(struct(a.EntropyOptimizer),struct(b.EntropyOptimizer)));
    for i=1:2
        assert(isequaln(struct(a.CriticOptimizer(i)),struct(b.CriticOptimizer(i))));
    end
end
