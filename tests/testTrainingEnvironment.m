%% Small rlFunctionEnv integration check; run manually, not during training.
% trainingEnvironmentTestOptions = struct('nResets',5,'seed',314159);
% Constructor validation, explicit validation and replay add a few resets to
% the consecutive-reset sample below. This is not the reliability benchmark.
if ~exist('trainingEnvironmentTestOptions','var')
    trainingEnvironmentTestOptions = struct();
end
controls = trainingEnvironmentTestOptions;
nResets = option(controls,'nResets',5);
seed = option(controls,'seed',314159);
validateattributes(nResets,{'double'},{'scalar','integer','>=',2});
validateattributes(seed,{'double'},{'scalar','integer','nonnegative','<=',2^32-1});
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);
rngBefore = rng;
checkEnvironments(projectRoot,nResets,seed);
assert(isequal(rngBefore,rng),'Integration test changed the caller RNG.');
disp('PASS: training/diagnostic rlFunctionEnv integration, state handoff and seeded replay.');

function checkEnvironments(projectRoot,nResets,seed)
    caller = RandStream.getGlobalStream;
    streamGuard = onCleanup(@() RandStream.setGlobalStream(caller));
    RandStream.setGlobalStream(RandStream('mt19937ar','Seed',0));
    visibility = get(groot,'DefaultFigureVisible');
    figuresBefore = findall(groot,'Type','figure','Name','Franka Panda - Ready Configuration');
    figureGuard = onCleanup(@() restoreSetupFigures(visibility,figuresBefore));
    set(groot,'DefaultFigureVisible','off');
    % Resolve the reserved seed through the actual course setup before any
    % randomized training constructor/reset uses the requested test seed.
    run(fullfile(projectRoot,'Scripts','create_rl_environment.m'));
    defaultEnv = env;
    assert(environmentMode=="diagnostic" && seed~=cfg.evalSeed, ...
        'Expected the fixed default; do not use the reserved evaluation seed.');
    defaultFixed = reset(defaultEnv);
    RandStream.setGlobalStream(RandStream('mt19937ar','Seed',seed));
    environmentMode = "training";
    run(fullfile(projectRoot,'Scripts','create_rl_environment.m'));
    trainingEnv = env;
    assert(environmentMode=="training" && ~cfg.visualizeRobot);
    assert(isequal(trainingEnv.StepFcn,@environment.stepFunction));
    checkSpecifications(trainingEnv);
    % The environment constructor already validates. Also exercise the public
    % validation entry point explicitly; it may leave Info after a test step.
    validateEnvironment(trainingEnv);
    observations = zeros(14,nResets);
    targets = zeros(3,nResets);
    for k = 1:nResets
        obs = reset(trainingEnv);
        state = trainingEnv.Info; % Documented public state handoff in R2025b.
        verifyTrainingReset(obs,state);
        observations(:,k) = obs;
        targets(:,k) = state.target;
        [next,reward,done] = step(trainingEnv,zeros(7,1));
        stepped = trainingEnv.Info;
        verifyObservation(next,stepped);
        assert(isscalar(reward) && isfinite(reward));
        assert(isscalar(done) && islogical(done) && ~done);
        assert(stepped.stepCount==1 && stepped.failureCode==0);
        assert(isequal(next,obs) && isequal(stepped.q,state.q));
        assert(abs(reward+state.params.reward.step)<1e-12);
    end
    assert(any(any(diff(observations(1:7,:),1,2)~=0)), ...
        'Consecutive resets did not vary starting joints.');
    assert(any(any(diff(targets,1,2)~=0)), ...
        'Consecutive resets did not vary Cartesian goals.');
    % External replay only: restore both global-stream ownership and the RNG
    % snapshot. Assigning State on a saved handle alone does not ensure that
    % handle is still the global stream used by the next reset.
    stream = RandStream.getGlobalStream;
    replayState = rng;
    first = reset(trainingEnv); firstInfo = trainingEnv.Info;
    firstEndRNG = rng;
    firstChangedStream = ~isequal(stream,RandStream.getGlobalStream);
    RandStream.setGlobalStream(stream);
    rng(replayState);
    assert(isequal(replayState,rng), ...
        'Replay test did not restore the initial global RNG configuration.');
    replay = reset(trainingEnv); replayInfo = trainingEnv.Info;
    replayEndRNG = rng;
    replayChangedStream = ~isequal(stream,RandStream.getGlobalStream);
    replayFields = ["observation","start joints","target","IK goal", ...
        "obstacle pose","retry diagnostics"];
    replayMatches = [isequal(first,replay),isequal(firstInfo.q,replayInfo.q), ...
        isequal(firstInfo.target,replayInfo.target), ...
        isequal(firstInfo.sampledGoalQ,replayInfo.sampledGoalQ), ...
        isequal(firstInfo.obstacle.Pose,replayInfo.obstacle.Pose), ...
        isequaln(firstInfo.resetRetryDiagnostics,replayInfo.resetRetryDiagnostics)];
    assert(all(replayMatches), ...
        ['Externally seeded training reset replay changed: %s. ' ...
         'Global stream changed during first/replay reset: %d/%d. ' ...
         'Ending global RNG configurations match: %d.'], ...
        char(strjoin(replayFields(~replayMatches),', ')), ...
        firstChangedStream,replayChangedStream,isequal(firstEndRNG,replayEndRNG));

    % Switch in the SAME workspace: stale randomized fields must not survive.
    environmentMode = "diagnostic";
    run(fullfile(projectRoot,'Scripts','create_rl_environment.m'));
    diagnosticEnv = env;
    assert(isequal(diagnosticEnv.StepFcn,trainingEnv.StepFcn));
    checkSpecifications(diagnosticEnv);
    fixed = reset(diagnosticEnv); fixedInfo = diagnosticEnv.Info;
    assert(isequal(fixedInfo.q,qReference) && fixedInfo.stepCount==0 && fixedInfo.failureCode==0);
    assert(isequal(fixedInfo.obstacle.Pose(1:3,4),[3;3;3]) && fixedInfo.obstacle.Radius==0.03);
    assert(~isfield(fixedInfo.params,'ikSolver') && ...
        ~isfield(fixedInfo.params,'startJointRanges') && ~isfield(fixedInfo,'resetRetryDiagnostics'));
    assert(isequal(fixed,reset(diagnosticEnv)),'Diagnostic reset must stay deterministic.');
    [next,reward,done] = step(diagnosticEnv,zeros(7,1));
    verifyObservation(next,diagnosticEnv.Info);
    assert(isscalar(reward) && isfinite(reward) && islogical(done) && ~done && ...
        diagnosticEnv.Info.stepCount==1 && diagnosticEnv.Info.failureCode==0);
    assert(isequal(defaultFixed,fixed) && isequal(defaultFixed,reset(defaultEnv)), ...
        'Default and explicit diagnostic environments must stay identical.');
    % Reassigning script variables must not alter the old training callback.
    obs = reset(trainingEnv);
    verifyTrainingReset(obs,trainingEnv.Info);
    fprintf('Checked %d consecutive randomized resets, zero-action steps, mode switches and replay.\n',nResets);
end

function checkSpecifications(env)
    observation = getObservationInfo(env);
    action = getActionInfo(env);
    assert(isequal(observation.Dimension,[14 1]));
    assert(isequal(action.Dimension,[7 1]) && ...
        isequal(action.LowerLimit,-0.01*ones(7,1)) && ...
        isequal(action.UpperLimit,0.01*ones(7,1)));
end

function verifyTrainingReset(obs,s)
    verifyObservation(obs,s);
    p = s.params; r = s.resetRetryDiagnostics;
    assert(s.stepCount==0 && s.failureCode==0 && r.success);
    assert(p.obstacleSamplingMode=="mixed" && p.armSamplingProbability==0.5 && ...
        p.maxScenarioAttempts==10 && r.completeAttempts>=1 && r.completeAttempts<=10);
    assert(any(r.obstacleSamplingMode==["ee_tube","arm_sweep"]));
    assert(~p.recordScenarioDiagnostics && ~p.recordObstacleSamplingGeometry && ...
        ~isfield(s,'scenarioDiagnostics') && isnan(r.totalSeconds));
    assert(p.maxSteps==400 && p.maxIncrement==0.01);
    assert(isequal(s.q(8:9),[0.04 0.04]) && isequal(s.sampledGoalQ(8:9),s.q(8:9)));
    assert(isfinite(s.rrtRouteSteps) && s.rrtRouteSteps>0 && ...
        s.rrtRouteSteps==floor(s.rrtRouteSteps) && s.rrtRouteSteps<=400);
    assert(all(s.pEE>=p.workspaceBox(:,1) & s.pEE<=p.workspaceBox(:,2)) && ...
        all(s.target>=p.workspaceBox(:,1) & s.target<=p.workspaceBox(:,2)));
    distance = norm(s.target-s.pEE);
    assert(distance>=0.15 && distance<=0.50);
    center = s.obstacle.Pose(1:3,4); radius = s.obstacle.Radius;
    assert(radius>=0.03 && radius<=0.06 && ...
        all(center-radius>=p.workspaceBox(:,1)-1e-12) && ...
        all(center+radius<=p.workspaceBox(:,2)+1e-12));
end

function verifyObservation(obs,s)
    assert(isequal(size(obs),[14 1]) && all(isfinite(obs)));
    expected = [s.q(1:7).'; s.target-s.pEE; s.obstacle.Pose(1:3,4); s.obstacle.Radius];
    assert(isequal(obs,expected),'Observation contains an incorrect state component.');
end

function restoreSetupFigures(visibility,previous)
    set(groot,'DefaultFigureVisible',visibility);
    figures = findall(groot,'Type','figure','Name','Franka Panda - Ready Configuration');
    for h = figures(:).'
        if ~any(h==previous), delete(h); end
    end
end

function value = option(s,name,default)
    if isfield(s,name), value=s.(name); else, value=default; end
end
