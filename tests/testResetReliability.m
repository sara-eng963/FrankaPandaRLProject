%% Complete-scenario reset reliability; run manually, never during training.
% resetReliabilityOptions = struct('nResets',100,'maxScenarioAttempts',5, ...
%     'seed',314159,'obstacleSamplingMode',"mixed", ...
%     'armSamplingProbability',0.5,'detailedDiagnostics',false);
% Reference setup runs testEnvironment once (one raw scenario plus replay),
% outside the measurements below. Its figures are hidden and cleaned up.
% The stress loop calls the retry wrapper directly; no visualization is added.
if ~exist('resetReliabilityOptions','var'), resetReliabilityOptions=struct(); end
controls = resetReliabilityOptions;
nRequested = option(controls,'nResets',100);
maximum = option(controls,'maxScenarioAttempts',5);
baseSeed = option(controls,'seed',314159);
detailed = option(controls,'detailedDiagnostics',false);
validateattributes(nRequested,{'double'},{'scalar','integer','positive'});
validateattributes(maximum,{'double'},{'scalar','real','finite','integer','positive'});
validateattributes(baseSeed,{'double'}, ...
    {'scalar','integer','nonnegative','<=',2^32-nRequested});
validateattributes(detailed,{'logical'},{'scalar'});
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);
rngBefore = rng;
fprintf('Loading the authoritative single-scenario diagnostic configuration (not timed).\n');
[retryTemplate,evaluationSeed] = referenceTemplate(projectRoot,controls,baseSeed);
assert(evaluationSeed<baseSeed || evaluationSeed>baseSeed+nRequested-1, ...
    'Reliability seed range must exclude the reserved evaluation seed.');
retryTemplate.params.maxScenarioAttempts = maximum;
retryTemplate.params.recordScenarioDiagnostics = detailed;
retryTemplate.params.recordObstacleSamplingGeometry = false;
assert(retryTemplate.params.maxSteps==400 && retryTemplate.params.maxIncrement==0.01);

requestedSeeds = baseSeed+(0:nRequested-1).';
succeeded = false(nRequested,1);
completeAttempts = zeros(nRequested,1);
rejectedScenarios = zeros(nRequested,1);
failureCounts = zeros(nRequested,3);
resetSeconds = zeros(nRequested,1);
selectedMethods = strings(nRequested,1);
acceptedRadii = nan(nRequested,1);
acceptedDistances = nan(nRequested,1);
reliabilityRetryDiagnostics = cell(nRequested,1);
for k = 1:nRequested
    [obs,state,retry,resetSeconds(k)] = requestReset(retryTemplate,requestedSeeds(k));
    succeeded(k) = retry.success;
    completeAttempts(k) = retry.completeAttempts;
    rejectedScenarios(k) = retry.rejectedScenarios;
    selectedMethods(k) = retry.obstacleSamplingMode;
    failureCounts(k,:) = [retry.startFailures retry.targetFailures retry.obstacleFailures];
    reliabilityRetryDiagnostics{k} = retry;
    assert(retry.completeAttempts>=1 && retry.completeAttempts<=maximum);
    assert(sum(failureCounts(k,:))==retry.rejectedScenarios);
    assert(retry.completeAttempts==retry.rejectedScenarios+double(retry.success));
    if retry.success
        verifyEpisode(obs,state,retry);
        assert(string(state.params.obstacleSamplingMode)==string(retryTemplate.params.obstacleSamplingMode));
        acceptedRadii(k) = state.obstacle.Radius;
        acceptedDistances(k) = norm(state.target-state.pEE);
    else
        assert(isempty(obs) && retry.completeAttempts==maximum);
    end
    if k==1
        replayObs=obs; replayState=state; replayRetry=retry;
    end
    fprintf('Reset %d/%d: %s; complete attempts %d/%d; method %s; %.2f s.\n', ...
        k,nRequested,char(stringOutcome(retry.success)),retry.completeAttempts, ...
        maximum,char(retry.obstacleSamplingMode),resetSeconds(k));
end
% One requested episode replay, including every complete retry and RNG draw.
[obs,state,retry] = requestReset(retryTemplate,requestedSeeds(1));
assert(isequaln(withoutTimings(replayRetry),withoutTimings(retry)), ...
    'Seeded retry history is not reproducible.');
assert(isequal(replayObs,obs),'Seeded retry observation is not reproducible.');
if retry.success
    verifyEpisode(obs,state,retry);
    assert(isequal(replayState.q,state.q) && isequal(replayState.pEE,state.pEE) && ...
        isequal(replayState.target,state.target) && ...
        isequal(replayState.sampledGoalQ,state.sampledGoalQ) && ...
        isequal(replayState.obstacle.Pose,state.obstacle.Pose) && ...
        replayState.obstacle.Radius==state.obstacle.Radius && ...
        replayState.rrtRouteSteps==state.rrtRouteSteps, ...
        'Seeded accepted episode is not reproducible.');
end
assert(isequal(rngBefore,rng),'Reliability diagnostics changed the caller RNG.');

reliabilitySummary = table(nRequested,nnz(succeeded),nnz(~succeeded), ...
    mean(completeAttempts),max(completeAttempts),mean(resetSeconds),max(resetSeconds), ...
    sum(failureCounts(:,1)),sum(failureCounts(:,2)),sum(failureCounts(:,3)), ...
    'VariableNames',{'Requested','Successful','Exhausted','MeanCompleteAttempts', ...
    'MaxCompleteAttempts','MeanReset_s','MaxReset_s', ...
    'StartScenarioFailures','TargetScenarioFailures','ObstacleScenarioFailures'});
reliabilityResults = table(requestedSeeds,succeeded,selectedMethods, ...
    completeAttempts,rejectedScenarios,resetSeconds,acceptedRadii,acceptedDistances, ...
    failureCounts(:,1),failureCounts(:,2),failureCounts(:,3), ...
    'VariableNames',{'Seed','Success','Method','CompleteAttempts','RejectedScenarios', ...
    'Reset_s','AcceptedRadius_m','AcceptedDistance_m', ...
    'StartFailures','TargetFailures','ObstacleFailures'});
methods = ["ee_tube";"arm_sweep"];
methodData = zeros(2,8);
for m = 1:2
    chosen = selectedMethods==methods(m);
    good = chosen & succeeded;
    methodData(m,:) = [nnz(chosen),nnz(good),nnz(chosen & ~succeeded), ...
        mean(completeAttempts(chosen)),mean(resetSeconds(chosen)), ...
        min([NaN;acceptedRadii(good)],[],'omitnan'),mean(acceptedRadii(good)), ...
        max([NaN;acceptedRadii(good)],[],'omitnan')];
end
reliabilityMethodSummary = [table(methods,'VariableNames',{'Method'}), ...
    array2table(methodData,'VariableNames',{'Selected','Successful','Exhausted', ...
    'MeanCompleteAttempts','MeanReset_s','MinAcceptedRadius_m', ...
    'MeanAcceptedRadius_m','MaxAcceptedRadius_m'})];
disp(reliabilitySummary);
disp(reliabilityMethodSummary);
fprintf('%d exhausted requested resets returned no episode.\n',nnz(~succeeded));
disp('State, accounting and replay assertions completed; exhaustion remains a failed reset.');
% reliabilityResults and reliabilityRetryDiagnostics remain available for
% inspecting accepted radii/distances and per-attempt failures by method.

function [template,evaluationSeed] = referenceTemplate(projectRoot,controls,seed)
    % Reuse the actual diagnostic configuration, including all IK, RRT, radius,
    % start-range and workspace parameters; do not maintain a second copy.
    caller = RandStream.getGlobalStream;
    streamGuard = onCleanup(@() RandStream.setGlobalStream(caller));
    RandStream.setGlobalStream(RandStream('mt19937ar','Seed',0));
    previousVisibility = get(groot,'DefaultFigureVisible');
    previousFigures = findall(groot,'Type','figure','Name','Franka Panda - Ready Configuration');
    figureGuard = onCleanup(@() restoreSetupFigures(previousVisibility,previousFigures));
    set(groot,'DefaultFigureVisible','off');
    environmentDiagnosticOptions = struct('nEpisodes',1,'doVisualize',false, ...
        'seed',seed,'reportCollisionLinks',false,'recordObstacleSamplingGeometry',false, ...
        'obstacleSamplingMode',option(controls,'obstacleSamplingMode',"mixed"), ...
        'armSamplingProbability',option(controls,'armSamplingProbability',0.5));
    run(fullfile(projectRoot,'tests','testEnvironment.m'));
    template = scenarioState;
    evaluationSeed = cfg.evalSeed;
end

function restoreSetupFigures(visibility,previous)
    set(groot,'DefaultFigureVisible',visibility);
    figures = findall(groot,'Type','figure','Name','Franka Panda - Ready Configuration');
    for h = figures(:).'
        if ~any(h==previous), delete(h); end
    end
end

function [obs,state,retry,seconds] = requestReset(template,seed)
    caller = RandStream.getGlobalStream;
    guard = onCleanup(@() RandStream.setGlobalStream(caller));
    RandStream.setGlobalStream(RandStream('mt19937ar','Seed',seed));
    resetTimer = tic;
    try
        [obs,state,retry] = environment.resetEpisodeWithRetries(template);
    catch exception
        if ~strcmp(exception.identifier,'environment:resetEpisodeWithRetries:Exhausted')
            rethrow(exception); % Invalid configuration or programming error.
        end
        obs = []; state = struct();
        retry = exhaustionDiagnostics(exception,template.params.maxScenarioAttempts);
    end
    seconds = toc(resetTimer);
end

function retry = exhaustionDiagnostics(exception,maximum)
    % The wrapper always throws on exhaustion. Structured cause identifiers
    % preserve stage counts and selected mode without parsing error messages.
    assert(numel(exception.cause)==maximum,'Incomplete exhaustion history.');
    retry = struct('maxScenarioAttempts',maximum,'completeAttempts',maximum, ...
        'rejectedScenarios',maximum,'startFailures',0,'targetFailures',0, ...
        'obstacleFailures',0,'obstacleSamplingMode',"",'success',false, ...
        'totalSeconds',NaN,'failureStages',strings(maximum,1), ...
        'exhaustionCauses',strings(maximum,2));
    for k = 1:maximum
        cause = exception.cause{k};
        parts = split(string(cause.identifier),":");
        assert(numel(parts)==4 && parts(1)=="environment" && ...
            parts(2)=="resetEpisodeWithRetries" && ...
            any(parts(3)==["ee_tube","arm_sweep"]),'Unexpected exhaustion cause.');
        if k==1, retry.obstacleSamplingMode=parts(3); end
        assert(parts(3)==retry.obstacleSamplingMode,'Sampling method changed during retries.');
        retry.failureStages(k) = parts(4);
        retry.exhaustionCauses(k,:) = [string(cause.identifier) string(cause.message)];
        switch parts(4)
            case "start", retry.startFailures=retry.startFailures+1;
            case "target", retry.targetFailures=retry.targetFailures+1;
            case "obstacle", retry.obstacleFailures=retry.obstacleFailures+1;
            otherwise, error('Unexpected failure stage in exhaustion history.');
        end
    end
end

function verifyEpisode(obs,s,retry)
    p=s.params;
    assert(isequal(size(obs),[14 1]) && all(isfinite(obs)));
    assert(s.stepCount==0 && s.failureCode==0);
    assert(isequal(size(s.q),[1 9]) && all(isfinite(s.q)));
    assert(isequal(size(s.sampledGoalQ),[1 9]) && all(isfinite(s.sampledGoalQ)));
    assert(isequal(s.q(8:9),s.sampledGoalQ(8:9)));
    assert(isequal(obs(1:7),s.q(1:7).') && isequal(obs(8:10),s.target-s.pEE));
    assert(isequal(obs(11:13),s.obstacle.Pose(1:3,4)) && obs(14)==s.obstacle.Radius);
    assert(isfinite(s.rrtRouteSteps) && s.rrtRouteSteps>0 && ...
        s.rrtRouteSteps==floor(s.rrtRouteSteps) && s.rrtRouteSteps<=p.maxSteps);
    assert(all(s.pEE>=p.workspaceBox(:,1) & s.pEE<=p.workspaceBox(:,2)));
    assert(all(s.target>=p.workspaceBox(:,1) & s.target<=p.workspaceBox(:,2)));
    radius = s.obstacle.Radius; center = s.obstacle.Pose(1:3,4);
    assert(radius>=p.obstacleRadiusRange(1) && radius<=p.obstacleRadiusRange(2));
    assert(all(center-radius>=p.workspaceBox(:,1)-1e-12) && ...
        all(center+radius<=p.workspaceBox(:,2)+1e-12));
    distance = norm(s.target-s.pEE);
    assert(distance>=p.minTargetDistance);
    if isfield(p,'maxTargetDistance'), assert(distance<=p.maxTargetDistance); end
    assert(retry.success && isequaln(s.resetRetryDiagnostics,retry));
    if isfield(s,'scenarioDiagnostics')
        assert(s.scenarioDiagnostics.success && ...
            s.scenarioDiagnostics.obstacleSamplingMode==retry.obstacleSamplingMode);
    end
end

function value = withoutTimings(value)
    % Remove only elapsed seconds; preserve deterministic counters, geometry,
    % method choices, radius/distance histories and solver time-limit counts.
    value = rmfield(value,'totalSeconds');
    if isfield(value,'attemptDiagnostics')
        timers = {'startSeconds','targetSeconds','rrtSeconds', ...
            'rrtSetupSeconds','rrtPlanSeconds','rrtVerifySeconds'};
        for k = 1:numel(value.attemptDiagnostics)
            value.attemptDiagnostics{k}=rmfield(value.attemptDiagnostics{k},timers);
        end
    end
end

function text = stringOutcome(success)
    if success, text="accepted"; else, text="EXHAUSTED"; end
end

function value = option(s,name,default)
    if isfield(s,name), value=s.(name); else, value=default; end
end
