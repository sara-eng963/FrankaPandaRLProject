function [InitialObs, LoggedSignals, retryDiagnostics] = resetEpisodeWithRetries(initialState)
%RESETEPISODEWITHRETRIES Bounded complete-scenario retries, explicitly opt-in.
% Requires randomized start, IK target, obstacle and radius configuration.
% p.maxScenarioAttempts defaults to 5 (an experimental limit, not an optimum).
% Mixed sampling chooses once per requested episode, before any start draw.
% Each call to the raw generator receives the same original state template;
% the current random stream advances normally and is never reseeded here.
%
% Lightweight counters/history are returned in LoggedSignals.resetRetryDiagnostics
% and the optional third output. Timings are measured when the third output or
% existing detailed scenario diagnostics are requested. Detailed attempt data
% are retained only when p.recordScenarioDiagnostics is explicitly true.
% Exhaustion always throws environment:resetEpisodeWithRetries:Exhausted.
% Its causes identify each failed attempt by selected method and failure stage.
    p = initialState.params;
    maximum = 5;
    if isfield(p,'maxScenarioAttempts'), maximum=p.maxScenarioAttempts; end
    validateattributes(maximum,{'double'},{'scalar','real','finite','integer','positive'});
    assert(all(isfield(p,{'startJointRanges','ikSolver', ...
        'obstacleFractionRange','obstacleRadiusRange'})), ...
        'environment:resetEpisodeWithRetries:Configuration', ...
        'Complete-scenario retries require randomized start, IK target, obstacle and radius parameters.');
    detailed = isfield(p,'recordScenarioDiagnostics') && ...
        isequal(p.recordScenarioDiagnostics,true);
    recordTiming = detailed || nargout>=3;
    if recordTiming, resetTimer=tic; end
    mode = "ee_tube";
    if isfield(p,'obstacleSamplingMode'), mode=string(p.obstacleSamplingMode); end
    assert(isscalar(mode) && any(mode==["ee_tube","arm_sweep","mixed"]), ...
        'environment:resetEpisodeWithRetries:Configuration','Invalid obstacle sampling mode.');
    probability = 0.5;
    if isfield(p,'armSamplingProbability'), probability=p.armSamplingProbability; end
    validateattributes(probability,{'numeric'},{'scalar','real','finite','>=',0,'<=',1});
    selectedMode = mode;
    if mode=="mixed"
        selectedMode = "ee_tube";
        if rand<probability, selectedMode="arm_sweep"; end
    end
    template = initialState;
    template.params.obstacleSamplingMode = selectedMode;
    retryDiagnostics = struct('maxScenarioAttempts',maximum, ...
        'completeAttempts',0,'rejectedScenarios',0, ...
        'startFailures',0,'targetFailures',0,'obstacleFailures',0, ...
        'obstacleSamplingMode',selectedMode,'success',false,'totalSeconds',NaN, ...
        'failureStages',strings(maximum,1),'attemptRadii',nan(maximum,1), ...
        'attemptTargetDistances',nan(maximum,1));
    if detailed, retryDiagnostics.attemptDiagnostics=cell(maximum,1); end
    for attempt = 1:maximum
        % Never pass the previous attempt's partially generated state back in.
        [InitialObs, LoggedSignals, status] = environment.resetFunction(template);
        assert(status.obstacleSamplingMode=="" || status.obstacleSamplingMode==selectedMode, ...
            'environment:resetEpisodeWithRetries:InvalidStatus', ...
            'Raw reset changed the selected obstacle sampling method.');
        retryDiagnostics.completeAttempts = attempt;
        retryDiagnostics.failureStages(attempt) = status.failureStage;
        retryDiagnostics.attemptRadii(attempt) = status.obstacleRadius;
        retryDiagnostics.attemptTargetDistances(attempt) = status.targetDistance;
        if detailed
            retryDiagnostics.attemptDiagnostics{attempt}=LoggedSignals.scenarioDiagnostics;
        end
        if status.success
            assert(~isempty(InitialObs) && status.failureStage=="", ...
                'environment:resetEpisodeWithRetries:InvalidStatus', ...
                'Raw reset reported inconsistent success status.');
            retryDiagnostics.success = true;
            retryDiagnostics.failureStages = retryDiagnostics.failureStages(1:attempt);
            retryDiagnostics.attemptRadii = retryDiagnostics.attemptRadii(1:attempt);
            retryDiagnostics.attemptTargetDistances = ...
                retryDiagnostics.attemptTargetDistances(1:attempt);
            if detailed
                retryDiagnostics.attemptDiagnostics=retryDiagnostics.attemptDiagnostics(1:attempt);
            end
            % Keep the requested configuration (including mixed) in the state.
            % Actual episode selection is explicit in both diagnostic records.
            LoggedSignals.params = p;
            if recordTiming, retryDiagnostics.totalSeconds=toc(resetTimer); end
            LoggedSignals.resetRetryDiagnostics = retryDiagnostics;
            return;
        end
        assert(isempty(InitialObs) && any(status.failureStage==["start","target","obstacle"]), ...
            'environment:resetEpisodeWithRetries:InvalidStatus', ...
            'Raw reset returned an unrecognized failure status.');
        retryDiagnostics.rejectedScenarios = retryDiagnostics.rejectedScenarios+1;
        switch status.failureStage
            case "start"
                retryDiagnostics.startFailures = retryDiagnostics.startFailures+1;
            case "target"
                retryDiagnostics.targetFailures = retryDiagnostics.targetFailures+1;
            case "obstacle"
                retryDiagnostics.obstacleFailures = retryDiagnostics.obstacleFailures+1;
        end
    end
    % No catch: configuration errors, solver exceptions and bugs propagate.
    durationText = '';
    if recordTiming
        retryDiagnostics.totalSeconds = toc(resetTimer);
        durationText = sprintf(' Total reset duration: %.3f s.',retryDiagnostics.totalSeconds);
    end
    exhausted = MException('environment:resetEpisodeWithRetries:Exhausted', ...
        ['Reset exhausted %d complete scenarios using %s ' ...
         '(start=%d, target=%d, obstacle=%d). No valid episode was returned.%s'], ...
        maximum,char(selectedMode),retryDiagnostics.startFailures, ...
        retryDiagnostics.targetFailures,retryDiagnostics.obstacleFailures,durationText);
    for attempt = 1:maximum
        stage = retryDiagnostics.failureStages(attempt);
        % Stable cause identifiers let diagnostics count failures without
        % parsing human-readable messages or suppressing unexpected errors.
        cause = MException(sprintf('environment:resetEpisodeWithRetries:%s:%s', ...
            char(selectedMode),char(stage)), ...
            'Scenario %d: bounded %s failure; radius %.17g m; start-target distance %.17g m.', ...
            attempt,char(stage),retryDiagnostics.attemptRadii(attempt), ...
            retryDiagnostics.attemptTargetDistances(attempt));
        exhausted = addCause(exhausted,cause);
    end
    throw(exhausted);
end
