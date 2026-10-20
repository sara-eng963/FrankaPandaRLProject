%% Franka Panda: online scenario diagnostic and regression checks
% Override controls before run, e.g. environmentDiagnosticOptions = struct( ...
%   'nEpisodes',3,'doVisualize',false,'startRangeMultiplier',1, ...
%   'targetDistanceRange',[0.15 0.50],'seed',314159, ...
%   'obstacleSamplingMode',"ee_tube",'armSamplingProbability',0.5);
% Optional: 'armSweepFractions',[0.4 0.5 0.6], 'reportCollisionLinks',true.
if ~exist('environmentDiagnosticOptions','var')
    environmentDiagnosticOptions = struct();
end
controls = environmentDiagnosticOptions;
nEpisodes = option(controls,'nEpisodes',30);
doVisualize = option(controls,'doVisualize',true);
reportCollisionLinks = option(controls,'reportCollisionLinks',false);
maxVisibleScenarios = option(controls,'maxVisibleScenarios',30);
diagnosticSeed = option(controls,'seed',314159);
halfWidthsDegrees = option(controls,'halfWidthsDegrees',[30 40 30 30 30 35 40]);
startRangeMultiplier = option(controls,'startRangeMultiplier',1);
targetDistanceRange = option(controls,'targetDistanceRange',[0.15 0.50]);
ikMaxInitialGuesses = option(controls,'ikMaxInitialGuesses',3);
ikMaxIterations = option(controls,'ikMaxIterations',100);
ikMaxTime = option(controls,'ikMaxTime',1);
ikPositionTolerance = option(controls,'ikPositionTolerance',0.001);
ikSolutionTolerance = option(controls,'ikSolutionTolerance',1e-6);
validateattributes(nEpisodes,{'double'},{'scalar','integer','positive'});
validateattributes(reportCollisionLinks,{'logical'},{'scalar'});
validateattributes(maxVisibleScenarios,{'double'},{'scalar','integer','positive'});
validateattributes(diagnosticSeed,{'double'}, ...
    {'scalar','integer','nonnegative','<=',2^32-nEpisodes});
validateattributes(halfWidthsDegrees,{'double'},{'size',[1 7],'finite','positive'});
validateattributes(startRangeMultiplier,{'double'},{'scalar','finite','positive'});
validateattributes(ikMaxInitialGuesses,{'double'},{'scalar','integer','positive'});
validateattributes(ikMaxIterations,{'double'},{'scalar','integer','positive'});
validateattributes(ikMaxTime,{'double'},{'scalar','finite','positive'});
validateattributes(targetDistanceRange,{'double'},{'size',[1 2],'nonnan'});
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);
% Never inherit a training-mode variable left in the caller's workspace.
environmentMode = "diagnostic";
run(fullfile(projectRoot,'Scripts','create_rl_environment.m'));
assert(cfg.evalSeed<diagnosticSeed || cfg.evalSeed>diagnosticSeed+nEpisodes-1, ...
    'Diagnostic seed range must exclude the reserved evaluation seed.');
obs = reset(env);
[nextObs,reward,done,info] = step(env,zeros(7,1));
assert(isequal(size(obs),[14 1]) && isequal(size(nextObs),[14 1]) && ...
    isfinite(reward) && ~done && info.stepCount==1);
assert(cfg.maxSteps==400 && cfg.maxIncrement==0.01);
assert(isfinite(targetDistanceRange(1)) && targetDistanceRange(1)>cfg.posTol && ...
    targetDistanceRange(2)>targetDistanceRange(1),'Invalid target-distance limits.');
disp('PASS: RL reset/step interface.');

%% Original joint-space starts; Cartesian-first goals with a reusable solver.
p = initialState.params;
p.workspaceBox = cfg.box;
p = environment.configureScenarioParameters(robot,qReady,p,controls);
p.recordScenarioDiagnostics = true;
scenarioState = initialState;
scenarioState.params = p;

%% Generate each scenario online. Retain outputs for inspection only.
rngBefore = rng;
episodes = cell(nEpisodes,1);
observations = cell(nEpisodes,1);
status = strings(nEpisodes,1);
resetSeconds = zeros(nEpisodes,1);
targetAttempts = zeros(nEpisodes,1);
targetSeconds = zeros(nEpisodes,1);
rrtMeasurements = zeros(nEpisodes,4);
rrtSeconds = zeros(nEpisodes,1);
obstacleAttempts = zeros(nEpisodes,1);
samplingMethods = strings(nEpisodes,1);
anchorCounts = zeros(nEpisodes,3);
noFeasibleAnchors = false(nEpisodes,1);
firstCollisionPairs = strings(nEpisodes,1);
ikCalls = zeros(nEpisodes,1); ikFailures = ikCalls; ikTimeLimits = ikCalls;
goals = nan(nEpisodes,3); starts = nan(nEpisodes,3);
counts = zeros(nEpisodes,7);
routeSteps = nan(nEpisodes,1);
radii = nan(nEpisodes,1);
distances = nan(nEpisodes,1);
nominalBlocked = false(nEpisodes,1);
selfBlocked = false(nEpisodes,1);
nominalSteps = nan(nEpisodes,1);
for k = 1:nEpisodes
    fprintf('Reset %d/%d ...\n',k,nEpisodes);
    [episodeObs,s,resetSeconds(k)] = runReset(scenarioState,diagnosticSeed+k-1);
    verifyReset(episodeObs,s,cfg);
    d = s.scenarioDiagnostics;
    episodes{k} = s; observations{k} = episodeObs;
    targetAttempts(k) = d.targetAttempts; targetSeconds(k) = d.targetSeconds;
    rrtMeasurements(k,:) = [d.rrtSetupSeconds d.rrtPlanSeconds ...
        d.rrtVerifySeconds d.rrtCollisionChecks];
    rrtSeconds(k) = d.rrtSeconds;
    obstacleAttempts(k) = d.attempts;
    samplingMethods(k) = d.obstacleSamplingMode;
    anchorCounts(k,:) = [d.armAnchorsGenerated d.armAnchorsRejected ...
        d.armAnchorsRetained];
    noFeasibleAnchors(k) = d.armNoFeasibleAnchors;
    ikCalls(k)=d.ikCalls; ikFailures(k)=d.ikFailures; ikTimeLimits(k)=d.ikTimeLimits;
    starts(k,:)=s.pEE.';
    counts(k,:) = [d.targetPointRejects d.startCollisionRejects ...
        d.goalCollisionRejects d.workspaceRejects double(d.success) ...
        d.rrtRejects d.rrtAttempts-d.rrtRejects];
    if d.success
        status(k) = "accepted";
        goals(k,:)=s.target.';
        radii(k) = s.obstacle.Radius;
        distances(k) = norm(s.target-s.pEE);
        routeSteps(k) = s.rrtRouteSteps;
        [nominalBlocked(k),selfBlocked(k),nominalSteps(k), ...
            firstCollisionPairs(k)] = inspectInterpolation(s,reportCollisionLinks);
    else
        status(k) = d.failureStage;
    end
    fprintf('  %s; %.2f s; target draws %d; RRT rejected/attempted %d/%d.\n', ...
        status(k),resetSeconds(k),d.targetAttempts,d.rrtRejects,d.rrtAttempts);
end
% Replay one accepted scenario (or the first failure if all calls failed).
% This tests the same real reset, including RRT, without maintaining a pool.
replay = find(status=="accepted",1);
if isempty(replay), replay=1; end
[replayObs,replayState] = runReset(scenarioState,diagnosticSeed+replay-1);
a = episodes{replay}; b = replayState;
assert(isequal(observations{replay},replayObs) && isequal(a.q,b.q) && ...
    isequal(a.target,b.target) && isequal(a.obstacle.Pose,b.obstacle.Pose) && ...
    a.obstacle.Radius==b.obstacle.Radius,'Seeded reset is not reproducible.');
timers = {'startSeconds','targetSeconds','rrtSeconds', ...
    'rrtSetupSeconds','rrtPlanSeconds','rrtVerifySeconds'};
assert(isequaln(rmfield(a.scenarioDiagnostics,timers), ...
    rmfield(b.scenarioDiagnostics,timers)));
if a.scenarioDiagnostics.success
    assert(a.rrtRouteSteps==b.rrtRouteSteps);
end
assert(isequal(rngBefore,rng),'Diagnostic changed the caller RNG.');

%% Concise scenario report, including exhaustion rather than hiding failures.
ok = status=="accepted"; nAccepted = nnz(ok);
summary = table(nEpisodes,nAccepted,nnz(status=="start"),nnz(status=="target"), ...
    nnz(status=="obstacle"),mean(resetSeconds(ok)), ...
    sum(resetSeconds)/nAccepted,mean(targetAttempts),1000*mean(targetSeconds), ...
    sum(counts(:,6)),mean(routeSteps(ok)), ...
    max([NaN;routeSteps(ok)],[],'omitnan'),100*mean(nominalBlocked(ok)), ...
    'VariableNames',{'Attempted','Accepted','StartFailed','TargetFailed', ...
    'ObstacleFailed','MeanSuccessfulReset_s','TotalSecondsPerSuccess', ...
    'MeanTargetAttempts','MeanTarget_ms','RRTRejected','MeanRouteSteps', ...
    'MaxRouteSteps','DirectBlockedPercent'});
% Observed endpoint cells, not proof of full-cell or workspace reachability.
gridSize=[10 10 10]; points=goals(ok,:);
indices=min(floor((points-cfg.box(:,1).')./ ...
    (diff(cfg.box,1,2).'./gridSize))+1,gridSize);
ids=sub2ind(gridSize,indices(:,1),indices(:,2),indices(:,3));
occupancyCounts=reshape(accumarray(ids,ones(nAccepted,1),[prod(gridSize) 1]),gridSize);
summary.IKCalls=sum(ikCalls); summary.IKFailures=sum(ikFailures);
summary.IKTimeLimits=sum(ikTimeLimits);
summary.OccupiedPercent=100*nnz(occupancyCounts)/prod(gridSize);
obstacleSummary = array2table(sum(counts,1),'VariableNames', ...
    {'TargetOverlap','StartCollision','GoalCollision','Workspace','Accepted', ...
    'RRTRejected','RRTAccepted'});
disp(summary); disp(obstacleSummary);
% Empty method means start/goal generation failed before method selection.
% Successful-reset RRT time includes all its candidate feasibility calls.
methods = unique(samplingMethods);
samplingSummary = table();
for methodIndex = 1:numel(methods)
    selected = samplingMethods==methods(methodIndex);
    successful = selected & ok;
    methodLabel = methods(methodIndex);
    if methodLabel=="", methodLabel="not_selected"; end
    row = table(methodLabel,nnz(successful),nnz(selected & ~ok), ...
        mean(obstacleAttempts(successful)), ...
        sum(counts(selected,1)),sum(counts(selected,2)), ...
        sum(counts(selected,3)),sum(counts(selected,4)), ...
        sum(counts(selected,6)),mean(resetSeconds(successful)), ...
        mean(rrtSeconds(successful)), ...
        'VariableNames',{'Method','Accepted','Failed','MeanCandidatesPerSuccess', ...
        'TargetOverlap','StartCollision','GoalCollision','Workspace', ...
        'RRTRejected','MeanSuccessfulReset_s','MeanRRTPerSuccess_s'});
    samplingSummary = [samplingSummary; row]; %#ok<AGROW>
end
fprintf('Configured obstacle sampling mode: %s\n',string(p.obstacleSamplingMode));
disp(samplingSummary);
armSelected = samplingMethods=="arm_sweep";
anchorSummary = array2table(mean(anchorCounts(armSelected,:),1), ...
    'VariableNames',{'MeanGenerated','MeanGeometricallyRejected','MeanRetained'});
anchorSummary.ArmScenarios = nnz(armSelected);
anchorSummary.NoFeasibleAnchors = nnz(noFeasibleAnchors);
disp('Arm-sweep anchors per selected scenario (including failures):');
disp(anchorSummary);
if reportCollisionLinks
    pairs = unique(firstCollisionPairs(firstCollisionPairs~=""));
    pairCounts = zeros(numel(pairs),1);
    for pairIndex = 1:numel(pairs)
        pairCounts(pairIndex) = nnz(firstCollisionPairs==pairs(pairIndex));
    end
    disp('Detected pair at first colliding interpolation sample (not continuous first contact):');
    disp(table(pairs,pairCounts,'VariableNames',{'DetectedPair','Scenarios'}));
end
% Include all RRT attempts within each accepted reset, including rejections.
rrtTimingSummary = array2table(mean(rrtMeasurements(ok,:),1), ...
    'VariableNames',{'MeanSetup_s','MeanPlanning_s', ...
    'MeanVerification_s','MeanCollisionChecks'});
disp('Mean RRT work per accepted scenario (all candidate attempts):');
disp(rrtTimingSummary);
fprintf('Start half-width multiplier %.2f; position-only IK tolerance %.4g m.\n', ...
    startRangeMultiplier,ikPositionTolerance);
if nAccepted>0
    fprintf('Accepted distance [min mean max] m: %.4f %.4f %.4f\n', ...
        min(distances(ok)),mean(distances(ok)),max(distances(ok)));
    fprintf('Accepted radius [min mean max] m: %.4f %.4f %.4f\n', ...
        min(radii(ok)),mean(radii(ok)),max(radii(ok)));
    fprintf('Accepted XYZ min [m]: %.4f %.4f %.4f; max: %.4f %.4f %.4f\n', ...
        min(points,[],1),max(points,[],1));
end
fprintf('Direct interpolation: blocked=%d, self-blocked=%d, over horizon=%d.\n', ...
    nnz(nominalBlocked(ok)),nnz(selfBlocked(ok)),nnz(nominalSteps(ok)>cfg.maxSteps));
disp('RRT validates discrete full-arm routes; continuous swept clearance is not established.');
if doVisualize
    showDashboard(episodes,find(ok),robot,cfg.box,nEpisodes,maxVisibleScenarios);
end
assert(isequal(rngBefore,rng));
disp('PASS: FK, workspace, radii, collision safety, route constraints, accounting and reproducibility.');

function value = option(s,name,default)
    if isfield(s,name), value=s.(name); else, value=default; end
end

function [obs,s,elapsed] = runReset(state,seed)
    callerStream = RandStream.getGlobalStream;
    guard = onCleanup(@() RandStream.setGlobalStream(callerStream));
    RandStream.setGlobalStream(RandStream('mt19937ar','Seed',seed));
    timer = tic;
    [obs,s] = environment.resetFunction(state);
    elapsed = toc(timer);
end

function verifyReset(obs,s,cfg)
    p=s.params; d=s.scenarioDiagnostics;
    assert(d.armAnchorsGenerated==d.armAnchorsRejected+d.armAnchorsRetained);
    if d.armNoFeasibleAnchors
        assert(~d.success && d.failureStage=="obstacle" && d.attempts==0 && ...
            d.obstacleSamplingMode=="arm_sweep" && ...
            d.armAnchorsGenerated>0 && d.armAnchorsRetained==0);
    end
    if d.attempts>0 || d.armNoFeasibleAnchors
        assert(any(d.obstacleSamplingMode==["ee_tube","arm_sweep"]));
        if string(p.obstacleSamplingMode)~="mixed"
            assert(d.obstacleSamplingMode==string(p.obstacleSamplingMode));
        end
    else
        assert(d.obstacleSamplingMode=="");
    end
    if d.obstacleSamplingMode=="arm_sweep"
        assert(d.armAnchorsGenerated==7*numel(p.armSweepFractions));
        assert(d.armNoFeasibleAnchors==(d.armAnchorsRetained==0));
    else
        assert(d.armAnchorsGenerated==0 && ~d.armNoFeasibleAnchors);
    end
    assert(d.attempts==d.targetPointRejects+d.startCollisionRejects+ ...
        d.goalCollisionRejects+d.workspaceRejects+d.rrtRejects+double(d.success));
    startPassed = d.failureStage~="start";
    targetPassed = startPassed && d.failureStage~="target";
    assert(d.startAttempts==d.startSampleCollisionRejects+ ...
        d.startSampleWorkspaceRejects+double(startPassed));
    assert(d.targetAttempts==d.targetIKRejects+d.targetDistanceRejects+double(targetPassed));
    assert(d.ikCalls==d.ikFailures+double(targetPassed));
    assert(d.ikCalls<=p.maxTargetAttempts*p.ikMaxInitialGuesses);
    assert(d.rrtAttempts==d.rrtRejects+double(d.success));
    assert(d.startAttempts<=p.maxResetAttempts && d.targetAttempts<=p.maxTargetAttempts);
    assert(d.attempts<=p.maxObstacleAttempts);
    assert(numel(d.candidateCodes)==d.attempts);
    for code=[1 2 3 4 5 6]
        expected = [d.targetPointRejects d.startCollisionRejects d.goalCollisionRejects ...
            double(d.success) d.workspaceRejects d.rrtRejects];
        assert(nnz(d.candidateCodes==code)==expected(code));
    end
    if ~d.success
        assert(isempty(obs) && ismember(d.failureStage,["start","target","obstacle"]));
        return;
    end
    assert(isequal(size(obs),[14 1]) && all(isfinite(obs)));
    assert(all(s.pEE>=cfg.box(:,1) & s.pEE<=cfg.box(:,2)));
    assert(all(s.target>=cfg.box(:,1) & s.target<=cfg.box(:,2)));
    assert(norm(obs(1:7)-s.q(1:7).')<1e-12);
    assert(norm(obs(8:10)-(s.target-s.pEE))<1e-12);
    center=s.obstacle.Pose(1:3,4); radius=s.obstacle.Radius;
    assert(norm(obs(11:13)-center)<1e-12 && abs(obs(14)-radius)<1e-12);
    assert(radius>=p.obstacleRadiusRange(1) && ...
        radius<=p.obstacleRadiusRange(2) && radius==d.obstacleRadius);
    assert(all(center-radius>=cfg.box(:,1)-1e-10) && ...
        all(center+radius<=cfg.box(:,2)+1e-10));
    distance=norm(s.target-s.pEE);
    assert(distance>=p.minTargetDistance);
    if isfield(p,'maxTargetDistance'), assert(distance<=p.maxTargetDistance); end
    qStart=s.q; qGoal=s.sampledGoalQ;
    for i=1:2
        if i==1, q=qStart; ranges=p.startJointRanges; xyz=s.pEE;
        else, q=qGoal; ranges=p.jointLimits; xyz=s.target; end
        assert(all(q(1:7)>=ranges(:,1).' & q(1:7)<=ranges(:,2).'));
        assert(all(q(1:7)>=p.jointLimits(:,1).' & q(1:7)<=p.jointLimits(:,2).'));
        assert(isequal(q(8:9),[0.04 0.04]));
        T=kinematics.forwardKinematics(s.robot,q,s.eeFrame);
        if i==1
            assert(norm(T(1:3,4)-xyz)<1e-10);
        else
            assert(norm(T(1:3,4)-xyz)<=p.ikPositionTolerance);
            assert(all(T(1:3,4)>=cfg.box(:,1) & T(1:3,4)<=cfg.box(:,2)));
            actualDistance=norm(T(1:3,4)-s.pEE);
            assert(actualDistance>=p.minTargetDistance);
            if isfield(p,'maxTargetDistance'), assert(actualDistance<=p.maxTargetDistance); end
        end
        assert(~collision.checkRobotCollision(s.robot,q,s.obstacle,p.skipPairs));
    end
    assert(isfinite(s.rrtRouteSteps) && s.rrtRouteSteps>0 && ...
        s.rrtRouteSteps<=cfg.maxSteps && s.rrtRouteSteps==floor(s.rrtRouteSteps));
end

function [blocked,selfBlocked,nSteps,firstCollisionPair] = ...
        inspectInterpolation(s,reportCollisionLinks)
    q=s.q; dq=s.sampledGoalQ(1:7)-q(1:7);
    nSteps=max(1,ceil(max(abs(dq))/s.params.maxIncrement));
    assert(max(abs(dq/nSteps))<=s.params.maxIncrement+1e-12);
    blocked=false; selfBlocked=false;
    firstCollisionPair = "";
    for j=1:nSteps
        qSample=q; qSample(1:7)=q(1:7)+(j/nSteps)*dq;
        if reportCollisionLinks
            % Same query and early-stop policy as checkRobotCollision; only
            % request the supported distance output for offline attribution.
            [flags,separation] = checkCollision(s.robot,qSample,{s.obstacle}, ...
                'SkippedSelfCollisions',s.params.skipPairs,'Exhaustive','off');
            colliding = any(flags);
        else
            colliding = collision.checkRobotCollision( ...
                s.robot,qSample,s.obstacle,s.params.skipPairs);
        end
        if colliding
            blocked=true;
            if reportCollisionLinks
                % NaN marks a detected collision; unchecked pairs are Inf.
                % An early self-collision may hide an obstacle collision.
                [bodyIndex,otherIndex] = find(isnan(separation),1);
                assert(~isempty(bodyIndex),'Collision API returned no detected pair.');
                bodyNames = [string(s.robot.BodyNames(:)); string(s.robot.BaseName)];
                if otherIndex<=s.robot.NumBodies+1
                    firstCollisionPair = bodyNames(bodyIndex)+ ...
                        " <-> "+bodyNames(otherIndex)+" (self)";
                else
                    firstCollisionPair = bodyNames(bodyIndex)+" <-> obstacle";
                end
            end
            break;
        end
    end
    % Preserve the prior obstacle-free reference on blocked interpolations.
    if blocked
        for j=1:nSteps
            qSample=q; qSample(1:7)=q(1:7)+(j/nSteps)*dq;
            if checkCollision(s.robot,qSample,'SkippedSelfCollisions',s.params.skipPairs)
                selfBlocked=true; break;
            end
        end
    end
end

%% Local visualization routines (not called by the RL environment)
function drawWorkspaceBox(ax,limits)
    x=limits(1,:); y=limits(2,:); z=limits(3,:);
    v=[x(1) y(1) z(1);x(2) y(1) z(1);x(2) y(2) z(1); ...
       x(1) y(2) z(1);x(1) y(1) z(2);x(2) y(1) z(2); ...
       x(2) y(2) z(2);x(1) y(2) z(2)];
    f=[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
    patch(ax,"Vertices",v,"Faces",f, ...
        "FaceColor",[0.08 0.67 0.88],"FaceAlpha",0.07, ...
        "EdgeColor",[0.16 0.77 0.98],"LineWidth",1.3,"HandleVisibility","off");
end

function finishAxes(ax)
    axis(ax,"equal");
    view(ax,130,23);
    grid(ax,"on"); box(ax,"on");
    xlabel(ax,"X (m)");ylabel(ax,"Y (m)");zlabel(ax,"Z (m)");
    ax.Color=[0.10 0.12 0.16]; ax.XColor=[0.88 0.90 0.94];
    ax.YColor=ax.XColor;ax.ZColor=ax.XColor;
    % Entire robot + task region; avoid clipping the manipulator.
    xlim(ax,[-0.35 0.82]); ylim(ax,[-0.55 0.55]); zlim(ax,[0 0.95]);
end

function renderSelectedScenario(data,selection)
    ax=data.axes;
    cla(ax);
    ep=data.episodes{data.indices(selection)};
    d=ep.scenarioDiagnostics;
    show(data.robot,ep.q,"Parent",ax,"Visuals","on", ...
        "Collisions","off","Frames","off","PreservePlot",false);
    hold(ax,"on");
    drawWorkspaceBox(ax,data.workspace);
    a=ep.pEE;b=ep.target;
    plot3(ax,[a(1) b(1)],[a(2) b(2)],[a(3) b(3)], ...
        "-","Color",[0.50 0.75 1],"LineWidth",2);
    scatter3(ax,a(1),a(2),a(3),90,[0.24 0.82 0.58],"filled");
    scatter3(ax,b(1),b(2),b(3),90,[0.99 0.36 0.43],"filled");

    % Only a capped subset of rejected candidates is drawn to avoid clutter.
    r=ep.obstacle.Radius;
    n=numel(d.candidateCodes);
    if d.success
        c=ep.obstacle.Pose(1:3,4);
        surf(ax,r*data.sphereX+c(1),r*data.sphereY+c(2), ...
            r*data.sphereZ+c(3),"EdgeColor","none", ...
            "FaceColor",[1 0.67 0.22],"FaceAlpha",0.48);
    end
    rejected=find(d.candidateCodes~=4);
    if ~isempty(rejected)
        subset=rejected(round(linspace(1,numel(rejected), ...
            min(32,numel(rejected)))));
        centers=d.candidateCenters(subset,:);
        cc=d.candidateCodes(subset);
        palette=[0.70 0.70 0.70; 0.88 0.31 0.36; ...
                 0.99 0.64 0.22; 0 0 0; 0.55 0.63 0.78];
        for code=[1 2 3 5]
            m=cc==code;
            if any(m)
                scatter3(ax,centers(m,1),centers(m,2),centers(m,3), ...
                    26,palette(code,:),"filled","MarkerFaceAlpha",0.6);
            end
        end
    end
    finishAxes(ax);
    if d.success
        episodeLabel = "accepted";
    else
        episodeLabel = "failed: " + d.failureStage;
    end
    title(ax,sprintf("Episode %d | %s | %.1f cm | %d candidates", ...
        data.indices(selection),char(episodeLabel), ...
        100*norm(b-a),n),"Color","w");
end

function showDashboard(episodes,available,robot,workspace,nAttempted,maxVisible)
    % One figure: all accepted start-goal pairs and a selected-episode inspector.
    % Use the already collected diagnostics; never generate new scenarios here.
    if isempty(available)
        warning('No accepted scenarios to visualize.');
        return;
    end

    available=available(:);
    n=numel(available);
    starts=zeros(n,3);
    goals=zeros(n,3);
    for i=1:n
        ep=episodes{available(i)};
        starts(i,:)=ep.pEE(:).';
        goals(i,:)=ep.target(:).';
    end

    fig=figure('Name','Panda accepted scenarios dashboard', ...
        'NumberTitle','off','Color',[0.10 0.12 0.16], ...
        'Position',[60 80 1500 780]);

    %% Left: accepted start-goal pairs (ALL accepted episodes)
    axOverview=axes('Parent',fig,'Position',[0.05 0.12 0.43 0.72]);
    hold(axOverview,'on');
    drawWorkspaceBox(axOverview,workspace);
    for i=1:n
        plot3(axOverview,[starts(i,1) goals(i,1)], ...
            [starts(i,2) goals(i,2)], ...
            [starts(i,3) goals(i,3)],'-', ...
            'Color',[0.53 0.58 0.65],'LineWidth',0.8, ...
            'HandleVisibility','off');
    end
    selectedLine=plot3(axOverview,[starts(1,1) goals(1,1)], ...
        [starts(1,2) goals(1,2)],[starts(1,3) goals(1,3)],'-', ...
        'Color',[1.0 0.79 0.31],'LineWidth',2.6, ...
        'HandleVisibility','off');
    hStart=scatter3(axOverview,starts(:,1),starts(:,2),starts(:,3), ...
        48,[0.24 0.82 0.58],'filled');
    hGoal=scatter3(axOverview,goals(:,1),goals(:,2),goals(:,3), ...
        48,[0.99 0.36 0.43],'filled');
    finishAxes(axOverview);
    margin=0.035;
    xlim(axOverview,workspace(1,:)+[-margin margin]);
    ylim(axOverview,workspace(2,:)+[-margin margin]);
    zlim(axOverview,workspace(3,:)+[-margin margin]);
    title(axOverview,sprintf('Accepted scenarios: %d/%d',n,nAttempted), ...
        'Color','w','FontWeight','bold');
    legend(axOverview,[hStart hGoal],{'Start','Goal'}, ...
        'Location','northwest','TextColor','w', ...
        'Color',[0.10 0.12 0.16]);

    %% Right: reuse the existing, tested scenario renderer
    axInspect=axes('Parent',fig,'Position',[0.54 0.11 0.43 0.72]);
    [sx,sy,sz]=feval('sphere',16);
    inspectIndices=available(1:min(maxVisible,n));
    data=struct('axes',axInspect,'figure',fig,'episodes',{episodes}, ...
        'indices',inspectIndices,'robot',robot,'workspace',workspace, ...
        'sphereX',sx,'sphereY',sy,'sphereZ',sz);
    selection=uicontrol(fig,'Style','popupmenu','Units','normalized', ...
        'Position',[0.78 0.94 0.18 0.043], ...
        'String',cellstr("Episode "+string(inspectIndices)), ...
        'BackgroundColor',[0.12 0.14 0.18],'ForegroundColor','w');
    selection.Callback=@(src,~) selectDashboardEpisode( ...
        data,src.Value,available,starts,goals,selectedLine);
    selectDashboardEpisode(data,1,available,starts,goals,selectedLine);
end

function selectDashboardEpisode(data,selection,available,starts,goals,selectedLine)
    % Update the inspector and highlight the corresponding pair at left.
    renderSelectedScenario(data,selection);
    idx=find(available==data.indices(selection),1);
    set(selectedLine, ...
        'XData',[starts(idx,1) goals(idx,1)], ...
        'YData',[starts(idx,2) goals(idx,2)], ...
        'ZData',[starts(idx,3) goals(idx,3)]);
end
