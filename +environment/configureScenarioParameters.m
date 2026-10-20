function p = configureScenarioParameters(robot,qReady,p,controls)
%CONFIGURESCENARIOPARAMETERS Shared validated randomized-scenario settings.
% Used by training initialization and the raw single-scenario diagnostic.
% p supplies course limits, workspace, collision exclusions and reward weights.
% Optional controls preserve testEnvironment's existing experiment overrides.
% Construct the reusable IK solver once here, never inside an episode reset.
    if nargin<4, controls=struct(); end
    halfWidthsDegrees = option(controls,'halfWidthsDegrees',[30 40 30 30 30 35 40]);
    multiplier = option(controls,'startRangeMultiplier',1);
    distanceRange = option(controls,'targetDistanceRange',[0.15 0.50]);
    maxIterations = option(controls,'ikMaxIterations',100);
    maxTime = option(controls,'ikMaxTime',1);
    solutionTolerance = option(controls,'ikSolutionTolerance',1e-6);
    validateattributes(halfWidthsDegrees,{'double'},{'size',[1 7],'finite','positive'});
    validateattributes(multiplier,{'double'},{'scalar','finite','positive'});
    validateattributes(distanceRange,{'double'},{'size',[1 2],'nonnan'});
    validateattributes(maxIterations,{'double'},{'scalar','integer','positive'});
    validateattributes(maxTime,{'double'},{'scalar','finite','positive'});
    assert(isfinite(distanceRange(1)) && distanceRange(1)>p.positionTolerance && ...
        distanceRange(2)>distanceRange(1),'Invalid target-distance limits.');
    widths = multiplier*deg2rad(halfWidthsDegrees).';
    p.startJointRanges = [max(qReady(:)-widths,p.jointLimits(:,1)), ...
                         min(qReady(:)+widths,p.jointLimits(:,2))];
    p.maxResetAttempts = 200;
    p.maxTargetAttempts = option(controls,'maxTargetAttempts',200);
    p.ikSolver = createIKSolver(robot,maxIterations,maxTime,solutionTolerance);
    p.ikMaxInitialGuesses = option(controls,'ikMaxInitialGuesses',3);
    p.ikPositionTolerance = option(controls,'ikPositionTolerance',0.001);
    p.minTargetDistance = distanceRange(1);
    if isfinite(distanceRange(2))
        p.maxTargetDistance = distanceRange(2);
    elseif isfield(p,'maxTargetDistance')
        p = rmfield(p,'maxTargetDistance');
    end
    p.obstacleRadiusRange = [0.03 0.06];
    p.obstacleFractionRange = [0.4 0.6];
    p.obstacleOffsetRange = [0.02 0.08];
    p.obstacleSamplingMode = option(controls,'obstacleSamplingMode',"ee_tube");
    p.armSamplingProbability = option(controls,'armSamplingProbability',0.5);
    p.armSweepFractions = option(controls,'armSweepFractions',[0.4 0.5 0.6]);
    p.maxObstacleAttempts = 100;
    p.rrt = struct('enabled',true,'maxIterations',750, ...
        'maxConnectionDistance',0.3,'validationDistance',0.01);
    p.recordScenarioDiagnostics = false;
    p.recordObstacleSamplingGeometry = option(controls,'recordObstacleSamplingGeometry',false);
end

function solver = createIKSolver(robot,maxIterations,maxTime,solutionTolerance)
    % Preserve the existing diagnostic's isolated solver-construction stream.
    % Episode generation uses the caller's active stream without reseeding.
    caller = RandStream.getGlobalStream;
    guard = onCleanup(@() RandStream.setGlobalStream(caller));
    RandStream.setGlobalStream(RandStream('mt19937ar','Seed',0));
    solver = inverseKinematics('RigidBodyTree',robot,'SolverAlgorithm','LevenbergMarquardt');
    solver.SolverParameters = struct('MaxIterations',maxIterations, ...
        'MaxTime',maxTime,'SolutionTolerance',solutionTolerance, ...
        'EnforceJointLimits',true,'AllowRandomRestart',false);
end

function value = option(s,name,default)
    if isfield(s,name), value=s.(name); else, value=default; end
end
