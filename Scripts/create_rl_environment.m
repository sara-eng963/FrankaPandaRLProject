
%% Panda RL Environment — Fixed Diagnostic or Randomized Training
% Creates a standard MATLAB RL environment using the
% existing step/reset functions and official course limits.
%
% Default: environmentMode = "diagnostic" (the original fixed interface fixture).
% Set environmentMode = "training" before running for randomized resets.

%% 1. Robot and Project Setup

projectRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(projectRoot);

if ~exist('environmentMode','var'), environmentMode="diagnostic"; end
environmentMode = string(environmentMode);
assert(isscalar(environmentMode) && any(environmentMode==["diagnostic","training"]), ...
    'Invalid environmentMode: use diagnostic or training.');
robotSetupVisualize = environmentMode~="training";
run(fullfile(projectRoot, "Scripts", "robot_setup.m"));
clear robotSetupVisualize

%% 2. Environment Parameters

% Fresh structures prevent training fields leaking into a later diagnostic run.
envParams = struct();
envParams.jointLimits = jointLimits;
envParams.skipPairs = skipPairs;

% Official course constraints
envParams.maxIncrement = cfg.maxIncrement;
envParams.maxSteps = cfg.maxSteps;
envParams.positionTolerance = cfg.posTol;
envParams.workspaceBox = cfg.box;

% Initial reward weights — subject to tuning
envParams.reward.progress = 1;
envParams.reward.step = 0.01;
envParams.reward.success = 10;
envParams.reward.failure = 10;

if environmentMode=="training"
    envParams = environment.configureScenarioParameters(robot,qReady,envParams);
    envParams.obstacleSamplingMode = "mixed";
    envParams.armSamplingProbability = 0.5;
    envParams.maxScenarioAttempts = 10;
    envParams.recordScenarioDiagnostics = false;
    envParams.recordObstacleSamplingGeometry = false;
end

%% 3. Fixed Fixture / Original Template for Randomized Resets

% Distant obstacle isolates environment-interface behavior.
obstacle = collisionSphere(0.03);
obstacle.Pose = trvec2tform([3, 3, 3]);

initialState = struct();
initialState.robot = robot;
initialState.q = qReference;
initialState.eeFrame = eeFrame;
initialState.pEE = pReady;
initialState.target = pReady + [0.10; 0; 0];
initialState.obstacle = obstacle;
initialState.params = envParams;
initialState.stepCount = 0;
initialState.failureCode = 0;

%% 4. RL Interface Specifications

% Observation: joints, target error, obstacle center, radius
obsInfo = rlNumericSpec([14 1]);
obsInfo.Name = "PandaObservations";

% Action: seven bounded joint increments [rad]
actInfo = rlNumericSpec([7 1]);
actInfo.Name = "JointIncrements";
actInfo.LowerLimit = -cfg.maxIncrement * ones(7,1);
actInfo.UpperLimit =  cfg.maxIncrement * ones(7,1);

%% 5. Construct Environment

if environmentMode=="training"
    resetFcn = @() environment.resetEpisodeWithRetries(initialState);
else
    resetFcn = @() environment.resetFunction(initialState);
end
% rlFunctionEnv obtains the accepted state from reset's second output (Info).
% Its constructor validates the interface, including an initial reset/step.
env = rlFunctionEnv( ...
    obsInfo, actInfo, ...
    @environment.stepFunction, ...
    resetFcn);

disp("Panda RL environment created ("+environmentMode+").");
