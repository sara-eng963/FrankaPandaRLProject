
%% Panda RL Environment — Diagnostic Configuration
% Creates a standard MATLAB RL environment using the
% existing step/reset functions and official course limits.
%
% The scenario below is for integration testing only.
% Training scenarios will be configured separately.

%% 1. Robot and Project Setup

projectRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(projectRoot);

run(fullfile(projectRoot, "Scripts", "robot_setup.m"));

%% 2. Environment Parameters

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

%% 3. Diagnostic Scenario

% Distant obstacle isolates environment-interface behavior.
obstacle = collisionSphere(0.05);
obstacle.Pose = trvec2tform([3, 3, 3]);

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

env = rlFunctionEnv( ...
    obsInfo, actInfo, ...
    @environment.stepFunction, ...
    @() environment.resetFunction(initialState));

disp("PASS: Panda RL environment created.");
