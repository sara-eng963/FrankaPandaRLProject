
%% Franka Panda - Simulink Initialization
% Run this script before pressing Run in simulink/PandaRL.slx.
% It prepares one valid randomized episode and an UNTRAINED placeholder agent.
% No RL training takes place here.

%% 1. Load the existing robot and environment configuration
projectRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(projectRoot, fullfile(projectRoot, "simulink"));

environmentMode = "training";
run(fullfile(projectRoot, "Scripts", "create_rl_environment.m"));

%% 2. Open the existing Simulink model
model = "PandaRL";
load_system(fullfile(projectRoot, "simulink", model + ".slx"));

%% 3. Provide the shared robot template to the model workspace
% simulinkStep.m retrieves this data from the model workspace on each step.
modelWorkspace = get_param(model, "ModelWorkspace");
assignin(modelWorkspace, "pandaTemplate", initialState);

%% 4. Prepare a valid randomized episode for a manual Simulink run
% The graphical Run button does NOT invoke simEnv.ResetFcn automatically.
[episodeInitialObservation, episode] = ...
    environment.resetEpisodeWithRetries(initialState);

episodeInitialState = [
    episode.q(1:7).';
    episode.pEE(:);
    episode.target(:);
    episode.obstacle.Pose(1:3,4);
    episode.obstacle.Radius;
    episode.stepCount;
    episode.failureCode
];

assert(isequal(size(episodeInitialObservation), [14 1]));
assert(isequal(size(episodeInitialState), [19 1]));

% Keep these initial conditions in the base workspace.
assignin("base", "episodeInitialObservation", episodeInitialObservation);
assignin("base", "episodeInitialState", episodeInitialState);

%% 5. Create a temporary, untrained agent for interface testing
% Replace this with a trained agent when appropriate.
agentObj = rlSACAgent(obsInfo, actInfo);
agentObj.AgentOptions.SampleTime = cfg.dt;
assignin("base", "agentObj", agentObj);

agentBlock = model + "/RL Agent";
set_param(agentBlock, "Agent", "agentObj");

%% 6. Prepare optional programmatic RL simulations
simEnv = rlSimulinkEnv(model, agentBlock, obsInfo, actInfo);
simEnv.ResetFcn = @(in) resetPandaSimulink(in, initialState);

set_param(model, "StopTime", num2str(cfg.dt * cfg.maxSteps));
open_system(model);

disp("PandaRL initialized. Press Run in Simulink for the prepared episode.");
disp("The SAC agent is untrained; this is an interface test, not training.");
