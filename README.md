# Franka Panda RL

MATLAB R2025b kinematic reaching and obstacle-environment research project.
Requires Reinforcement Learning Toolbox, Robotics System Toolbox and Deep Learning
Toolbox, plus the Franka Panda robot model data used by `loadrobot`.
Simulink is optional and is not used by the SAC reaching workflow.

From the repository root:

```matlab
addpath('Scripts');
result = SAC_Experiment();                 % evaluation only; writes no results
addpath('tests');
report = runSACVerification();             % non-training verification
```

Evaluation requires the external `SAC_C04_Agent.mat` artifact at the location
recorded in [checkpoint provenance](docs/SAC_Checkpoint_Provenance.json).
Artifacts are excluded from Git. A source checkout alone does not include the agent.
See [SAC workflow](docs/SAC-Workflow.md) for configuration, explicit training,
checkpoint contents and limitations. Existing environment design and historical
validation are described in [environment notes](docs/Environment-Design.md).
