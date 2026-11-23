# SAC experiment workflow

## Active architecture

`Scripts/SAC_Experiment.m` is a function entry point with a configuration block
near its beginning. It owns agent loading/creation, deterministic evaluation,
explicit training, checkpoint selection and provenance. It does not consume base
workspace variables. `Scripts/SAC_Environment.m` owns normalized actions, the
joint-limit reward term, ready-state construction and nearby-goal resets.
Its file-local callbacks are valid MATLAB function handles; keep the file and
`Scripts` on the path when restoring saved callbacks.

The core reset, step, FK and collision modules are unchanged. The normalized
adapter multiplies all seven actions by 0.01 rad, without clipping. The step
function retains its success, invalid-action, joint-limit, collision and timeout
rules. The proximity penalty applies on valid steps, including success, and is
skipped for all failure codes including timeout. This is reaching with a distant
sphere, not trained obstacle avoidance.

## Configure and evaluate

```matlab
addpath('Scripts');
r = SAC_Experiment();  % mode="evaluate", no files saved and no training
r = SAC_Experiment(struct('curriculum',5,'goalX',[0.03 0.04], ...
    'goalY',[-0.0125 0.005],'goalZ',0, ...
    'extraGoals',[0.03 -0.0125 0; 0.04 -0.0125 0]));
```

All distances are metres. Equal range endpoints provide a fixed target.
Unknown configuration fields are rejected. The workflow seeds its own work and
restores the caller RNG afterward. Environment constructor validation consumes
randomness before training; initialization and pretraining RNG states are saved.

The local benchmark preserves the historical nine grid targets plus five
extensions, their order, ready configuration, 400-step horizon and 2 cm success
criterion. Success also requires failure code zero. Workspace evaluation uses the
reconstructed 25-target grid X=[3,4,5,6,8] cm, Y=[-1.5,-1,-0.5,0,1] cm, Z=0.
Each result contains offsets, success flags, final errors, failure codes and steps.
This reconstruction matches the recorded success count but does not recover the
missing historical evaluator source or independently identify the old result row.

## Explicit training (not executed during refactoring)

```matlab
r = SAC_Experiment(struct('mode',"train",'curriculum',5, ...
    'maxEpisodes',20,'evaluateEvery',2));
```

Continuation loads the variable `agent` from the standardized C04 checkpoint and
validates its K settings. It does not reconstruct a new agent around actor weights
or reset the replay buffer. Policy update frequency is 5; actor, both critics and
entropy learning rates are 3e-5; gamma=0.995, batch=128, warm start=512,
replay capacity=100000, learning frequency=1, one epoch and one minibatch,
target smoothing=0.001 and target update frequency=1.
Reward weights are progress=20, step=0.01, success=10, failure=10,
joint-limit margin=0.20 rad and weight=0.02. These are not curriculum knobs.

For fresh-agent experiments, set `startCheckpoint` to the empty string. Fresh
agents use the same documented options and two 128-unit hidden layers. Evaluation
of a fresh agent remains non-training. Training a loaded agent requires the local
baseline to pass unless `requireBaselineSuccess=false` is deliberately supplied.

New curricula change configuration, not scripts. The default C05 Y range
[-0.0125,0.005] is a proposed continuation distribution. K's exact original
training ranges/RNG were not saved; C04 is the new official baseline identifier.

## Output and checkpoint policy

Evaluation returns results in memory only. Training atomically reserves the first
unused `Training_Results/SAC/Cxx/Run_NNN` directory; existing directories are never
reused. It writes:

- `config.mat`: full configuration, agent options, seeds/RNG, toolbox versions,
  source commit, runtime source hashes and starting checkpoint SHA-256.
- `starting.mat`: continuation agent and pretraining benchmark results.
- `checkpoints/Evaluation_NNNNNN.mat`: agent, local evaluation, training episode
  information and RNG state, every configured evaluation interval.
- `final.mat`: agent, training statistics and ending RNG.
- `best.mat`: selected agent, score and selected source snapshot.
- `results.mat`: baseline, final local evaluation and selection metadata.

Every checkpoint uses variable `agent`. Replay, optimizer, target and SAC policy
state saving are enabled. Best selection uses original local success rate; ties
retain the earliest snapshot, including the starting agent. Thus an already
14/14 baseline may remain selected even when a later policy improves elsewhere.
Use the final and periodic snapshots to investigate curriculum progress; selection
on new-region performance is a separate experimental decision.

A checkpoint preserves learned state, not an identical uninterrupted experiment:
MATLAB's SAC serializer omits some scheduling counters; original K RNG/session
state is unavailable. Recorded commit plus source hashes identify dirty/uncommitted
runtime source. No environment objects or personal absolute paths are deliberately
added to provenance; MATLAB training statistics may still contain its own paths.

## Baseline preservation and verification

Original: `Training_Results/SAC_K_20261120_225730/best_reaching_agent.mat`,
variable `bestReachingAgent`. Standardized:
`Training_Results/SAC/C04/SAC_C04_Agent.mat`, variable `agent`.
Both hashes are in `SAC_Checkpoint_Provenance.json`. A read-only external original
backup and historical source backup were made before standardization. Original
experiment files remain in place. MATLAB reserialization changes checkpoint bytes.

Recorded K evidence: 14/14 local; 18/25 workspace with historical agent association
based on experiment context. Refactoring evaluation independently returned 14/14
and 18/25 on the reconstructed grid. Replay count is 1204. Verification compares
actor/critic/target parameters, target state, Adam moments/update counters via
read-only struct inspection, entropy state, complete replay contents and greedy
actions at every visited workspace-benchmark observation.

```matlab
addpath('tests');
report = runSACVerification();
```

The runner also executes six existing suites: FK, action update, collision,
step/reset (22 cases), goal sampling and actual randomized environment integration.
The SAC suite covers physical mapping, reward shaping, failure/termination cases,
fixed/random resets, default and fresh evaluation, local success and migration
comparisons. Historical comparisons run when the original local artifact exists;
the legacy evaluator comparison also requires its preserved local source.
All six existing suites and the SAC suite passed locally on 2026-11-23.
Historical/new evaluator final errors differed by at most 1.94e-10 m in the full
suite (a focused rerun was exact); tests use a 1e-9 m numerical comparison bound.
Task success remains 0.02 m, with identical success, failure codes and step counts.
Tests report whether historical checkpoint comparison occurred. A source-only
checkout must obtain the official checkpoint before policy regression testing.

Training execution, interrupted-run recovery and live periodic checkpoint saving
have not been exercised: no RL training was authorized. Offline inspector and long
reset-reliability diagnostic tools remain available but are not part of this suite.

## Git and historical files

Only the two active SAC files, two new tests, documentation, provenance and scoped
ignore changes are proposed for commits. Six superseded SAC scripts/reset helpers were removed from the active project
after byte-for-byte backup verification. Their copies remain in the external
pre-refactor backup. The historical evaluator and its two normalized-step wrappers
remain locally intact and ignored for direct regression comparisons. Bulk results are ignored by directory,
not by a global MAT-file rule. The unrelated modified Simulink model and untracked
viewer remain outside this refactor's proposed commits. Distribute the protected
checkpoint as a versioned external artifact, with its hash, rather than accidentally
staging all research outputs.
