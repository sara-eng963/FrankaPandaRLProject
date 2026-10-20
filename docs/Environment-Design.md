# Franka Panda RL Environment — Engineering Notes

Updated: 2026-10-20
Course: MCT464 Selected Topics in Robotics & AI  
Status: **Pre-training validation**; do not interpret unexecuted code as a passed test.

## Assignment and design boundary

- Kinematic 7-DoF Franka Panda; finger joints held at 0.04 m.
- Observation/action interface through MathWorks `rlFunctionEnv`.
- Agent actions: seven joint increments, each bounded by $|\Delta q_i| \le 0.01$ rad per step.
- Sampling period: 0.05 s; horizon: 400 steps; positional success tolerance: 0.02 m.
- One stationary spherical obstacle. Full-arm collision checking; feasible avoidance routes are required.
- The **agent** selects all actions. Offline diagnostics do not select policy actions.

## Existing architecture

| Responsibility | Existing implementation |
|---|---|
| Robot, joint limits, collision exclusions, course settings | `Scripts/robot_setup.m` |
| MathWorks RL observation/action specs and interface | `Scripts/create_rl_environment.m` |
| Shared validated randomized-scenario configuration and reusable IK solver | `+environment/configureScenarioParameters.m` |
| Sampling start, reachable target, and sphere | `+environment/resetFunction.m` |
| Bounded complete-scenario retries with one fixed method per requested episode | `+environment/resetEpisodeWithRetries.m` |
| Cartesian target proposals and position-only IK validation | `+environment/sampleGoalConfiguration.m` |
| State transition, collision penalties, reward, termination | `+environment/stepFunction.m` |
| Full-arm collision and end-effector forward kinematics | `+collision/checkRobotCollision.m`, `+kinematics/forwardKinematics.m` |
| Regression and scenario diagnostics | `tests/*.m` |

The 14-component observation is

$$
o_t = \begin{bmatrix}
q_{1:7,t} \\
p_g - p_{EE}(q_t) \\
p_{obs} \\
r_{obs}
\end{bmatrix} \in \mathbb{R}^{14}.
$$

Seven joints encode the kinematic robot state; goal error and sphere geometry encode the task. Explicit clearance would require additional computation and has not been justified. Remaining episode time is not observed: strict finite-horizon Markov sufficiency therefore warrants review before final training.

Current reward (not yet empirically calibrated):

$$
r_t = w_p(d_{t-1}-d_t)-w_s
$$

plus the existing success bonus or terminal failure penalty. The numerical scales should be evaluated before changing weights.

## Creating diagnostic and training environments

Run from the project root:

```matlab
environmentMode = "diagnostic";
run('Scripts/create_rl_environment.m');
obs = reset(env);

environmentMode = "training";
run('Scripts/create_rl_environment.m');
obs = reset(env);
```

If `environmentMode` is absent, creation defaults to **diagnostic**: the original ready configuration, fixed target, distant 3 cm placeholder sphere, and raw single-scenario reset. The scenario diagnostic explicitly requests this mode, regardless of a caller's previous training selection. Each creation starts with fresh state/parameter structures so mode switches cannot retain randomized fields.

**Training** uses the shared validated settings in `environment.configureScenarioParameters`, also used by `tests/testEnvironment.m` and consequently the existing reliability benchmark. Start half-widths, IK settings, workspace, 15–50 cm target distances, 3–6 cm radii, obstacle ranges and RRT parameters are unchanged. Training explicitly sets `mixed`, probability `0.5`, and `maxScenarioAttempts=10`. Its reset callback is `@() environment.resetEpisodeWithRetries(initialState)`; each requested episode selects one method and keeps it through its bounded complete retries. Exhaustion and unexpected errors propagate. The wrapper's standalone default remains five; training's ten-attempt limit is explicit.

The IK solver is constructed once per randomized environment, with construction isolated from the caller's RNG. Normal resets advance the active random stream; they do not reseed. Both modes use `@environment.stepFunction` and the unchanged 14D observation and 7D action specifications. The accepted reset state passes through the documented `env.Info` property, not the deprecated `LoggedSignals` property. IK witnesses remain internal state and RRT paths are not policy inputs.

Training disables scenario/candidate-history recording, arm visualization geometry and robot setup figures. Only the wrapper's lightweight retry statistics remain; its elapsed timing is disabled for the two-output RL reset callback. Standalone robot setup retains its visualization default.

`rlFunctionEnv` automatically validates its callbacks on construction, including a reset and step; creating a training environment can therefore consume randomized scenarios and time before the first explicit `reset(env)`. Explicit `validateEnvironment(env)` also resets/steps it. Set the external experiment seed **before construction** when reproducing an entire initialization sequence. Keep `cfg.evalSeed` reserved. [MathWorks R2025b reference](https://www.mathworks.com/help/releases/R2025b/reinforcement-learning/ref/rl.env.rlfunctionenv.html).

Manual interface smoke check:

```matlab
trainingEnvironmentTestOptions = struct('nResets',5,'seed',314159);
run('tests/testTrainingEnvironment.m');
```

This checks the actual environment interface, state handoff, consecutive variation, external seeded replay, both explicit modes and the fixed default. It uses the public `Info`, `StepFcn`, specification accessors and `validateEnvironment`; it does not invoke the retry wrapper directly. Setup figures are hidden and cleaned up. The existing raw diagnostic, reliability benchmark and offline inspector remain separate workflows. The user reported 100/100 successful mixed resets at a ten-attempt limit; this environment integration and its new test have **not** been runtime-validated as part of the source changes.

## What was actually verified

Position-only IK verification on 2026-10-19 using MATLAB R2025b:

- The 3-episode smoke test accepted 3/3 resets. The 30-episode diagnostic accepted 23/30; all seven failures exhausted obstacle placement, with no start, target-generation or RRT failures.
- All 30 generated targets passed the independent IK/FK feasibility checks on the first guess. There were no rejected IK solutions or solver timeouts. Accepted RRT routes averaged 136 steps and reached at most 217 steps; 22/23 direct joint interpolations were obstructed by the sphere.
- Focused IK tests and all four legacy regression scripts passed, including the 22 reset/step cases, seeded replay and observation checks. Focused tests also force bounded IK exhaustion and verify that changing the requested rotation has no effect with zero orientation weights.
- A fresh isolated snapshot of the previous FK implementation, with identical starts, seeds and scenario parameters, accepted 24/30 resets. Mean successful reset time was 2.27 s for FK versus 1.45 s for IK; mean goal generation time was 40.7 ms versus 21.6 ms. These are single-run timings, not a speed guarantee.
- Observed 10×10×10 target occupancy was 2.3% for FK and 2.2% for IK. At 23 accepted points each, both occupied 2.2% of cells. The small sample provides no evidence of improved overall coverage. Goal rejection changes downstream radius, obstacle and RRT draws even when starting seeds match.

Previous FK-based cleanup verification on 2026-10-19 using MATLAB R2025b:
- The 3-episode diagnostic accepted 3/3 scenarios; the 30-episode diagnostic accepted 24/30.
- Six resets exhausted obstacle placement; none failed start or target generation. All 24 accepted scenarios passed RRT validation, with a maximum validated route length of 330 steps.
- FK, workspace, endpoint collision safety, fixed fingers, radius/observation consistency, rejection accounting, seeded replay, and caller RNG preservation checks passed.
- All four regression scripts listed below passed, including the 22 reset/step test cases. These checks do not establish reset reliability for every seed or continuous swept-path clearance.

Historical MATLAB results supplied for the **local fixed 3 cm sphere configuration** showed:
- 28 of 30 randomized episode resets accepted.
- 27 of 28 accepted scenarios blocked the direct joint-space interpolation.
- None of the 28 corresponding obstacle-free interpolations were self-colliding at tested configurations.
- All 28 nominal direct interpolations fit the 400-step action-count bound.

These historical checks establish obstruction of sampled nominal whole-arm motion, not alternative-route feasibility. They are not measurements of the current randomized 3–6 cm sphere configuration. The default diagnostic interface fixture in `Scripts/create_rl_environment.m` uses a distant 3 cm placeholder sphere; training mode and the scenario diagnostic configure randomized obstacles separately.

Two alternative routes were also validated in an earlier exploratory MATLAB run (131 and 224 joint-increment steps). Those historical examples do not establish feasibility for the full randomized distribution.

## Current scope

Starts retain uniform joint-space sampling. Goals are proposed uniformly in the unchanged Cartesian workspace, distance-filtered, and validated by position-only IK. Radius-aware sphere placement, exact full-arm collision checks, and RRT feasibility validation remain in place. RRT validates candidate scenarios through `checkScenarioFeasibility.m`; neither IK nor RRT supplies actions or paths to the RL agent. Accepted routes are rechecked at increments of at most 0.01 rad per arm joint and must fit 400 steps. These discrete checks do not prove continuous swept-path clearance.

`tests/testEnvironment.m` runs one online scenario diagnostic and configures the reusable solver outside `resetFunction.m`:

- Original arm-joint half-widths about `qReady`: `[30 40 30 30 30 35 40]` degrees.
- `startRangeMultiplier = 1`; start intervals are clamped to the robot model's physical joint limits. Goal joint intervals are no longer configured; IK may use all physical arm-joint limits.
- `targetDistanceRange = [0.15 0.50]` metres. An infinite upper bound omits the optional maximum-distance filter. The minimum must exceed the existing position tolerance. Both requested XYZ and the FK witness must satisfy the workspace and distance constraints.
- Goal randomization is enabled by a prebuilt `p.ikSolver`. Omit it only for intentionally fixed-target fixtures. The obsolete `goalJointRanges` setting no longer enables goal randomization.
- `inverseKinematics` uses Levenberg–Marquardt and weights `[0 0 0 1 1 1]`: all orientation weights are zero. The transform's rotation is therefore unconstrained. [MathWorks documents the weights and solver interface.](https://www.mathworks.com/help/robotics/ref/inversekinematics-system-object.html)
- Defaults: `maxTargetAttempts = 200` Cartesian proposals, `ikMaxInitialGuesses = 3` per distance-admissible target, `ikMaxIterations = 100`, `ikMaxTime = 1` second per solve, and `ikPositionTolerance = 0.001` metres. The solver's `ikSolutionTolerance = 1e-6` is also configurable. The independent FK tolerance must not exceed the task position tolerance.
- The first guess is the sampled start; later guesses uniformly sample the physical arm limits. Fixed fingers are restored before checking FK, position error, physical limits, workspace/distance bounds and exact self-collision (plus a static obstacle, when applicable). Every returned witness passes these checks, even if the solver reports only a best-available iterate.
- Automatic IK random restarts are disabled; explicit retries use the reset's random stream. Solver construction is isolated from the caller's RNG in the diagnostic. A binding time limit can affect repeatability under load; time-limit exits are reported separately.
- Sphere radius is drawn once per reset from `[0.03 0.06]` metres. Collision and workspace checks still reject unsuitable samples.
- Raw single-scenario attempts remain bounded. Exhaustion is reported by stage in diagnostic mode; raw normal calls retain their existing errors. Training uses the explicit complete-scenario wrapper and raises a named error if all ten complete attempts fail.

The diagnostic accepts overrides without changing the reset algorithm:

```matlab
environmentDiagnosticOptions = struct('nEpisodes',3,'doVisualize',false, ...
    'startRangeMultiplier',1,'ikMaxInitialGuesses',3, ...
    'targetDistanceRange',[0.15 0.50],'seed',314159);
run('tests/testEnvironment.m');
environmentDiagnosticOptions.nEpisodes = 30;
run('tests/testEnvironment.m');
```

`halfWidthsDegrees` optionally overrides the seven start half-widths. The diagnostic uses private per-episode random streams and restores the caller's global stream, including on errors. It replays an accepted reset to check reproducibility and retains generated scenarios only for inspection. `sampledGoalQ` is only a feasibility witness for endpoint checks and RRT; the agent receives only the unchanged 14D observation. IK and RRT do not run in `stepFunction.m`. There are no precomputed goals, episode pools, or Cartesian grid guidance. The 10×10×10 grid in the diagnostic only measures observed coverage.

Uniform candidate XYZ does not imply uniform accepted goals: distance filtering, finite IK search, collision checking, obstacle placement and RRT all change the accepted distribution. Bounded IK failure does not prove mathematical unreachability.

`tests/testGoalSampling.m` checks position-only weighting, fixed fingers, FK accuracy, exact safety checks, bounded search exhaustion and RNG reproducibility. Legacy regression scripts are `tests/testForwardKinematics.m`, `tests/testActionUpdate.m`, `tests/testRobotCollision.m`, and `tests/testStepFunction.m`. The last retains randomized reset, workspace, observation, failure-handling, and optional-diagnostics checks with the new goal interface.

## Remaining pre-training work

1. Establish a dependable basic RL training scenario; retain the feasible-route requirement and distinguish discretely validated accepted routes from reset reliability over the full randomized distribution.
2. Eliminate randomized reset-exhaustion errors before unattended training.
3. Review reward scale and behavior; test observation correctness and terminal conditions.
4. Benchmark environment-step execution before optimizing.
5. Add a gradual curriculum and separated training/evaluation seeds.
6. Train and compare two continuous-control agents; evaluate frozen policies on unseen cases and compare with the conventional IK baseline.

## References

- MathWorks: [`rlFunctionEnv`](https://www.mathworks.com/help/reinforcement-learning/ref/rl.env.rlfunctionenv.html)
- Xu et al. (2024), *Open-Source Reinforcement Learning Environments Implemented in MuJoCo with Franka Manipulator*, arXiv:2312.13788 — architectural inspiration, not a specification for this MATLAB implementation.
