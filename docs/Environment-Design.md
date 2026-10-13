# Franka Panda RL Environment — Engineering Notes

Updated: 2026-10-13  
Course: MCT464 Selected Topics in Robotics & AI  
Status: **Pre-training validation**; do not interpret unexecuted code as a passed test.

## Assignment and design boundary

- Kinematic 7-DoF Franka Panda; finger joints held at 0.04 m.
- Observation/action interface through MathWorks `rlFunctionEnv`.
- Agent actions: seven joint increments, each bounded by $|\Delta q_i| \le 0.01$ rad per step.
- Sampling period: 0.05 s; horizon: 400 steps; positional success tolerance: 0.02 m.
- One stationary spherical obstacle. Full-arm collision checking; feasible avoidance routes are required.
- The **agent** selects all actions. IK, RRT, and nominal joint interpolation are **not** used in the policy loop.

## Existing architecture

| Responsibility | Existing implementation |
|---|---|
| Robot, joint limits, collision exclusions, course settings | `Scripts/robot_setup.m` |
| MathWorks RL observation/action specs and interface | `Scripts/create_rl_environment.m` |
| Sampling start, reachable target, and sphere | `+environment/resetFunction.m` |
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

## What was actually verified

The MATLAB results supplied for the **local 3 cm sphere configuration** showed:
- 28 of 30 randomized episode resets accepted.
- 27 of 28 accepted scenarios blocked the direct joint-space interpolation.
- None of the 28 corresponding obstacle-free interpolations were self-colliding at tested configurations.
- All 28 nominal direct interpolations fit the 400-step action-count bound.

These tests prove the sphere obstructs nominal whole-arm motion, **not** that alternative routes exist. The GitHub `main` diagnostic factory still defines a 5 cm placeholder sphere; local uncommitted settings must not be silently overwritten.

## First incremental addition: offline route-feasibility audit

Location: `tests/testEnvironment.m`, section `5B`.

- Test **only**: use built-in MathWorks `manipulatorRRT` on a copy of the robot.
- Replace both finger joints with fixed joints, preserving their original geometry at the 0.04 m opening; validate the transformed geometry before planning.
- Disable the RRT connect heuristic to retain self-collision checks.
- Use a bounded, seeded search on the first **two** accepted direct-motion-blocked scenarios.
- Independently recheck any returned path on the **original nine-joint model**, restoring the fixed finger coordinates.
- Reject candidate paths with joint-limit violations, any detected collision, or more than 400 actions when sampled at at most 0.01 rad per joint.
- Report three distinct outcomes: verified feasible route, found route exceeding horizon / invalid, or planner search inconclusive.

**Important:** The new audit is source-checked on GitHub but has **not yet been run in MATLAB**. An RRT failure is not evidence of an impossible scenario. Finite-resolution collision checking is not a continuous swept-volume guarantee.

## Remaining pre-training work

1. Run and evaluate the new offline route audit; then expand feasibility evidence to the randomized scenario distribution without using planned paths to control the RL agent.
2. Eliminate random reset-exhaustion errors for real training runs.
3. Establish an easy-to-hard curriculum, preserving separate train and held-out evaluation seeds.
4. Review reward scale, potential reward exploitation, and finite-horizon state sufficiency.
5. Benchmark step execution before attempting optimization.
6. Train and compare two continuous-control agents, then evaluate frozen policies against a conventional IK-based baseline and unseen scenarios.

## References

- MathWorks: [`rlFunctionEnv`](https://www.mathworks.com/help/reinforcement-learning/ref/rl.env.rlfunctionenv.html)
- MathWorks: [`manipulatorRRT`](https://www.mathworks.com/help/robotics/ref/manipulatorrrt.html), [`plan`](https://www.mathworks.com/help/robotics/ref/manipulatorrrt.plan.html)
- Xu et al. (2024), *Open-Source Reinforcement Learning Environments Implemented in MuJoCo with Franka Manipulator*, arXiv:2312.13788 — architectural inspiration, not a specification for this MATLAB implementation.
