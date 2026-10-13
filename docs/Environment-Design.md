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
- The **agent** selects all actions. Offline diagnostics do not select policy actions.

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

## Current scope

The environment keeps the core reset/step functions and the two test-only nominal-motion collision diagnostics. Optional motion-planner feasibility code has been removed from the test suite to keep the implementation focused.

Two alternative paths were validated in a previous exploratory MATLAB run (131 and 224 joint-increment steps). This **does not establish feasibility for the full randomized distribution**. Collision-free endpoints alone never guarantee an avoidance route. A rigorous future evaluation should address this course requirement, without supplying paths to the RL policy.

## Remaining pre-training work

1. Establish a dependable basic RL training scenario; retain the assignment's feasible-route requirement and explicitly label randomized feasibility as not yet guaranteed.
2. Eliminate randomized reset-exhaustion errors before unattended training.
3. Review reward scale and behavior; test observation correctness and terminal conditions.
4. Benchmark environment-step execution before optimizing.
5. Add a gradual curriculum and separated training/evaluation seeds.
6. Train and compare two continuous-control agents; evaluate frozen policies on unseen cases and compare with the conventional IK baseline.

## References

- MathWorks: [`rlFunctionEnv`](https://www.mathworks.com/help/reinforcement-learning/ref/rl.env.rlfunctionenv.html)
- Xu et al. (2024), *Open-Source Reinforcement Learning Environments Implemented in MuJoCo with Franka Manipulator*, arXiv:2312.13788 — architectural inspiration, not a specification for this MATLAB implementation.
