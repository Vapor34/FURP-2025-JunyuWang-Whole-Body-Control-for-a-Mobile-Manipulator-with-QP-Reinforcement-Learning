[![Mobile Dual-Arm Manipulator Research Banner](dualarm_banner.svg)](dualarm_banner.svg)

# 🤖 Mobile Dual-Arm Manipulator Research

**Whole-Body Control · Coordinated Dual-Arm Motion · QP Optimization · Reinforcement Learning**
A hardware-based research project for an over-actuated mobile manipulator on a steering-wheel chassis.

![platform](https://img.shields.io/badge/platform-Mobile%20Dual--Arm-00B8D9?style=for-the-badge)
![status](https://img.shields.io/badge/status-Hardware%20Built%20%2B%20Tested-36CFC9?style=for-the-badge)
![focus](https://img.shields.io/badge/focus-MoveIt%20%7C%20QP%20%7C%20RL-8A63D2?style=for-the-badge)
![dof](https://img.shields.io/badge/control-Whole--Body%20Coordination-FFB000?style=for-the-badge)

[Start Here](#-start-here) · [What is a Mobile Manipulator?](#-what-is-a-mobile-manipulator) · [Motivation](#-why-build-one) · [Implementation](#-implementation-path) · [Research Focus](#-research-focus) · [Media](#-media-gallery) · [References](#-references)

---

## 🧭 Start Here

This repository documents our exploration of **coordinated control for a mobile dual-arm robot mounted on a steering-wheel (omni-steerable) chassis**, working toward **whole-body control that fuses model-based optimization (QP) with reinforcement learning**.

A mobile manipulator is not just "two arms bolted onto a moving base". It is an **over-actuated, redundant system** whose base and arms must move as one. In practice, this means it can:

- 🦾 use base motion to extend the reachable workspace of both arms;
- 🤝 coordinate the two arms to grasp, hold, or carry a single object together;
- 🧭 satisfy an end-effector task while the redundant joints stay within limits;
- ⚖️ trade off mobility, manipulability, and balance through a single optimization.

> **Project statement:** operate the existing mobile dual-arm hardware platform, master arm control and MoveIt, then build a whole-body controller that coordinates base and arms using a QP layer augmented with reinforcement learning.

> **Note on scope:** the two titles — *Coordinated Control for a Mobile Dual-Arm Robot on a Steering-Wheel Chassis* and *Whole-Body Control for a Mobile Manipulator with QP + Reinforcement Learning* — describe **one and the same project** seen from two angles. The first emphasizes coordinated motion on the hardware; the second emphasizes the control method.

---

## ✨ Project at a Glance

| Item | Description |
| ----------------- | -------------------------------------------------------------------------------------- |
| 🤖 Robot type | Mobile dual-arm manipulator on a steering-wheel chassis |
| 🎯 Research target | Coordinated dual-arm motion and whole-body control (QP + RL) |
| 🧱 Hardware status | A complete mobile dual-arm platform has been constructed and tested |
| 🧠 Core challenge | Map a single Cartesian task into safe, coordinated base + arm joint commands |
| 🪜 First milestone | Get familiar with manipulator control methods and become productive in **MoveIt** |
| 🧪 Validation path | MoveIt planning → simulation → single-arm control → dual-arm coordination → whole-body |

[![Project roadmap](research_roadmap.svg)](research_roadmap.svg)

---

## 🪜 Phase 1 — First Milestone (MoveIt Onboarding)

Before any whole-body or learning work, the **first goal is to get productive quickly**. This phase is about building intuition and tooling fluency, not novel research.

- [x] Read the URDF / robot description and confirm the kinematic chains for both arms.
- [ ] Bring up the robot model in **RViz** and inspect TF frames.
- [ ] Configure **MoveIt** for the dual-arm setup (planning groups, end effectors, collision model).
- [ ] Plan and execute simple point-to-point motions for one arm.
- [ ] Plan motions for both arms (separate planning groups first).
- [ ] Add basic collision objects and verify collision-aware planning.
- [ ] Script motions through the MoveIt API (Python / C++) instead of the GUI.
- [ ] Document a "hello MoveIt" tutorial so the next student can reproduce it.

> The exit criterion for Phase 1: a student can describe the robot's kinematics, set up MoveIt for both arms, and command collision-free motions from code.

---

## 🤖 What is a Mobile Manipulator?

A fixed-base manipulator can only reach within the workspace defined by its link lengths. A **mobile manipulator** mounts the arm(s) on a moving base, so the reachable workspace becomes (in principle) unbounded — but the price is **redundancy and coupling** between base and arm motion.

```
Want to reach a far target?
    → move the base toward it
    → arm pose and base pose both affect the end effector
    → mobility and manipulation are coupled
```

For our platform, a desired end-effector motion $\dot{\mathbf{x}}$ relates to the whole-body joint velocities $\dot{\mathbf{q}}$ through the **whole-body Jacobian** $J$:

$$
\dot{\mathbf{x}} = J(\mathbf{q})\,\dot{\mathbf{q}},
\qquad
\mathbf{q} =
\begin{bmatrix}
\mathbf{q}_{base} \\
\mathbf{q}_{arm,L} \\
\mathbf{q}_{arm,R}
\end{bmatrix}
$$

where:

- $\mathbf{q}_{base}$ describes the steering-wheel chassis configuration;
- $\mathbf{q}_{arm,L}$ and $\mathbf{q}_{arm,R}$ are the left and right arm joints;
- $J(\mathbf{q})$ stacks the base and arm contributions to each end effector.

Because the system has more actuated degrees of freedom than a single 6-DoF task requires, there is a **null space** that can be used for secondary objectives such as keeping joints centered, avoiding obstacles, or improving manipulability.

### 🧩 Fixed-base arm vs. Mobile dual-arm manipulator

| Capability | Fixed-base single arm | Mobile dual-arm manipulator |
| ---------------------------------------- | --------------------- | --------------------------------- |
| Bounded reachable workspace | ✅ (limited) | ✅ (effectively unbounded) |
| Extend reach by moving the base | ❌ | ✅ |
| Carry a large object with two arms | ❌ | ✅ |
| Redundancy / null-space optimization | Limited | Strong |
| Base–arm motion coupling | None | High |
| Control difficulty | Medium | High |

---

## 💡 Why Build One?

The design motivation is simple: **many real manipulation tasks need both mobility and two hands**.

A fixed arm is excellent for bench tasks, pick-and-place, and structured cells. But its fixed base becomes a bottleneck when the robot needs to:

- 🚪 reach targets spread across a room larger than any single arm's workspace;
- 📦 lift and carry bulky objects that one gripper cannot hold alone;
- 🤝 perform bimanual tasks where two arms must hold a fixed relative pose;
- 🧪 test whole-body control, redundancy resolution, and learning-based control;
- ⚖️ coordinate base and arms so motion is smooth instead of "drive, stop, then reach".

For our project, the mobile dual-arm robot is valuable not because it is easy to control, but because it is a **rich control problem**:

```
Steering-wheel chassis kinematics
    ↓
Whole-body Jacobian (base + two arms)
    ↓
QP-based whole-body control
    ↓
Coordinated dual-arm motion
    ↓
Reinforcement learning augmentation
```

---

## 🛠️ Implementation Path

This project builds up in stages, from tooling to coordination to learning:

1. **MoveIt onboarding** — model bring-up, planning groups, collision-aware motion (Phase 1 above).
2. **Single-arm control** — Cartesian and joint control for one arm, verified in sim and on hardware.
3. **Dual-arm coordination** — both arms tracking a shared task or a fixed relative grasp.
4. **Whole-body control (QP)** — base + arms solved together as a constrained optimization.
5. **Reinforcement learning** — learn residual policies or high-level decisions on top of the QP layer.

---

## 🚗 Steering-Wheel Chassis

[![Steering-wheel chassis kinematics](chassis_kinematics.svg)](chassis_kinematics.svg)

The base uses **steering-wheel (steerable) modules** rather than fixed differential wheels. Each module can steer and drive, which gives the chassis a wide motion envelope but also makes its kinematics more involved than a simple unicycle model.

### 🧠 Chassis design logic

| Design idea | Why it matters |
| ------------------------------- | ----------------------------------------------------------- |
| 🛞 Steerable wheel modules | Wider motion envelope than fixed differential drive |
| 🧭 Rich base mobility | Lets the base actively support manipulation tasks |
| 🔗 Base–arm kinematic coupling | Base motion directly enters the whole-body Jacobian |
| ⚙️ Nonholonomic / steering constraints | Must be respected inside the whole-body controller |
| 🧮 Unified configuration vector | Treat base and arms as one system, not two subsystems |

### ⚠️ Practical issues

- Steering modules add wheel-steer angle states that must be modeled and constrained.
- Base motion is generally slower and less precise than arm motion, so the controller must weight them appropriately.
- Wheel slip and odometry drift introduce model mismatch at the base.
- Coordinating fast arm motion with slower base motion requires careful task prioritization.

---

## 🤝 Dual-Arm Coordination

[![Dual-arm coordination concept](dualarm_coordination.svg)](dualarm_coordination.svg)

The two arms can be controlled in several modes, in increasing difficulty:

| Mode | Description | Difficulty |
| ------------------------- | ------------------------------------------------------ | ---------- |
| 🅰️ Independent | Each arm tracks its own goal, no coupling | Low |
| 🔗 Relative-pose (rigid) | Arms hold a fixed relative transform (carry one object) | Medium |
| 🌀 Whole-body coordinated | Base + both arms solved together for one task | High |

For a bimanual task carrying a shared object, the **relative pose** between the two end effectors should stay constant:

$$
{}^{L}\mathbf{T}_{R} = \text{const}
\quad\Longrightarrow\quad
J_{rel}\,\dot{\mathbf{q}} = 0
$$

so the controller must drive the absolute object motion while holding the relative-pose constraint, all within base and joint limits.

---

## 🧮 Modeling Foundation

The robot is modeled as a redundant kinematic (and optionally dynamic) system. The key object is the **whole-body Jacobian** $J$ stacking base and arm contributions for each end-effector task.

### 🔍 What we need to identify

| Model item | Why we need it |
| ----------------------------- | --------------------------------------------- |
| Arm DH / URDF parameters | Forward kinematics and Jacobians |
| Chassis kinematic model | Maps wheel/steer commands to base motion |
| Steering constraints | Required for feasible base commands |
| Joint position / velocity limits | Constraints in the QP |
| Base velocity / acceleration limits | Keep motion feasible and safe |
| Self-collision geometry | Avoid arm–arm and arm–base collisions |
| End-effector / payload properties | Needed for bimanual carrying and dynamics |

### 🧪 Suggested workflow

```
1. Confirm robot description
   URDF, joint limits, frames, planning groups

2. Verify forward kinematics
   joint angles -> end-effector pose (RViz / MoveIt)

3. Build the whole-body Jacobian
   stack base + left arm + right arm

4. Add constraints
   joint limits, base limits, steering, self-collision

5. Validate in simulation
   commanded task vs. achieved end-effector motion

6. Transfer to hardware
   start slow, log everything, compare sim vs. real
```

---

## 🧠 Research Focus

[![Whole-body control stack](control_stack.svg)](control_stack.svg)

This project lands on three major control directions:

1. **Coordinated / Whole-Body Control** — solve base and arms together so the robot moves as one.
2. **QP-Based Optimization** — turn the task, constraints, and priorities into a real-time quadratic program.
3. **Reinforcement Learning** — augment the model-based controller with learned policies for hard-to-model behavior.

---

## 🎯 1. Whole-Body Control

Whole-body control is used when we want the robot to satisfy a task while exploiting redundancy and respecting constraints.

The most immediate problem is **redundancy resolution**:

> Given a desired end-effector motion, choose base + arm joint velocities that achieve it safely while keeping secondary objectives satisfied.

A simple baseline is the pseudo-inverse with null-space projection:

$$
\dot{\mathbf{q}} = J^{\dagger}\dot{\mathbf{x}}
+ (I - J^{\dagger}J)\,\dot{\mathbf{q}}_{0}
$$

where the second term moves the redundant joints toward a secondary goal $\dot{\mathbf{q}}_0$ (e.g. joint centering) without disturbing the primary task. This is easy to implement, but it cannot directly enforce inequality constraints such as joint limits or self-collision avoidance — which is why we move to a QP.

---

## ⚙️ 2. QP-Based Control

A more practical whole-body controller is written as a **quadratic program** solved at every control step:

$$
\begin{aligned}
\min_{\dot{\mathbf{q}}} \quad &
\lVert J\dot{\mathbf{q}} - \dot{\mathbf{x}}_{des} \rVert_{W}^{2}
+ \alpha \lVert \dot{\mathbf{q}} \rVert^{2} \\
\text{s.t.} \quad &
\dot{\mathbf{q}}_{min} \le \dot{\mathbf{q}} \le \dot{\mathbf{q}}_{max} \\
& \mathbf{q}_{min} \le \mathbf{q} + \dot{\mathbf{q}}\,\Delta t \le \mathbf{q}_{max} \\
& \text{(steering, base, and self-collision constraints)}
\end{aligned}
$$

where:

- $\lVert J\dot{\mathbf{q}} - \dot{\mathbf{x}}_{des}\rVert_W^2$ tracks the desired task (with task weighting $W$);
- $\lVert\dot{\mathbf{q}}\rVert^2$ regularizes / minimizes motion effort;
- the constraints enforce joint, base, steering, and collision limits.

Multiple tasks (e.g. both end effectors plus a relative-pose constraint) can be stacked with priorities or weights.

### 🧭 QP research questions

- How should base motion and arm motion be weighted when both can serve a task?
- Can task hierarchies (strict priorities) outperform soft weighting for this robot?
- How do steering constraints change the attainable task space?
- Can the QP run fast enough on the onboard computer for smooth control?
- How well does a sim-tuned QP transfer to the real chassis?

---

## 🧠 3. Reinforcement Learning Augmentation

[![QP plus RL architecture](qp_rl_architecture.svg)](qp_rl_architecture.svg)

Pure model-based control struggles with effects that are hard to model: wheel slip, contact, payload variation, and complex task decisions. **Reinforcement learning** can augment — not replace — the QP layer.

| Strategy | Role in this project |
| ---------------------------- | ---------------------------------------------------------- |
| Residual policy | Learn a correction added on top of the QP solution |
| Task / reference shaping | Learn good task references for the QP to track |
| High-level decision policy | Decide *when* to move the base vs. the arms |
| Learned constraint tuning | Adapt QP weights online from experience |
| Sim-to-real transfer | Train in simulation, deploy with domain randomization |

```
task goal
    ↓
RL policy (high level / residual)
    ↓
QP whole-body controller (constraints enforced)
    ↓
base + dual-arm joint commands
    ↓
robot + state estimation ──► reward / feedback ──► RL policy
```

Keeping the QP in the loop means the **safety constraints are always enforced**, even while the learned policy is still improving.

### 🧪 RL research questions

- Does a residual policy improve tracking over the QP alone, without breaking constraints?
- Can RL learn base/arm coordination decisions better than hand-tuned heuristics?
- How large is the sim-to-real gap for the steering-wheel chassis?
- What is the safest way to deploy a partially trained policy on real hardware?

---

## 🧪 Experiment Plan

### Phase 1 — MoveIt onboarding
- Bring up the model, configure MoveIt, plan single- and dual-arm motions, script via API.

### Phase 2 — Single-arm control
- Implement and verify Cartesian / joint control for one arm in sim, then on hardware.

### Phase 3 — Dual-arm coordination
- Track shared tasks and a fixed relative grasp; verify the relative-pose constraint.

### Phase 4 — Whole-body QP control
- Build the whole-body Jacobian, implement the QP, add base/steering/collision constraints.

### Phase 5 — RL augmentation
- Train residual / high-level policies in simulation; deploy carefully with the QP as a safety layer.

---

## 🧰 Suggested Repository Structure

```
mobile-dualarm-manipulator-research/
├── README.md
├── 
│   ├── dualarm_banner.svg
│   ├── research_roadmap.svg
│   ├── chassis_kinematics.svg
│   ├── dualarm_coordination.svg
│   ├── control_stack.svg
│   ├── qp_rl_architecture.svg
│   └── hardware_prototype.jpg
├── docs/
│   ├── moveit_setup.md
│   ├── kinematics_notes.md
│   ├── whole_body_control.md
│   ├── qp_formulation.md
│   ├── rl_notes.md
│   └── experiment_logbook.md
├── description/
│   ├── urdf/
│   └── moveit_config/
├── control/
│   ├── single_arm/
│   ├── dual_arm/
│   ├── whole_body_qp/
│   └── rl/
├── simulation/
│   ├── gazebo/
│   └── python/
├── experiments/
│   ├── moveit_tests/
│   ├── single_arm_tests/
│   ├── dual_arm_tests/
│   └── whole_body_tests/
└── references/
    ├── papers.md
    └── links.md
```

---

## ✅ Safety Rules

A mobile dual-arm robot can move its base and swing two arms at once. Treat every test as high-risk.

- 🛑 Keep a physical emergency stop within reach during every powered test.
- 🐢 Start every new controller at reduced speed and reduced workspace limits.
- 🧍 Keep people clear of the base motion area and the arm sweep volume.
- 🚧 Always enable self-collision and base/arm limit checks before running.
- 🪢 Test arm control with the base disabled first, then enable base motion.
- 🔋 Monitor battery, motor temperature, and current during long runs.
- 📝 Log the exact configuration, parameters, and software version for each test.
- 🧪 Change one thing at a time, and never run learned policies without the QP safety layer.

---

## 📊 Evaluation Metrics

| Category | Metric |
| ----------------- | --------------------------------------------------------- |
| Task tracking | end-effector position / orientation RMS error, max error |
| Coordination | relative-pose error during bimanual tasks |
| Whole-body usage | base vs. arm contribution, joint-limit margin |
| QP performance | solve time, constraint violations, smoothness |
| Redundancy | manipulability index, null-space utilization |
| RL augmentation | improvement over QP baseline, constraint compliance |
| Sim-to-real | gap between simulated and real tracking |
| Safety | aborted runs, e-stop triggers, collision-check correctness |

---

## 🧾 Key Takeaways

- 🤖 A mobile dual-arm manipulator is a redundant, over-actuated system where base and arms must move together.
- 🪜 The first milestone is practical: get fluent with manipulator control and **MoveIt** before any research.
- 🚗 The steering-wheel chassis makes the base genuinely useful for manipulation, but adds kinematic constraints.
- ⚙️ A QP-based whole-body controller turns tasks, priorities, and constraints into real-time joint commands.
- 🧠 Reinforcement learning augments the QP layer — the QP stays in the loop to keep constraints enforced.
- 🛠️ Our hardware platform has already been built and tested, making this project suitable for real control experiments.

---

## 🔗 References

### Software and tooling

1. **MoveIt Motion Planning Framework** — <https://moveit.ai/>
2. **MoveIt Tutorials** — <https://moveit.picknik.ai/>
3. **ROS Documentation** — <https://docs.ros.org/>

### Whole-body control and QP

1. Whole-body control surveys and tutorials on hierarchical / QP-based inverse kinematics for mobile manipulators.
2. Quadratic-programming–based redundancy resolution with inequality constraints (joint limits, collision avoidance).

### Reinforcement learning for manipulation

1. Residual reinforcement learning for robotic control (learning corrections on top of a model-based controller).
2. Sim-to-real transfer with domain randomization for mobile manipulation.

> Fill in specific papers, course notes, and links as the project progresses.

---

**Built with steered wheels, two arms, joint-limit constraints, QP solvers, and a healthy fear of self-collision.**
🤖🦾🧮📈
