# Online Manipulation Environment Architecture

Status: Accepted migration architecture; Phase 4 task and query integrated
Source of truth: `docs/ONLINE_ENV_REQUIREMENTS.md`

## 0. Migration Baseline

Phase 0 is checkpointed at `ab551a2`, `d5d6dc2`, and `7e8a210`.
The current `ZerithOnlineEnv` remains the tested legacy implementation. The
new public environment will initially delegate to it through an Adapter; it
will not be replaced in one rewrite.

Migration order:

1. Define public dataclasses, protocols, and contract tests without Drake
   construction side effects.
2. Implement `ZerithRobotAdapter` and `ScenarioSpec` around the existing
   names, limits, and package registration.
3. Route typed actions and normalized observations through a compatibility
   Adapter while keeping the legacy PREGRASP regression green.
4. Extract Task, ContactPolicy, PlanningQuery, EpisodeRunner, and the DMD
   finalizer behind their contracts.
5. Remove compatibility code only after equivalent regression coverage
   exists.

## 1. Dependency Direction

```text
Behavior Tree / TAMP / Policy
              ↓
       Public Environment API
              ↓
  Environment / EpisodeRunner
       ↓        ↓        ↓
RobotAdapter  Task   PlanningQuery
       ↓
Controller / Drake Plant / SceneGraph
```

上层模块可以依赖下层模块；下层模块不得反向依赖具体策略或任务。

## 2. Public API

Phase 1 exposes the experimental public package `src.online_manipulation`
with `PUBLIC_API_VERSION = "0.1"`. Version 0.1 is additive: the existing
`ZerithOnlineEnv` entry points remain supported while the new environment
delegates to them through an Adapter. Compatibility code may be removed only
after the real Adapter regression covers the existing PREGRASP behavior.

目标接口：

```python
env = OnlineManipulationEnv(
    scenario=scenario_spec,
    robot=robot_spec,
    task=task_spec,
    timing=timing_config,
)

obs, info = env.reset(seed=0)
obs, reward, terminated, truncated, info = env.step(action)
env.write_updated_scenario(output_path)
```

外部调用者不应依赖：

* Drake Context 的内部组织；
* 固定关节索引；
* Zerith 专用 link 名称；
* 当前红盒或咖啡桌名称；
* 当前场景绝对路径。

## 3. Core Components

### OnlineManipulationEnv

负责：

* 构建和推进仿真；
* reset/step；
* 多频率调度；
* 控制器调用；
* observation 聚合；
* safety enforcement；
* recording；
* finalizer。

不得包含具体任务状态机。

### ScenarioSpec

负责：

* DMD；
* package.xml；
* 接触和仿真参数；
* 初始对象状态；
* 可视化及输出配置。

### RobotAdapter

负责所有机器人专用映射：

* model instance；
* controlled/locked joints；
* actuator mapping；
* end-effector frame；
* gripper；
* limits；
* controller gains；
* collision groups；
* home configuration。

第一个实现为 `ZerithRobotAdapter`。

The Phase 2 Zerith implementation reads position and velocity limits from the
generated URDF and combines them with the already validated servo gains and
effective effort limits. Its controlled order contains the seven left-arm
joints followed by the two gripper joints. The rail and every other movable
joint are explicit locked-joint entries; the rail value is 0.4 m for the
current pick task.

`make_legacy_zerith_environment()` is the temporary compatibility boundary.
It constructs the unchanged `ZerithOnlineEnv` from `ScenarioSpec`,
`TimingConfig`, and `ZerithRobotAdapter`. For this legacy-only function, the
first `ScenarioSpec.package_xmls` entry is the scene package and later entries
are additional packages. Target model identity remains an explicit argument
until Task owns it in Phase 4.

### Controller

输入结构化控制目标和当前机器人状态，输出受限执行器力矩。

控制器不得依赖具体任务或场景对象。

Phase 3 extracts `CoupledInverseDynamicsServo`. It accepts ordered Drake
joints and actuators plus robot-independent `JointSpec` values, computes the
full coupled inverse-dynamics command, and reports gravity, PD, raw, applied,
and saturation telemetry. The compatibility runtime receives these specs
from `ZerithRobotAdapter`; direct legacy construction retains the same
defaults.

### Task

负责：

* task observation；
* reward；
* termination；
* success/failure；
* allowed contacts；
* task metrics；
* finalization metadata。

Phase 4 provides `NullTask` and `PickLiftTask`. PickLift owns only target
observation identity, allowed gripper-target contacts, reward, success, and
the lift/hold thresholds. Motion phases remain Policy state and are not part
of Environment or Task evaluation.

### Policy

存在于 Environment 外部，只通过 observation、action 和 query API 工作。

The minimal integrations under `examples/online_manipulation` demonstrate the
boundary without adding a framework dependency. A Behavior Tree leaf advances
exactly one `env.step()` per tick. The TAMP handoff synchronizes object poses,
checks a proposed direct edge through PlanningQuery, and only then sends typed
online actions. Neither integration is imported by the environment core.

### PlanningQuery

向 TAMP 提供：

* FK；
* IK；
* configuration collision；
* edge collision；
* clearance；
* joint limits；
* 独立 planning context。

不得推进真实仿真状态。

`PlanningQuery` owns a separate RobotDiagram context initialized through the
same RobotAdapter. It provides body/frame FK, joint limits, configuration and
dense edge checks, dual-layer clearance, pose IK with independent endpoint
validation, and bounded differential-IK steps for online Cartesian commands.
Before each online command, observed free-body poses are synchronized from the
latest runtime observation into this separate context; fixed observed bodies
remain at their independently loaded scene poses. A differential-IK result is
globally scaled to preserve its joint-space direction, then the complete edge
is densely checked. These queries do not run RRT, TOPPRA, or advance the real
simulation context.

### EpisodeRunner

负责：

* seed；
* reset；
* policy loop；
* batch episodes；
* JSON/CSV/HTML；
* benchmark metrics。

## 4. Action Model

公共 Action 使用有类型的数据结构，至少包括：

* HoldAction
* JointPositionAction
* JointDeltaAction
* CartesianDeltaAction
* GripperAction
* CompositeAction

所有字段必须说明坐标系、单位和语义。

`OnlineManipulationEnv` is a typed facade over a concrete runtime backend.
The Zerith compatibility backend translates named joint position/delta and
physical gripper-width commands to the legacy 7+1 array. The public facade
does not accept bare arrays. World-frame Cartesian deltas use the configured
PlanningQuery to produce one bounded differential-IK increment and reject an
invalid edge before it reaches the servo. Unsupported frames or a missing
query fail explicitly; there is no silent fallback.

## 5. Observation Model

Observation 分为：

* time
* robot
* objects
* contacts
* task

任务对象不得硬编码成固定顶层字段。

Phase 3 normalizes the legacy runtime into these fields. Scene bodies exposed
to policies are declared with `ObservedBodySpec`; the current task's red box
appears under the caller-selected name `pick_target`, not a core field.

## 6. Timing Contract

默认：

* physics_dt = 0.001 s
* controller_dt = 0.005 s
* policy_dt = 0.100 s

一次 `step()` 精确推进一个 `policy_dt`。策略周期之间使用零阶保持。`step()` 中不得通过 `SetPositions()` 制造运动。

## 7. Collision and Contact

保留两层语义：

* nonpenetration
* safety clearance

Task 通过 ContactPolicy 声明允许的接触模式：

* free_motion
* approach
* grasp
* carrying

任务白名单只影响规划和安全检查，不改变动力学 Plant 的真实 collision filter。

The concrete representation is `PairContactPolicy`: unordered qualified-body
pairs are denied by default and must be explicitly listed. Allowed pairs are
excluded only from safety-clearance scoring; the nonpenetration layer still
checks them. Pairs whose relative pose is invariant to the active arm joints
are likewise excluded only from safety. Zerith's left shoulder-to-torso pair
is the sole explicit assembly safety exemption in RobotSpec.

`CartesianDeltaAction` initially uses one online pose-IK solve per policy
command. Translation and rotation-vector increments are expressed in world;
the delta rotation pre-multiplies the current world orientation. Other
reference frames fail explicitly until their transform semantics are added.

## 8. Migration Rule

更换场景：只改 ScenarioSpec。
更换任务：只实现 Task。
更换策略：只实现 Policy。
更换机器人：新增 RobotAdapter 和配置。

如果完成上述操作需要修改 Environment 核心，说明抽象边界失败。

## 9. Current Physical Assumptions

* Zerith 底座固定。
* 导轨固定在 0.4 m。
* 导轨真实速度和推力参数未知，因此暂不进行导轨动力学控制。
* 左臂和夹爪使用 Drake 接触动力学。
* 不允许通过 weld、attach、瞬移或全局摩擦倍增伪造抓取。

## 10. Open Decisions

Agent 在实现过程中必须记录但不得静默决定：

* DMD finalizer 的通用对象选择规则。

These remain explicit design decisions rather than hidden implementation
defaults. Phase 1 uses a Drake-independent structural mock for the first
RobotAdapter contract test. Phase 2 must add a real Zerith Adapter integration
test before any legacy path can be retired; later phases own the remaining
decisions.
