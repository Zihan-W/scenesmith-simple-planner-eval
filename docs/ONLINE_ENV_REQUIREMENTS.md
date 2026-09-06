# Online Manipulation Environment Requirements

Status: Accepted implementation requirements
Last reviewed: 2026-09-06

你现在的任务不是继续开发一个写死的红盒抓取脚本，而是完成一个可交付给其他同事使用的在线机器人仿真环境。

项目目录：

`/root/workspace/scenesmith-simple-planner-eval`

## 一、产品目标

实现位于 Drake 与上层策略之间的通用执行层：

```python
env = make_env(config)

obs, info = env.reset(seed=0)

while not terminated:
    action = policy(obs)
    obs, reward, terminated, truncated, info = env.step(action)

env.write_updated_scenario(output_path)
```

后续同事应当能够：

* 替换 policy，而不修改环境；
* 替换 DMD 场景，而不修改控制器；
* 替换 task，而不修改环境核心；
* 接入 Behavior Tree 或 TAMP；
* 批量运行 episode，构建 benchmark；
* 将来通过新增 RobotAdapter 接入其他机器人。

红盒 Pick-and-Lift 只是端到端测试，不是系统架构中心。

## 二、首先封存当前成果

先检查当前未提交修改，运行现有回归并创建稳定 checkpoint，包含：

* 已人工确认的场景与红盒坐标；
* 固定导轨 0.4 m；
* PICK_HOME；
* PREGRASP；
* 双层碰撞检查；
* 在线到达 PREGRASP；
* 耦合逆动力学控制器。

不要提交废弃脚本：

`scripts/simulate_zerith_safe_pregrasp.py`

不要覆盖已有有效提交或修改上游 Zerith submodule、原始 SceneSmith 场景。

Phase 0 基线已拆分为以下 checkpoint，后续里程碑必须持续复现：

* `ab551a2`：场景、目标物体、固定导轨及抓取几何标定；
* `d5d6dc2`：双层碰撞、姿态搜索、PICK_HOME 和 PREGRASP；
* `7e8a210`：耦合逆动力学控制与在线 PREGRASP 执行。

精确回归命令和数值以 `ONLINE_ENV_PROGRESS.md` 为准。Phase 1 起只能在
这些 checkpoint 之上渐进迁移；旧 `ZerithOnlineEnv` 在 Adapter contract
通过前不得删除。

## 三、核心模块边界

### 1. Environment

环境核心只负责：

* 构建 Drake Diagram、Plant 和 SceneGraph；
* 仿真时钟和多频率调度；
* reset 与 step；
* 调用控制器；
* 提取 observation；
* 应用安全限制；
* 记录仿真结果；
* 写回最终场景。

环境核心不得包含：

* 红盒名称；
* 咖啡桌名称；
* 抓取阶段状态机；
* 当前 PREGRASP 数值；
* Zerith 的固定关节索引；
* 特定场景绝对路径；
* BT、TAMP 或具体策略逻辑。

每次 `env.step(action)` 必须准确推进一个策略周期。默认：

* Plant：1000 Hz；
* Controller：200 Hz；
* Policy：10 Hz。

策略周期内零阶保持最新控制目标。禁止通过 `SetPositions()` 实现运动。

### 2. ScenarioSpec

通过配置描述：

* DMD 路径；
* package.xml；
* 仿真和接触参数；
* 初始自由物体状态；
* Meshcat、日志和录制选项。

更换场景只能修改配置或构造参数，不能修改环境源码。

### 3. RobotSpec / RobotAdapter

机器人专用信息全部收敛到 Adapter：

* model instance；
* 基座 link；
* 受控关节；
* 锁定关节及位置；
* 末端 frame；
* 夹爪关节、方向和宽度；
* 关节及执行器映射；
* 位置、速度、力矩限制；
* 控制器增益；
* 碰撞分组；
* home configuration。

先实现 `ZerithRobotAdapter`。

导轨当前配置为固定 0.4 m。由于没有可信动力学参数，不要伪造导轨驱动；但 Adapter 必须允许未来将导轨加入受控关节。

验收原则：未来增加其他机器人时，只新增 Adapter 和配置，不修改 Environment。

### 4. Controller

将现有耦合逆动力学、重力补偿 PD 重构为通用控制器：

* 不假定机械臂一定是 7 轴；
* 不硬编码关节索引；
* 使用 RobotAdapter 提供的映射；
* 支持位置、速度和力矩限制；
* 记录限幅前后力矩；
* 检测饱和和跟踪发散；
* reset 时恢复控制器状态；
* 夹爪控制独立；
* headless 与 Meshcat 模式结果一致。

### 5. Task 接口

定义可替换的 Task 基类：

```python
class Task:
    def reset(self, env, rng): ...
    def observe(self, env): ...
    def evaluate(self, env): ...
    def allowed_contacts(self, env, action): ...
    def finalize(self, env): ...
```

至少实现：

* `NullTask`：仅测试机器人控制；
* `PickLiftTask`：当前红盒抓取测试。

Reward、success、failure、termination 和目标物体名称必须属于 Task，不属于 Environment。

### 6. Policy 与 Action

策略完全位于环境外部。

定义结构化 Action，至少支持：

* `HoldAction`
* `JointPositionAction`
* `JointDeltaAction`
* `CartesianDeltaAction`
* `GripperAction`
* 机械臂和夹爪组合命令

不得继续用含义不明确的固定长度裸数组作为唯一公共接口。可保留兼容层，但新接口必须有明确字段和单位。

BT 或 TAMP 应能在每个周期读取 observation，并发送下一条 action。

### 7. Observation

公共 observation 至少包含：

```python
{
    "time": ...,
    "robot": {
        "joint_names": ...,
        "q": ...,
        "v": ...,
        "q_commanded": ...,
        "torque_commanded": ...,
        "torque_applied": ...,
        "torque_saturated": ...,
        "end_effector_pose": ...,
        "end_effector_twist": ...,
        "gripper_width": ...,
    },
    "objects": {
        model_name: {
            "pose": ...,
            "spatial_velocity": ...,
        }
    },
    "contacts": ...,
    "task": ...,
}
```

不得把红盒作为 observation 的固定顶层字段。任务相关内容放入 `task` 或通用 `objects`。

## 四、为 BT/TAMP 提供查询接口

提供稳定、只读的 Planning/Query API：

* 获取 frame/body 世界位姿；
* 获取机器人当前 configuration；
* 检查单个 configuration；
* 检查 configuration edge；
* 查询机器人相关最小距离；
* 查询防穿透距离与安全 clearance；
* 创建独立 planning context；
* 调用通用 IK；
* 查询关节限位和当前碰撞对。

TAMP 负责生成计划，Environment 负责执行 action。不要把 RRT、TOPPRA 或任务规划器塞进 `env.step()`。

## 五、接触和安全接口

将碰撞规则抽象为可配置的 ContactPolicy，例如：

* `free_motion`
* `approach`
* `grasp`
* `carrying`

Task 决定当前允许哪些目标接触。

要求：

* 默认不允许未声明接触；
* 防穿透层和安全 clearance 层保持分离；
* 任务白名单只影响规划/安全检查；
* 不修改动力学 Plant 的真实 collision filter；
* action 被拒绝或缩放时，在 `info` 中返回明确原因；
* 抓取后必须检查携带物体与环境的碰撞。

Cartesian edge 的拒绝诊断必须明确区分关节限位、nonpenetration 和 safety
clearance。nonpenetration 失败必须记录限制它的 body/geometry pair、该 pair
相对于自身阈值的 signed-distance margin、边起点/终点距离以及全部采样距离，
以区分“安全脱离已有接触”和“继续加深穿透”。诊断不得引入新的白名单，也不得
放宽全局阈值。

## 六、场景写回

实现：

```python
env.write_updated_scenario(path)
```

要求：

* 读取结束时自由物体的真实位姿；
* 写入新的 DMD；
* 不修改输入文件；
* 固定家具保持原定义；
* 输出 DMD 能重新加载；
* 提供 write/reload round-trip 测试。

## 七、Benchmark 支撑

增加通用 EpisodeRunner：

```python
result = run_episode(
    env=env,
    policy=policy,
    seed=seed,
    max_steps=max_steps,
)
```

输出至少包括：

* success；
* termination reason；
* episode time；
* policy steps；
* 最小碰撞距离；
* 最大跟踪误差；
* 力矩饱和统计；
* 接触事件；
* 物体初始和最终位姿；
* JSON summary；
* CSV 时序日志；
* 可选 Meshcat HTML。

支持：

* headless 批量运行；
* 固定随机种子；
* 多 episode；
* 每次完整 reset；
* 结果目录不互相覆盖。
* reset-time 随机化由 seed 确定并写入结果；
* 固定初态重复性和随机扰动鲁棒性分别汇总，不能混用一个指标。

这将作为后续 benchmark 的基础。

## 八、示例与端到端验收

提供三个环境外部示例：

1. `HoldPolicy + NullTask`
2. `JointStepPolicy + NullTask`
3. `PickLiftPolicy + PickLiftTask`

PickLift 示例：

`PICK_HOME → PREGRASP → APPROACH → CLOSE → VERIFY → LIFT → HOLD`

要求：

* 导轨固定 0.4 m；
* 使用人工确认的场景和 PREGRASP；
* 在线小步控制；
* 真实双指接触；
* 禁止 weld、attach、物体瞬移；
* 禁止全局摩擦倍增；
* 红盒抬升至少 8 cm；
* 稳定保持至少 3 秒；
* 至少连续运行 3 次。

抓取失败时不得破坏通用架构去硬凑成功，也不得伪造接触。保存失败动画和诊断。

## 九、同事接入验收

README 必须让同事能在不了解 Drake 内部实现的情况下完成：

```python
class MyPolicy:
    def reset(self, obs, info):
        ...

    def act(self, obs):
        return JointDeltaAction(...)
```

并展示：

* Behavior Tree 如何在节点 tick 中调用 `env.step()`；
* TAMP 如何使用 query API 验证状态和路径，然后发送动作；
* 如何替换任务；
* 如何替换场景；
* 如何批量运行 benchmark；
* 如何保存最终 DMD。

还必须提供一个可从仓库外工作目录运行的示例。该示例只允许从
`src.online_manipulation` 公共入口导入，不得读取 Drake Context、内部关节
索引或 runtime/backend 私有对象。

不要实现完整 BT 或 TAMP，只提供稳定接口和最小接入示例。

## 十、Definition of Done

只有同时满足以下条件才算完成：

1. 环境核心没有红盒、咖啡桌和当前场景硬编码；
2. Zerith 专用内容集中在 RobotAdapter；
3. Policy 和 Task 可以独立替换；
4. `reset()` / `step()` 行为确定且有文档；
5. 多频率调度经过测试；
6. headless 批量 episode 可运行；
7. 碰撞查询和 IK 可供 TAMP 调用；
8. 最终 DMD 能写回并重新加载；
9. 三个示例策略不修改环境核心；
10. PickLift 作为端到端物理测试运行；
11. 测试、README、JSON/CSV/HTML 示例齐全；
12. Git 提交清晰且最终工作树干净。
13. 第二个可复现场景无需修改 Environment 源码即可运行。
14. MockRobotAdapter 证明核心不假定 7 轴、Zerith link、夹爪或固定动作维度。

优先级始终是：

**接口稳定性与解耦 > 控制正确性 > 可复现性 > 抓取示例效果。**

不要为了让红盒抓取成功而污染通用环境设计。持续工作直到 Definition of Done 全部满足，或遇到确实需要真实硬件参数、权限或用户决策的阻塞；普通实现和测试问题自行诊断解决。
