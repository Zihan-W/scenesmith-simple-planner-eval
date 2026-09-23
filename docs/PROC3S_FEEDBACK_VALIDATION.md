# PRoC3S 抓取前几何失败反馈闭环验证

日期：2026-09-22；开发机 `180.76.233.8:6308`；仓库 `/root/workspace/scenesmith-simple-planner-eval`。

## 结论与验收范围

**本次约定的闭环已在 `live_feedback_07` 同一轮真实仿真中成功。** 验证范围为“抓取前几何失败 → 在线模型修改技能程序 → 重新执行 → 实际抓取、抬升并保持成功”。用户明确选择先完成这一范围；物体已夹住后的抬升／保持失败恢复仍是后续缺口。

| 实测项目 | 结果 |
|---|---:|
| 最终物体抬升 | 8.1860 cm（要求 8 cm） |
| 最终连续保持 | 3.1000 s（要求 3 s） |
| 双侧手指接触 | True |
| 桌面支撑接触 | False |
| 非预期目标接触 | [] |
| 实际仿真步数／时间 | 274 / 27.40 s |
| 技能结构重规划／执行次数 | 1 / 3 |
| 在线 VLM／技能 LLM 调用 | 1 / 4 |
| 执行环境 reset 次数 | 1 |
| 原始终止原因 | task_goal_verified |

参考环境只用于核对物体 +10 cm 的初始位移，未执行动作；它与执行环境分别记账。没有在失败后 reset 后接续冒充恢复成功。所有已开始的失败／中止试验均保留。

## 验证过程与故障注入边界

保持原场景设置：物体沿世界 +X 移动 0.100 m，机器人初始位姿不变，只有 `NavigateToPick`、`PickLift` 两个技能，使用真实在线模型与原生产采样器。

首次独立 PickLift 求解时，夹具通过已有的 `grasp_arm_joint_positions` 输入故意提交当前收拢手臂配置。真实 FK／几何检查拒绝该配置；重复这个已拒绝候选使 64 次预算耗尽，随后立即恢复原采样器。没有注入检查结论、模型响应、符号事实、控制指令或物理成功。

**注入的是错误抓取候选，没有手工指定失败停车点。** 初始停车由原采样器生成并实际执行。故障影响了程序结构和后续随机采样过程；下一停车范围还依赖实测朝向。这是受控候选源故障测试，不是自然失败率测量，也不证明重新停车是必需或最优的恢复动作。

模型收到的失败摘要如下（原程序和已排除骨架也随请求提供）：

```json
[
  {
    "skill": "PickLift",
    "failed_constraints": [
      "ik"
    ],
    "involved_objects": [
      "pick_target"
    ],
    "program_unsat": true,
    "search_budget_exhausted": true
  }
]
```

实际链路：

```mermaid
flowchart LR
  A[实测停车状态] --> B[PickLift 候选被真实几何检查拒绝]
  B --> C[预算耗尽与失败摘要送入在线模型]
  C --> D[模型生成 NavigateToPick → PickLift]
  D --> E[重新求解并执行导航]
  E --> F[新实测状态：清除旧失败反馈]
  F --> G[重新求解并执行 PickLift]
  G --> H[实测高度和保持时间满足任务]
```

故障发生于 t=7.80 s。重新导航之后，日志中有 `program_feedback_reset`；后续模型请求反馈和骨架排除集合为空，CCSP 使用新观测及同步 planning query。执行器实际消费几何参数，并使用独立 `single_skill` 路径。成功由环境观测确认。

### 导航结束时 dipan_link 相对物体的位置

世界坐标中取 `dipan_link - 物体`，单位 cm：

| 技能序号 | 仿真时间 s | X | Y | 水平距离 | 导航成功 |
|---|---:|---:|---:|---:|---|
| 1 | 7.80 | 57.907 | 17.996 | 60.638 | True |
| 2 | 13.80 | 66.528 | 17.595 | 68.815 | True |

## 本次修复

1. `planner/src/tamp/failures.py`：关节边检查的关节限位失败不再误报为碰撞；保持超时、接触丢失和运动阶段超时映射到明确类别。原始数值诊断留在低层，模型只接收允许的抽象反馈字段。
2. `planner/src/tamp/online.py`：已验证的前置技能成功后，清除属于旧状态的抽象失败，匹配已有的骨架排除集合清除行为。修复前的契约测试能复现残留反馈，修复后通过，并已由本轮实际执行确认。
3. `simulation/src/control/navigation.py`：为紧停车容差保留朝向对准的进入／退出间隔。5 mm 到达容差下，进入对准的距离改为约 3.33 mm，退出边界仍为 5 mm；实际 5 mm 到达要求、朝向、静止、碰撞判据不变。默认 30 mm 容差仍按原来的 20 mm 进入边界工作。

导航修复依据：`live_feedback_04` 的 600 步保存观测重算原控制器，指令误差为 0，发现位置修正与朝向对准切换 55 次。该轮最后位置误差仅 4.60 mm，但朝向误差约 72.01°。修复前测试失败、修复后通过，相关导航测试 9 项通过。

BT 的 9 个模块文件哈希全部一致。每次正式仿真前后对 1826 个源码／模型／配置文件做完整性检查，验证期间未发生源文件变更。

## 回归与所有试验

针对性反馈测试 45 项通过。导航修复后的全量回归 **334 项：331 通过、3 跳过、0 失败**，耗时 641.481 s。跳过项分别需要独立 torch 环境、CUDA 环境、依赖完整的 real A 场景；本次不将这些项或 cuTAMP 声明为已验证。

| 试验 | 结果 | 说明 |
|---|---|---|
| live_feedback_01 | 主动结束（不计成功） | Fix independently reproduced stale feedback after verified prerequisite progress |
| live_feedback_02 | 夹具未触发，验证不确定 | 普通抓取探针全部通过几何检查 |
| live_feedback_03 | semantic_replan_exhausted | 技能执行 0 次；结构重规划 1 次 |
| live_feedback_04 | semantic_replan_exhausted | 技能执行 2 次；结构重规划 1 次 |
| live_feedback_05 | recovery_preconditions_failed | 技能执行 3 次；结构重规划 1 次 |
| live_feedback_06 | 主动结束（不计成功） | live_feedback_07 passed same-episode physical feedback-loop acceptance; stop additional validation |
| live_feedback_07 | 成功 | 技能执行 3 次；结构重规划 1 次 |

第 04 轮恢复导航未完成；第 05 轮已完成重新导航、清除旧反馈和新状态抓取求解，但实际抬升峰值仅 7.9715 cm、保持 0 s，因此未计成功。没有通过降低任务判据或跳过安全检查使测试通过。本报告证明本次受控失败能恢复成功，不代表所有失败或所有随机采样都能成功。

## 停车采样和导航检查说明

停车参考域是当前物体位置与机器人朝向下的 25 个点：距离 `0.57/0.62/0.67/0.72/0.77 m`，横向偏移 `−0.18/−0.09/0/0.09/0.18 m`。每次随机选两个同朝向参考点并随机插值。16 或 64 是一次 CCSP 完整参数采样预算，找到第一组可行解即停止；数字位置由采样器生成。有限预算耗尽不证明全域不可达，插值也可能让样本集中在区域中间。

导航几何检查基于当前 planning query 的独立快照，转换底盘／导航坐标系后，采样直线通道及起始、前进／倒车、最终朝向范围：平移步长不超过 1 cm、至少 31 个位置，角度步长不超过 2°。检查各机器人部件与环境和自碰撞；机器人—环境要求 5 mm 间距，自碰撞不能穿透；遵循模型碰撞过滤，排除地面接触及明确记录的固定颈部 CAD 重叠。

每个停车候选还必须存在完整 PickLift 几何见证；到达后再次实测求解。这些检查不提供连续碰撞数学证明、完整绕障搜索、导航动力学收敛或抓取力闭合保证。

## 重规划与执行预算

| 项目 | 仓库默认配置 | 本轮验证配置 |
|---|---:|---:|
| 语义重规划 `semantic_replans` | 2 | 0 |
| 技能程序重规划 `skill_replans` | 2 | 1 |
| 几何重试 `geometry_retries` | 3 | 1 |
| 技能执行重试 `skill_retries` | 1 | 1 |
| 累计技能执行 `max_skill_executions` | 20 | 6 |
| 单次技能执行步数 `max_skill_steps` | 600 | 600 |

重试／重规划预算不包含首次尝试。已验证的前置技能成功后，连续技能规划失败计数会清零，因此 `skill_replans=1` 不是整局累计只能重规划一次；累计技能执行及规划循环仍有上限。安全前置条件不成立时可提前终止。几何采样预算耗尽会进入程序重规划，不必先消耗执行失败后的几何重试。

本轮每次 CCSP 求解最多采样 64 组完整参数。模型 `max_attempts=3` 是每次模型生成的尝试预算，不等同机器人任务重规划次数。本轮实际消耗：技能重规划 1 次、几何重试 0 次、技能执行重试 0 次、技能执行 3 次。“持续到成功”通过保留各次结果、重复运行有界试验实现，没有将单轮预算改为无限。

## 明确保留的后续缺口

当前 PickLift 重启会进入 `pregrasp` 并张爪，且要求 `gripper_empty`；`hold` 阶段重复最后固定关节目标。物体已经夹住、但抬升／保持不足时，现有恢复门控会停止，不能直接重启整个技能。本次未实现阶段内保持夹持并继续规划，也未放宽这些安全门控。此前两次真实保持失败后模型调用均为 0，详见单独审计。

## 交付与复查

成功回放已在浏览器加载验证：无 JavaScript 错误，62 条动画轨道，录制时长 27.390625 s；最终帧已实际渲染并检查。回放时间与环境 27.40 s 的微小差异来自录制时间量化。

- [成功仿真回放](../runs/tamp-feedback-validation-20260922/feedback_simulation.html)
- [最终画面](../runs/tamp-feedback-validation-20260922/feedback_final.png)
- [物体高度、保持时间和相对位置曲线](../runs/tamp-feedback-validation-20260922/feedback_physics.png)
- [本轮完整自动审计](../runs/tamp-feedback-validation-20260922/feedback_audit.json)
- [先前 PickLift 保持失败审计](../runs/tamp-feedback-validation-20260922/picklift_runtime_recovery_audit.json)
- [导航修复后的全量回归日志](../runs/tamp-feedback-validation-20260922/regression-navigation-fix.log)

原始模型输入输出、每步仿真记录、失败约束、参数消费、完整回放、源码哈希保存在 `runs/tamp-feedback-validation-20260922/live_feedback_07`。脚本和全部试验根目录为 `runs/tamp-feedback-validation-20260922/`；文档另存 `docs/PROC3S_FEEDBACK_VALIDATION.md`。

复查全量测试：

```bash
PYTHONPATH=. .venv/bin/python -m unittest discover -s tests -v
```

重新运行同类真实试验（需要开发机既有模型服务环境；输出目录必须是新的）：

```bash
PYTHONPATH=. .venv/bin/python runs/tamp-feedback-validation-20260922/run_feedback_probe.py \
  --repository-root /root/workspace/scenesmith-simple-planner-eval \
  --scene-root /root/scenesmith-simple-planner-eval/scene/scene_000 \
  --experiment experiments/navigation_picklift_tamp.json \
  --output-root runs/tamp-feedback-validation-20260922/new_feedback_trial \
  --max-samples 64 --repeats 0 --shift-world-x-m 0.10
```

模型响应和采样未固定种子，重复运行不保证相同结果。
