# 当前架构

项目提供 BT 与 TAMP 两条规划入口，使用同一套技能驱动和 SceneSmith 仿真执行接口。
安装和完整运行命令见 [Quickstart](QUICKSTART_ONLINE_ENV.md) 与[项目 README](../README.md)。

## 规划层

| 路径或层级 | 输入与输出 | 选择与入口 |
|---|---|---|
| BT 生成 | 任务请求、观测及模型响应 → 可检查的 BT 产物 | `planner.src.bt.generation`；生成后显式执行 |
| TAMP 语义层 | 任务、标注图像及已观测事实 → JSON 谓词子目标 | `SemanticSubgoalPlanner` |
| TAMP 技能程序 | 子目标、技能前提和效果 → 带开放连续参数的技能程序 | PRoC3S 默认；STRIPS 可选 |
| TAMP 连续求解 | 当前物理快照和固定技能程序 → 参数化技能计划 | PRoC3S CCSP 默认；cuTAMP 可选 |
| 执行与验证 | 一个已检查技能 → 实测观测与任务结果 | 共享技能驱动，执行后验证并按预算恢复或重规划 |

独立 `sampling` 后端已经删除。PRoC3S CCSP 自身的整程序赋值采样仍是其算法的一部分。
选择 cuTAMP 替换的是连续求解器；不会先运行 PRoC3S CCSP 再串联 GPU 优化。

```text
任务 + 当前观测
  ├─ BT 生成 → 显式执行 BT
  └─ TAMP 语义子目标
       → PRoC3S / STRIPS 技能程序
       → PRoC3S CCSP / cuTAMP 连续参数
       → 执行一个技能 → 实测验证 → 下一步规划或有界恢复
                     ↓
           共享 SkillDriver / 仿真 Runtime
```

## 模块与执行契约

- TAMP 公共模块为 `planner.src.tamp`：`run(TampRunRequest(...))` 提供带进程监督的自动闭环，
  `create_session()` 提供同一进程内的 `plan()` / `execute()` 分离接口。
- `plan()` 不推进仿真；交接授权仅覆盖下一个技能。状态变化、计划过期、外来或重复句柄均拒绝执行。
  保存的计划 JSON 是审计记录，不能跨会话直接执行。完整接口见 [TAMP 模块](TAMP_MODULE.md)。
- BT 生成与执行分离，模型输出须通过任务编译和技能参数检查；见 [BT 生成](BT_GENERATION_PIPELINE.md)。
- 当前生产 TAMP 域为 `NavigateToPick` / `PickLift`。扩展技能还需实现执行绑定、领域约束，
  使用 cuTAMP 时还需相应算子/代价表示；模块化不表示任意机器人和任务都已支持。

## 仿真与几何检查

`simulation` 管理 RobotAdapter、Scenario、Task、Policy、观测和统一 Runtime。
规划查询从当前状态建立独立上下文；求解器不修改正在执行的仿真状态。
BT 与 TAMP 通过同一技能驱动和公开动作接口执行，实际任务成功由实测接触、抬升和保持判据决定。
详细动作、坐标、相机和状态契约见 [公共接口](GENERIC_ONLINE_EXAMPLE.md)。

cuTAMP 的 GPU 球近似用于候选优化，精确后验仍保留。默认首个完整通过精确检查的候选即停止；
候选质量窗口和自适应预算可选。见 [预算契约](CUTAMP_BUDGET_CONTRACT.md)。
单指释放/重抓尚未支持；有界抓持补偿默认关闭，其触发条件与终止边界见
[模型回放与恢复契约](TAMP_REPRODUCIBILITY.md)。

## 论文参考与实现边界

- [VLM-TAMP](https://zt-yang.github.io/vlm-tamp-robot/)：参考视觉语义子目标、物理规划细化和失败反馈。
  当前直接生成受限 JSON 谓词，未接入官方 PDDLStream/Fast Downward，也未实现规划对象缩减与碰撞驱动扩展。
- [PRoC3S](https://github.com/Learning-and-Intelligent-Systems/proc3s)：适配带开放参数的程序生成、
  整程序赋值拒绝采样及约束反馈；本地使用受限 JSON DSL 和 SceneSmith 几何检查。
- [cuTAMP](https://github.com/NVlabs/cuTAMP)：接入固定技能骨架上的 GPU 连续优化，
  并增加机器人适配、安装完整性校验和精确后验。

这些是当前接通的机制，不等同于完整复现各论文的任务集或可靠性结果。
GPU 环境从零重建入口见[项目 README](../README.md)；源码补丁、上游归档和 integrity 清单必须一致。

## 文档与证据

`docs/` 保留当前指南与接口契约；历史实验报告和截图已清理，可从 Git 历史查看。
`validation/` 保留小型验收索引；大型运行、录像和归档放在忽略的运行目录中。
本次 planner 发布标签为 `planner-v0.1`，与仿真公共 API/共享发行包的版本独立。
