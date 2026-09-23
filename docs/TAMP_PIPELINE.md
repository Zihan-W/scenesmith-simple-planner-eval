# SceneSmith legacy TAMP pipeline（消融与历史记录）

本文以下内容描述旧版整计划生成/回放路径，不是新的在线主路径。
通用入口是 `planner/src/tamp/cli.py`：默认使用
`--planner tamp --tamp-mode hierarchical --geometry-backend sampling`。
分层路径无需手工底盘候选，并在每个技能之后重新观测、校验和求解，见
[分层 TAMP](TAMP_HIERARCHICAL.md)。`scenesmith-plan-tamp` 保留为 legacy 入口；
原有 recorded proposal/program 与模型生成数值采样域用于消融，不应与主路径混称。

2026-09-19 更新：两条路径的在线模型配置均读取
`experiments/tamp_hierarchical_config.json`（可用 `--config` 覆盖）。
旧路径分别使用 `subgoal_model` 与 `skill_model`；`--model` 只作显式调试覆盖。
所有旧路径模型输出也进行有界传输/JSON/schema 重试，程序必须保持已选技能和对象；
重试耗尽写为规划失败，不执行无效程序。版本化提示位于 `prompts/tamp_legacy_*.txt`。
开发机可通过 `/root/.config/scenesmith/activate-tamp.sh` 加载用户授权的 API 配置，
真实在线调用已在 `runs/tamp-hierarchical-20260919/live_*` 验证；
这不能代替抓取物理成功证据。下文 2026-09-17/18 记录中的“尚未验证”仅指当时状态。

当前架构参考 [VLM-TAMP](https://zt-yang.github.io/vlm-tamp-robot/) 的视觉子目标与失败后重规划，以及 [PRoC3S](https://github.com/Learning-and-Intelligent-Systems/proc3s) 的连续参数采样、约束验证与反馈。这里实现的是 SceneSmith 适配，不宣称复现两篇论文的完整算法或实验结果。

`scenesmith-plan-tamp` 是唯一入口：从 `scene-eval` 实验配置 reset 场景，保存第一视角头部/左腕图片和物体状态；VLM 仅提出声明式 skill 子目标；约束求解器采样底盘停靠位，利用与仿真相同的 Drake 场景检查整机导航走廊、预抓取和抓取 IK。候选不可行时把精确约束失败原因反馈给 VLM。只有全部检查通过才编译 BT JSON、MDSL、Mermaid 和离线 HTML；`--execute` 才会启动实际仿真，并以任务 evaluator 的结果判定成功。

当前 skill 域限于 `NavigateToPick(target)` 和 `PickLift(target)`，底盘候选由 `--base-candidate x,y,yaw_rad` 给定。VLM 的 `scenesmith.tamp.goals.v2` 只写状态子目标；符号规划器根据前置条件/效果生成技能序列。第二次模型请求生成 `scenesmith.tamp.program.v1`，其中共享变量带离散、均匀或截断正态采样域，技能参数引用变量、场景状态或常量。每组参数先经整条技能链的几何检查；使用 `--execute` 时还会用 `scene-eval` 完整运行候选，物理失败反馈给下一轮提议。没有 API 时，可同时给 `--recorded-proposal` 和 `--recorded-program` 做确定性回放。新 skill 应扩展 `tamp_scenesmith.py` 的采样、检查、状态预测，以及 `tamp_pipeline.py` 的符号算子和 BT 绑定，不需要复制一份生成或执行脚本。

对于 `PickLift`，约束层现在检查预抓取与抓取的关节边，以及沿 8 cm 抬升高度的 8 个 IK 路点、关节连续性和限位裕度。执行层通过 `tamp_execution.py` 将这些已检查的路点绑定成一个技能状态机，仍使用公开的关节与夹爪命令；双指接触、抬升和保持继续由 SceneSmith 的实际物理任务判定。静态路点检查不能替代接触后的动态可执行性验证。

下面的旧命令属于 `legacy-vlm-domain` 消融路线；模型仍可生成数值采样域。离线诊断（录制提议仅用于确定性测试，不代表 VLM 调用）：

```bash
scenesmith-plan-tamp \
  --experiment experiments/navigation_picklift_attempt.json \
  --repository-root "$PWD" --scene-root "$SCENE_ROOT" \
  --output-root runs/tamp-navigation-picklift-001 \
  --goal "导航到红色物体并抓起保持" \
  --base-candidate 2.76,2.95,3.141592653589793 \
  --base-candidate 2.78,2.95,3.141592653589793 \
  --recorded-proposal experiments/inputs/tamp/recorded_navigation_picklift_goals.json \
  --recorded-program experiments/inputs/tamp/recorded_navigation_picklift_program.json
```

实时 VLM 调用时去掉两个录制参数，在 shell 中设置 `OPENAI_BASE_URL`、`OPENAI_API_KEY`，可用 `--model` 指定模型。开发机当前未设置这两个环境变量，因此录制提议的离线回放不代表在线 VLM 验证。输出 `world_snapshot.json`、`tamp_plan.json`；可行时额外生成 `bt/`，其中 `runtime_bindings.json` 记录 BT 之外的连续技能参数。加 `--execute` 时将每个候选的 `bt.html`、可独立运行的 `experiment.json`、物理仿真和任务判定保存到 `rollouts/`。每次选一个新的 `--output-root`，避免覆盖实验记录。

与原论文相比，仍缺少长时域多物体任务、PLACE/重新摆放等技能、执行中部分成功后的在线状态重建与重规划，以及跨多个 SceneSmith 场景/随机种子的成功率统计。当前 `--execute` 会从相同 reset 状态评测每个候选，失败重规划发生在仿真规划阶段。只有扩展这些能力并完成真实模型和多场景评测后，才能称为两篇工作的完整 SceneSmith 复现。

已知物理问题：先前底盘从 `(3.25,2.95)` 到 `(2.78,2.95)` 的导航成功，但原抓取标定在此处于对齐阶段超时；头部俯仰 0.5 rad 还会造成颈部零件初始碰撞导致关节命令被拒绝。TAMP 几何检查必须在与执行相同的头部姿态、目标位置和机器人模型下进行。当前的几何检查不能把旧的抓取标定自动变成新的动态抓取技能。

2026-09-17 物理回放记录：`runs/tamp-navigation-picklift-20260917/attempt_004` 对目标向机器人移动 12 cm 的场景验证了符号目标、共享参数程序、导航和抓取 IK，但直接把夹爪从 76.3 mm 命令到 0 mm 导致整个关节边被碰撞检查拒绝；510 步后双指接触超时。`attempt_005` 在 BT 的 PickLift 接口中使用 2 mm 递进闭合，夹爪到达 49.3 mm，随后左指距离目标表面约 0.05 mm，下一次 2 mm 步进超出 0.1 mm 的许可接触深度，因此再次被拒绝；510 步后没有形成双指接触、抬升或保持。`attempt_006` 使用失败反馈自动缩小闭合步长，被拒命令降至 4 次，左指接触成立，但右指与目标仍相距约 6.55 mm，任务失败。`attempt_007` 把抓取横向偏移 −3.2 mm 纳入参数程序，抓取 IK 可行；右指间隙减至约 0.45 mm，仍无稳定双指接触。2026-09-18 的 `attempt_008` 继续微调横向偏移，并在实验配置中将允许的夹爪-目标规划接触深度设为 0.5 mm；命令均被接受，夹爪收至 42.43 mm，但只有左指接触，未抬升。`attempt_009` 在单指接触后维持有限夹持力，20 步形成双指接触、目标抬高 5.6 mm；验证抬升到第 100 步仍与桌面接触，随后触发 `support_breakaway_timeout`。碰撞检查显示后续笛卡尔命令因肘关节越过 1.5708 rad 上限被拒。规划器因此新增 8 cm 抬升终点 IK 约束，`attempt_010_geometry` 的 x=2.77 m 底盘候选通过几何检查。`attempt_010` 的物理回放仍在接近阶段 301 步后超时；默认导航到达容差 3 cm 使实际停靠位偏离几何检查位约 1.9 cm。`attempt_011` 将导航到达容差设为 5 mm，实际底盘到达 x≈2.775 m，接近阶段通过，但右指接触、左指仍差约 0.99 mm，180 步后抓取超时。`attempt_012` 把抓取横向偏移作为与底盘停靠位联合采样的参数，x=2.77 m、偏移 −2.93 mm 通过几何检查；物理回放形成双指接触、目标抬高约 5.6 mm，但仍与桌面接触。验证抬升时左肘接近 1.5708 rad 上限，后续关节边被拒，100 步后触发 `support_breakaway_timeout`。下一步需要带关节裕度的接触后轨迹，而不是仅检查抬升终点 IK。每次物理结果都以各自 `summary.json`、`trace.csv` 和 `tamp_plan.json` 为准；几何可行不等于物理抓取成功。上述提议为固定录制输入，尚未验证在线 VLM 调用。
