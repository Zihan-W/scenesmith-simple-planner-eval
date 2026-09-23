# TAMP 实验与恢复契约

层级 TAMP 的在线模型调用默认写入 `model_transcript.jsonl`。它覆盖语义 VLM、PRoC3S、无效响应纠错和提供方错误重试，保留原始响应正文、模型与 usage。凭据不写入。该能力解决完整模型调用序列的复用；此前 `--recorded-subgoals` 只替换语义层，PRoC3S 仍可能在线调用。

使用相同 CLI 参数，并增加 `--model-replay /absolute/path/model_transcript.jsonl`，可在无 API 凭据时严格离线回放。模型、temperature、response_format、完整消息和图像内容必须匹配；图像文件路径可不同，但内容 SHA256 必须相同。重复请求按原顺序返回各自记录的响应，不把不同采样合并。缺失、额外或不匹配请求使实验失败并记录 `model_replay_divergence`，绝不退回在线调用。任务提前结束允许仅消费匹配前缀，并在 evidence 中记录未使用响应数与 matched_request_prefix；不能为了消费失败轮的恢复响应而继续执行已完成的任务。完整序列重复性检查可额外调用 assert_consumed。记录的提供方耗时不重放；模型等待时间不得作为回放性能收益。传输 token 上限和 endpoint 记录在文件头；离线回放不使用网络。

语义提示中的已观测事实按谓词契约声明顺序、谓词名和参数排序，PRoC3S 采样器名排序，两种请求的结构化 JSON 键均排序，消除集合跨进程散列顺序对实际模型输入的影响；严格回放不会忽略请求文本差异。

`model_evidence.json` 区分逻辑调用数、真实提供方调用数、原始 transcript 哈希和分叉。`result.json` 不再将“已录语义 + 在线 PRoC3S”标成全部 recorded。脚本 `scripts.validate_cutamp_online` 支持 `--replay-from PREVIOUS_ACCEPTANCE --repeats N`，每个 seed 使用其完整 transcript；也支持 `--model-replay FILE`，让所有 seed 共用一份固定响应序列。后者在请求能够逐项匹配时隔离模型重新采样；发生观测/分支变化仍明确失败。失败和回放分叉均保留在总试次数中。

## 统计口径

- live_record 是模型采样、规划与执行的联合结果，不能称为纯物理可靠性。
- strict_replay 是以固定模型响应为条件的规划/执行结果，按场景 seed、GPU seed、transcript SHA 分组。重复相同 seed 用于可复现性检查，不是独立可靠性样本。
- 改变规划策略后仍必须逐请求匹配；分叉是对照不可比的证据，不能用旧响应强行覆盖新观测。
- 聚合脚本仅为非重复在线样本计算小样本 Wilson 描述区间。固定回放或重复 seed/响应存在相关性，不输出合并 Wilson 区间，不能解释为总体可靠性保证。方差分解需要预先设计模型响应 × 执行扰动的交叉实验；本工具不虚构未实施的扰动分布。

官方 PRoC3S 同样提供 LLM 缓存入口 `+policy.use_cache=true`，见 [官方 README](https://github.com/Learning-and-Intelligent-Systems/proc3s)。本项目采用严格序列回放，以便显式暴露闭环观测分叉。

## 后验停止

正式 CuTAMPSettings 默认 `postcheck_quality_window=0`：首个候选的完整技能计划通过精确检查后即停止。没有完整通过时继续检查预算内候选。显式 `null` 恢复旧全预算评分行为，正整数表示通过后额外候选窗口。候选排序、精确几何检查和动态任务成功判定不变。球集仍是近似：浅穿透/保守误拒的边界不因确定性或停止策略而消失。

## 可选抓持补偿

层级 planner config 可显式加入：

```json
"grasp_compensation": {
  "max_attempts": 3,
  "max_step_m": 0.01,
  "max_total_m": 0.03,
  "height_margin_m": 0.006
}
```

缺省关闭。上述值是有界实验配置，不是标定结论。单位为世界坐标米，方向仅世界 +Z；次数最多 3，每次请求上升 6–10 mm，累计请求上升最多 30 mm（并非物体实际位移保证）。高度余量显式限定为 6–10 mm，仍是待标定的实验值。补偿 IK 使用 0.5 mm / 0.5° 容差，并核对实际几何上升不小于请求减位置容差；不能沿用名义抬升的 3 mm 粗容差，否则小幅修正会被 IK 容差与负载跟踪偏差吞掉。

触发条件是原计划进入 hold、实测抬升不足任务门槛、双指接触稳定，且关节跟踪/速度通过既有接触稳定判据。每次从同一时间戳的实测机器人、物体与 TCP 状态建立新的携带物体关系；保持当前 TCP 朝向，做 IK、关节裕度/连续性检查以及携带物体的精确 measured/commanded 起点边检查。执行仍通过原有逐步精确安全检查与关节速度限制。每次计划的位姿、时间戳、检查和拒绝原因进入 skill trace。

不在规划耗时内推进模拟时钟；使用现有整轮 perf_counter 截止与进程组 watchdog，不新增独立放宽预算。执行失去双指接触、出现异常接触、几何拒绝、超时或次数/行程耗尽时停止。不打开手指、不重抓、不移动底盘；单指释放恢复尚无控制契约。物理成功只能由实际高度和保持时间确定，不能由补偿 IK 成功代替。

## 证据与 ref 留存

`refs/validation/current` 保留当前已验收快照。`scripts.archive_validation_refs --output ARCHIVE --keep 2 --prune` 保留 current 与最近两个命名 ref，其余写成独立 Git bundle。工具在全新临时裸库逐 ref 恢复并核对 OID 后，才用 compare-and-delete 事务删除旧 ref。未加 `--prune` 只归档。恢复例：`git fetch ARCHIVE/validation.bundle refs/validation/NAME:refs/validation/NAME`。

`validation/` 存小型已发布证据索引：源码哈希、归档哈希、位置与恢复说明。大运行证据不进入 Git；归档应复制到至少两个存储位置并逐份核对 SHA256。索引不属于执行源码 manifest，以免发布结果反过来改变被测源码摘要；执行代码、测试、配置和文档仍在源码 manifest 中。

跨机验收必须在另一台有可用 CUDA 工具链的主机完成全新构建并运行安装校验/GPU 测试。本机补丁重放或容器重建不能替代该证据。

## 受限 gen_domain 位置反馈

cuTAMP 精确后验、PRoC3S CCSP 和普通采样器在真实几何失败时，通过 SceneSmith 域提取已检查控制参数的归一化位置。位置坐标沿用已注册 FULL envelope；不是当前缩小子域，也不是世界坐标。NavigateToPick 提供 forward/lateral，PickLift 提供 grasp lateral 投影。后者不包含手臂关节配置，不能推断相同 lateral 的其他抓姿失败。

ProgramFailure 最多携带八个不同的 sampled_failure_positions；每个包括原 program_step、已声明 variable/sampler、位置与白名单失败类别。接收端验证数值范围、物体绑定和与当前 world 一致的物理快照 token；裁掉导航前缀的当前位置抓取检查，会把 solver step 0 映回模型程序 step 1，无法唯一映射则丢弃。物理状态已改变的历史位置不进入新请求。语义 VLM 不接收这些数值。模型可据此提出注册范围内的 subdomain 修订；精确检查和子域约束继续生效。

不会从超时、未检查候选、排除赋值、约束共享错误或纯近似优化器违规生成“精确失败位置”。域外候选不钳制成虚假的边界样本。失败投影不是完整赋值，更不是区域不可行或 UNSAT 证明。

## 可复用验收入口

`validate_cutamp_online` 的 `--experiment PATH --task TEXT --shift-world-x-m VALUE` 可覆盖现有抓取实验默认值。换 seed、对象、起始扰动或任务阈值时复用任务/实验配置与脚本，无需复制验收实现。仍须由已注册的 TAMP 域支持实验的任务与技能；当前生产域是 NavigateToPick/PickLift，并非任意自然语言任务的自动判定器。新的任务类别应实现领域适配、成功谓词及技能安全检查，通用记录/回放、预算监督和证据统计层继续复用。
