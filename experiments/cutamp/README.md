# cuTAMP 的可复现输入

`config.json` 是正式配置。只需提供两个与机器有关的环境变量：

```bash
export CUTAMP_PYTHON=/your/gpu/venv/bin/python
export CUTAMP_ROOT=/your/cuTAMP-checkout
```

其余模板、成本权重、容差和标定路径都相对于该配置解析，不依赖 runs。

## 源码与环境

- cuTAMP：NVlabs/cuTAMP，提交 `7932e6cf0ee216331e37e06b60f18bb8b3ec1fbd`。
- cuRobo：NVlabs/curobo，提交 `d64c4b005459db10c5dd867d8b30a87d5bda9bdb`（0.7.8）。
- 精确归档哈希见 `upstream-archives.json`；提交号从已验证源码归档的 PAX 元数据提取。
- 对 cuTAMP 应用 `upstream-local.patch`，包括已有非实验 self-collision kernel 的显式开启、物体球确定性采样和 warp 依赖约束；已对原始源码执行补丁重放验证。
- Python/计算依赖版本见 `integrity.json` 和 `gpu-requirements.lock.txt`。先按上游说明准备对应 CUDA 编译工具链，再以此锁文件作为 pip constraints 安装锁定提交的本地 cuRobo/cuTAMP；这两个项目需要从上述源码安装，不能以同版本号的任意包替代。
- 当前实际验证使用 Python 3.11、torch 2.4.1+cu121、cuRobo 0.7.8、RTX 4090。没有声称其他机器已经实测。

## 执行前检查

CPU 适配层验证模板/权重/容差/抓取标定的 SHA256、cuTAMP Python 源码文件集合和内容，以及 GPU Python/计算依赖版本。不匹配会在启动优化前抛出错误。运行中继续检查机器人 URDF 和导出的 proximity mesh 哈希。

`robot.json` 与 `kinematics.json` 使用可移植 URDF 标识；实际问题构建时，先检查模型哈希，再绑定当前环境提供的模型路径。worker 从安装的 planner 包加载，不要求场景资产仓库带有 scripts 或 planner 源码。

该配置没有把近似碰撞模型标记为全面验证；现有 Drake 后验和物理成功判据继续有效。热启动和基于失败的连续约束学习仍未实现，能力契约明确声明当前反馈用于诊断。

后验按 `postcheck_order` 排列候选，但最终在**已完整通过精确检查**的候选中按
`rank_candidate` 选择：先比较左右手指间隙差，再比较提升关节余量和抓取横向偏移。
正式配置不设置 `postcheck_quality_window`，仍使用 24 次／180 秒上限内的全部检查结果。
独立实验配置可设置非负整数 `postcheck_quality_window`：首个候选通过后再检查指定数量的候选，
仍从这个窗口里按原评分选取。设为 `0` 时等于按检查顺序选择首个通过者，
`candidate_selection.json` 的停止原因会写为 `first_pass_found`；正数窗口用
`quality_window_complete`。这会改变可选集合及最终抓取参数，不应当当成纯计时优化。
`stop_reason` 只表示主停止原因；`candidate_selection.json` 另有独立的
`count_exhausted`，即使质量窗口与次数上限在同一次检查达到，也不会丢失次数证据。
间隙差是静态排序量；当前没有可把某个间隙差数值直接判定为双侧动态接触成功或失败的阈值。

目标物体的球近似由上游在 worker 运行时采样。worker 将已有的整数 seed 传给
`TAMPWorld`，后者按 `SHA256("worker_seed:object_name")` 前 64 位为每个物体派生
表面采样 seed；球数量、半径及贪心选择算法不变。每次实际使用的球坐标、球集 SHA-256
以及拟合源码 SHA-256 记录在 `optimizer/problem.json`，球集 SHA-256 也记录在
`optimizer/result.json`。同输入、同 seed 在锁定的软件环境中应生成相同球集；
该记录用于核对运行，不是精确碰撞证明。
球集哈希不同的运行不能作为 GPU 近似通过率或候选排序的配对实验，即使输入文件哈希
和 worker seed 相同。确定性只解决采样复现，不证明拟合覆盖或在线可靠性；
精确后验继续保留。
