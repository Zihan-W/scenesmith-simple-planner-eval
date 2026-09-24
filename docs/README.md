# 文档索引

本目录只保留当前使用说明、架构和接口契约。历史实验报告、审计流水账及截图已清理，
需要回溯时查看 Git 历史；小型验收索引仍保留在 [validation](../validation/)。

| 内容 | 入口 |
|---|---|
| 安装、场景与仿真运行 | [Quickstart](QUICKSTART_ONLINE_ENV.md) |
| 当前 BT/TAMP 架构、后端选择及能力边界 | [当前架构](CURRENT_ARCHITECTURE.md) |
| TAMP 自动模式与规划/执行分离 | [TAMP 模块接口](TAMP_MODULE.md) |
| 通用 BT 生成 | [BT 生成流程](BT_GENERATION_PIPELINE.md) |
| PickLift BT 请求、图像与语义约束 | [PickLift BT 模块](PICKLIFT_BT_GENERATION_MODULE.md) |
| 仿真动作、配置、观测与扩展契约 | [公共接口](GENERIC_ONLINE_EXAMPLE.md) |
| GPU 精确后验停止与预算 | [cuTAMP 预算契约](CUTAMP_BUDGET_CONTRACT.md) |
| 模型记录/回放、恢复和证据口径 | [TAMP 回放与恢复](TAMP_REPRODUCIBILITY.md) |
| 相机安装与仿真假设 | [Zerith 相机清单](ZERITH_CAMERA_INVENTORY.md) |
| 机器可读 BT 契约 | [请求 schema](contracts/picklift_bt_generation_request.schema.json)、[结果 schema](contracts/picklift_bt_generation_result.schema.json) |

文档和实验仍采用显式白名单；新增文件时更新根目录 `.gitignore` 末尾的白名单，
并执行 `git check-ignore -v --no-index 路径`，避免文档被静默遗漏。
不要将密钥、运行日志、录像、模型权重或实验大归档放入文档目录。
