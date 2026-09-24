# 通用 BT 生成 Pipeline

生成统一由 `planner/src/bt/generation.py` 和命令
`scenesmith-generate-bt` 负责。每次运行只换请求 JSON；不会为新任务复制一份生成程序。
已有 PickLift 命令保留旧请求兼容；导航脚本仅转发新 `--request` 接口，
原 `--environment`/`--task-plan` 参数需迁移为请求 JSON。导航、抓取的
**执行器**仍各自管理物理动作。

请求 schema 为 `scenesmith.bt_generation.request.v1`：

```json
{
  "schema": "scenesmith.bt_generation.request.v1",
  "request_id": "example-run-001",
  "environment": {"path": "environment.json", "sha256": "<64位十六进制哈希>"},
  "task": {
    "description": "任务自然语言描述",
    "plan_path": "task_plan.json",
    "plan_sha256": "<64位十六进制哈希>"
  },
  "observations": [],
  "model": {"id": "gpt-4.1-mini", "max_attempts": 3}
}
```

`environment`、`task_plan`、相机图片均相对于请求目录，且必须提供真实 SHA-256。
PickLift 使用有序的头部与左腕 PNG，并核对 reset metadata 的图片哈希和目标可见性；
当前导航任务使用空 `observations`。任务计划 schema 选用对应的严格编译器，
响应只能包含 `MAIN_SEQUENCE` 和 `ULTIMATE_GOAL`，MDSL 必须逐步匹配任务计划。

```bash
scenesmith-generate-bt --request /path/to/request.json \
  --output-dir /path/to/new-run/bt \
  --base-url "$OPENAI_BASE_URL"
```

模型 API 在运行生成命令的开发机上通过环境变量配置：

```bash
export OPENAI_BASE_URL="https://你的兼容接口地址/v1"
export OPENAI_API_KEY="你的密钥"
```

模型名称由请求 JSON 的 `model.id` 指定；也可用 `--base-url` 覆盖地址，
或用 `--api-key-env 其他变量名` 指定密钥变量。不要把密钥写入请求 JSON。
直接执行已有 BT JSON 的 `scene-eval` 不调用模型 API。

使用 `--model-response recorded_response.json` 可离线重放录制的模型响应。
输出统一为 `generated_plan.json`、`generated_bt.json`、`generated_bt.mdsl`、
`generated_bt.mmd`、`generated_bt.html`。输出目录已有任一同名文件时会拒绝覆盖。
生成只验证 BT 结构和技能参数，不能代表仿真执行成功。

新增 skill 时，在 `planner/src/bt/core.py` 的 `SKILLS` 登记类型、参数和提示，并加入相应的
`PROFILE_SKILLS`，
并在对应任务适配器中实现动作与计划参数校验。MDSL 解析、节点结构、
sequence/selector tick、模型调用、重试、哈希绑定和产物写入均复用共享代码。
新任务类型如需不同环境 schema，在 `planner/src/bt/generation.py` 的 `PROFILES` 加入适配关系。
