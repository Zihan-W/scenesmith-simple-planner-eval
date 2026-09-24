# TAMP 模块接口

TAMP 与 BT 同级安装，入口是 `planner.src.tamp`。模块保留 PRoC3S / cuTAMP 选择，
提供独立规划、显式执行及自动闭环三种调用动作；复用现有 runner，未新增另一套规划算法。

## 包与自动闭环

```bash
python -m planner.src.tamp --help
```

参数沿用原 `planner.src.tamp.cli`。省略选择时，默认 PRoC3S 程序生成 + PRoC3S CCSP。
cuTAMP 必须显式选择；STRIPS 可通过 `skill_planner="strips"` 选择，并搭配两个保留的几何后端。
独立 `sampling` 后端及其配置已删除；旧的 `geometry_backend="sampling"` 会明确报错。
PRoC3S CCSP 内部的采样算法仍保留。
安装到当前 Python 环境后，可以在仓库外导入和运行；场景、实验配置和 GPU 环境仍须提供。

```python
from pathlib import Path
from planner.src.tamp import TampRunRequest, run

repo = Path('/path/to/scenesmith-simple-planner-eval')
request = TampRunRequest(
    repository_root=repo,
    scene_root=Path('/path/to/scene_000'),
    experiment=repo / 'experiments/navigation_picklift_tamp.json',
    config=repo / 'experiments/tamp_current_validation.json',
    task='Pick up the red object and hold it.',
    output_root=Path('/path/to/new-output'),
    skill_planner='proc3s', geometry_backend='proc3s',
    seed=500, shift_world_x_m=0.10, record_html=True,
)
result = run(request)
print(result['success'], result['reason'])
```

选择 cuTAMP 时设置 `geometry_backend='cutamp'`，并传入
`cutamp_config=repo / 'experiments/cutamp/config.json'`。
API 使用既有 GPU 安装校验；`CUTAMP_ROOT` / `CUTAMP_PYTHON` 仍由调用环境提供。
`run()` 返回持久化 `result.json` 的完整字典，保留选择质量、配置、模型来源和失败原因。
任务失败返回 `success=False`；调用侧参数/配置错误及非空输出目录报错。
它沿用原 CLI 的进程组 watchdog。`model_replay` 可传完整模型记录，严格核对请求。

## 规划与执行分离

调用方持有已经初始化的环境、实验配置和模型客户端。`create_session()` 不 reset 或 step
环境，不调用模型；它准备执行绑定及当前图像。`session.plan()` 才进行模型调用和几何求解，
其间不执行技能。返回 `PreparedPlan` 后，调用方显式调用 `session.execute(prepared)`。

```python
import json
import os
from pathlib import Path
from planner.src.tamp import create_session
from planner.src.bt.generation import OpenAICompatibleChatClient
from planner.src.tamp.model_transcript import TranscriptChatClient
from planner.src.tamp.scene_setup import reset_scene
from simulation.src import load_experiment

repo = Path('/path/to/scenesmith-simple-planner-eval')
output = Path('/path/to/new-session')
output.mkdir(parents=True, exist_ok=False)
settings = json.loads((repo / 'experiments/tamp_current_validation.json').read_text())
experiment = load_experiment(
    repo / 'experiments/navigation_picklift_tamp.json',
    repository_root=repo, scene_root=Path('/path/to/scene_000'),
    cache_root=output / 'scene_cache', trust_factories=True,
)
experiment, env, observation, reset_info, _ = reset_scene(
    experiment, seed=500, shift_world_x_m=0.10)
client = TranscriptChatClient(
    output=output / 'model_transcript.jsonl',
    client=OpenAICompatibleChatClient(
        base_url=os.environ['OPENAI_BASE_URL'], api_key=os.environ['OPENAI_API_KEY']),
)

with create_session(
    env=env, experiment=experiment, reset_info=reset_info,
    repository_root=repo, output_root=output, task='Pick up the red object and hold it.',
    settings=settings, client=client, model_called=True,
    skill_planner='proc3s', geometry_backend='proc3s', seed=500,
) as session:
    while (prepared := session.plan()) is not None:
        document = prepared.as_dict()
        print(document['next_action']['skill_name'])
        prepared.write(output / f'prepared_{prepared.sequence:03d}.json')
        outcome = session.execute(prepared)
        print(outcome.success, outcome.reason)
    result = session.result
    print(result.success, result.reason)
```

在相同示例中选择 cuTAMP：

```python
from planner.src.tamp.cutamp import CuTAMPSettings
gpu = CuTAMPSettings.from_file(repo / 'experiments/cutamp/config.json')
# create_session(..., geometry_backend='cutamp', cutamp_settings=gpu)
```

完整模型回放时，将客户端改为 `TranscriptChatClient(output=..., replay=...)`，
并保持 `model_called=False`。调用方负责保存客户端 evidence、环境录像及外部会话结果；
需要自动整理这些标准产物时使用上面的 `run()`。

## 交接契约

- `plan()` 返回当前会话唯一待执行的 `PreparedPlan`，重复调用不会重复求解。
- 结果 JSON 含程序、整段候选计划、`next_action`、状态标识和单调截止时间；
  执行授权只覆盖 `next_action`，不能直接顺序执行整段候选计划。
- `as_dict()` 返回独立副本，修改它不会修改执行参数。`write()` 拒绝覆盖已有文件。
  JSON 是审计产物；不支持重新加载到另一个会话或进程后直接执行。
- `execute()` 只接受本会话尚未消费的原始句柄；外来、复制或重复使用的句柄报错。
- SceneSmith 状态守卫读取当前公开观测。外部 step/reset（即使重置到相同坐标）、
  可变观测内容的修改都会使交接失效。失效抛出 `StalePlanError`，不发出动作；
  应以新观测重新建立会话。禁止通过私有 backend 绕过环境接口修改状态。
- `execute()` 返回既有 `SkillExecution`。下一次 `plan()` 消费它，进行效果验证、
  原有有界恢复或重新求解；返回 None 时查看 `session.result`，失败也有明确结果。
- `session.run()` 自动驱动同一组 plan/execute 操作，可以接续已执行一部分的会话。
- `close()` 和上下文退出取消未执行计划，不发动作。每个会话必须由单一调用方拥有，
  不支持并发调用，也不允许调用方在计划交接期间另行推进同一环境。

分离接口是进程内 API，保留协作式单调截止；调用方等待也占用预算，截止后不执行旧计划。
它不自行建立后台 watchdog，无法强制抢占正在运行的原生求解调用。
自动 `run(TampRunRequest(...))` / 包 CLI 保留外部进程组 watchdog；嵌入分离接口的
应用若需要同样的硬截止，应由宿主进程监督整个会话。没有放宽现有控制或成功阈值。

当前生产域仍为 NavigateToPick / PickLift。模块化不表示任意任务、机器人或场景都已支持。
