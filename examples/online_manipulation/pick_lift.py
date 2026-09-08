"""复现固定 Zerith PickLift：只使用公共 API，不复制环境或抓取策略。

前提：安装本仓库、生成 Zerith OBJ，并取得已验收的 SceneSmith 场景。
这不是任意场景通用抓取；experiments/picklift.json 引用版本化专家输入，
包含已经标定的初态、目标绑定与策略参数，不需要历史 output 文件。
"""

import argparse
from dataclasses import replace
import json
from pathlib import Path

from src.online_manipulation import load_experiment


def main() -> int:
    """准备只读场景的派生缓存，执行一次抓取，并保存可检查的结果。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repository-root", type=Path, required=True,
        help="eval checkout 根目录，包含 experiments/ 和准备好的机器人模型。",
    )
    parser.add_argument(
        "--scene-root", type=Path, required=True,
        help="原始 scene_000 目录，包含 package.xml 和 combined_house/。",
    )
    parser.add_argument(
        "--run-dir", type=Path, required=True,
        help="新的空目录；cache/ 存派生资产，output/ 存实验结果。",
    )
    parser.add_argument(
        "--meshcat", action="store_true",
        help="开启浏览器可视化，同时保存离线 simulation.html（文件较大）。",
    )
    args = parser.parse_args()
    root = args.repository_root.resolve()
    run_dir = args.run_dir.resolve()
    if run_dir.exists() and any(run_dir.iterdir()):
        parser.error("--run-dir 必须是新目录或空目录，避免覆盖之前的结果。")

    # 1. 短配置选择机器人、控制、初态、场景、Task、Policy 和 evaluator。
    #    loader 校验输入并生成 cache；不会覆盖 SceneSmith 原始场景。
    experiment = load_experiment(
        root / "experiments/picklift.json",
        repository_root=root,
        scene_root=args.scene_root.resolve(),
        cache_root=run_dir / "cache",
        meshcat=args.meshcat,
    )

    # 2. 环境配置与策略是两个独立对象；策略实现不定义在环境里。
    #    保留已验收 seed=500 和最多 1200 步，不在示例里重写抓取参数。
    experiment = replace(experiment, run_options={
        **experiment.run_options,
        "record_html": args.meshcat,
        "write_final_dmd": True,
    })
    print(f"Policy: {type(experiment.policy).__name__}", flush=True)
    print(f"Run options: {dict(experiment.run_options)}", flush=True)
    print(f"Results: {run_dir / 'output'}", flush=True)

    # 3. 复用唯一 Runner，而不是实现第二套控制/记录循环。Runner 内部：
    #    env = make_env(experiment.environment_config)
    #    obs, info = env.reset(seed=500); policy.reset(obs, info)
    #    每个策略周期执行 env.step(policy.act(obs))，直到结束或超时。
    #    本配置一步推进 0.1 秒，期间由低层控制器和接触动力学执行动作。
    results = experiment.run(run_dir / "output")

    # 4. 成功由 Task/evaluator 判定，不以“脚本没有异常”冒充抓取成功。
    for result in results:
        print(json.dumps({key: result.summary[key] for key in (
            "seed", "success", "termination_reason",
            "episode_time_s", "policy_steps",
        )}, ensure_ascii=False))
    success = all(result.success for result in results)
    print("PickLift SUCCESS" if success else "PickLift FAILED：请检查 summary.json 和 trace.csv。")
    print(f"Artifacts: {run_dir / 'output'}")
    print("final.dmd.yaml 可能引用 cache/，使用最终场景前请保留该缓存。")
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
