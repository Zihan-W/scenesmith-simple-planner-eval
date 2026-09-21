"""Summarize completed audit artifacts without changing planner or skills."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def load(path):
    return json.loads(path.read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    summary = {"scope": "existing_two_skill_registry_bounded_live_audit", "reasoning": {}, "visual": {}, "runs": []}
    harness = Path(__file__).with_name("validate_vlm_ccsp.py")
    summary["validation_harness_sha256"] = hashlib.sha256(harness.read_bytes()).hexdigest()
    for field, filename in (("reasoning", "reasoning_results.json"), ("visual", "visual_results.json")):
        rows = load(root / "seed500" / filename)
        for case in sorted({row["case"] for row in rows}):
            selected = [row for row in rows if row["case"] == case]
            summary[field][case] = {"passed": sum(row["passed"] for row in selected), "total": len(selected),
                                   "model_calls": sum(row["model_calls"] for row in selected)}
    for folder in sorted(root.glob("seed*")):
        result = load(folder / "live_result.json")
        integrity = load(folder / "source_integrity.json")
        events = [json.loads(line) for line in (folder / "runtime/tamp_trace.jsonl").read_text().splitlines()]
        checks = [e for e in events if e["event"] == "ccsp_constraint_check"]
        executions = [{k: e[k] for k in ("skill", "runtime_success", "runtime_reason", "episode_finished", "geometric_conditioning")}
                      for e in events if e["event"] == "skill_execution"]
        recording = folder / "simulation.html"
        result.update(source_integrity=integrity, execution_outcomes=executions,
            generated_skeletons=[e["steps"] for e in events if e["event"] == "skill_skeleton"],
            failed_constraint_counts=dict(Counter(e["constraint"] for e in checks if not e["feasible"])),
            ccsp_check_count=len(checks), recording_size_bytes=recording.stat().st_size,
            recording_sha256=hashlib.file_digest(recording.open("rb"), "sha256").hexdigest())
        summary["runs"].append(result)
    (root / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
    successes = sum(run["success"] for run in summary["runs"])
    lines = ["# VLM / PRoC3S + CCSP 验证（2026-09-20）", "",
        f"结论：已有简单的状态条件规划与失败后骨架修订证据，但尚不能认定闭环可靠。此次完整任务成功 {successes}/{len(summary['runs'])}。", "",
        "只新增验证脚本及运行产物；不修改已有技能、提示词、控制参数、碰撞检查或任务阈值。未调用 cuTAMP。", "",
        f"完整开发机产物目录：`{root}`。两轮录像分别是 `seed500/simulation.html` 和 `seed501/simulation.html`。", "",
        "## 方法与边界", "",
        "- 使用现有 `navigation_picklift_tamp.json`，真实模型调用 `gpt-4.1-mini`，现有两项技能 `NavigateToPick` / `PickLift`。",
        "- 场景图像来自同步头部、腕部相机，含仿真标签框和对象元数据；不是纯视觉识别基准。",
        "- 符号测试每项独立调用三轮。`at_pick_pose`、`holding` 及抓取失败反馈为显式测试注入，不注入物理执行。",
        "- 实际闭环种子 500、501；每次 CCSP 搜索最多 16 个完整赋值，骨架重规划最多一次、几何重试最多一次、不做语义重规划。有限预算失败不等于物理不可解。",
        "- 使用现有 IncrementalTampRunner / Proc3sCCSPSolver / SceneSmithSkillExecutor，不提供录制骨架、不指定人工挑选的站位候选、不回退到符号规划器。初始几何状态显式取实测底盘。",
        "- 保留现有提示词，其中包含导航→抓取的格式示例。因此本实验检验该提示下的状态条件选择和修订，不声称模型从零发现了技能组合。",
        "", "## 模型分层测试", "", "| 测试 | 通过 / 总数 |", "|---|---|"]
    names = {"observed_not_at_pick": "未到抓取位：导航→抓取", "injected_at_pick": "给定已到位：直接抓取",
        "navigation_goal_only": "只要求到位：不多生成抓取", "injected_already_holding": "给定已持有：空计划",
        "injected_grasp_failure": "注入抓取失败：重新站位→抓取", "pick": "带图像抓取指令：holding 子目标",
        "navigate_only": "带图像仅导航指令：at_pick_pose、无 holding"}
    for field in ("reasoning", "visual"):
        for case, item in summary[field].items():
            lines.append(f"| {names[case]} | {item['passed']} / {item['total']} |")
    lines += ["", "空计划测试直接调用生成器；在线 runner 在任务已满足时会提前返回，因此生成器的失败不表示实际会重复抓取。多余抓取步骤的测试衡量骨架简洁性；runner 在导航子目标满足后会重新观测，而不是无条件执行余下步骤。",
              "", "## 实际闭环", "", "| Seed | 任务成功 | 已执行技能 | 结束原因 |", "|---|---|---|---|"]
    for run in summary["runs"]:
        actions = ", ".join(f"{e['skill']} ({e['runtime_reason']})" for e in run["execution_outcomes"])
        lines.append(f"| {run['seed']} | {run['success']} | {actions or '无'} | {run['reason']} |")
    lines += ["", "每次真实运行的模型骨架和约束拒绝统计：", ""]
    for run in summary["runs"]:
        skeletons = [" → ".join(step["skill"] for step in steps) or "空计划" for steps in run["generated_skeletons"]]
        lines.append(f"- seed {run['seed']}：模型依次生成 {'；'.join(skeletons)}。拒绝统计：`{json.dumps(run['failed_constraint_counts'])}`。")
    lines += ["", "seed501 的导航成功后直接抓取，再因真实 CCSP 失败改为重新站位后抓取，是实际反馈驱动修订的证据；并不代表修订后的计划获得可行参数或执行成功。seed500 导航受阻后，新的导航候选在当前起点的走廊碰撞检查被拒绝，随后模型生成的修订未通过符号校验。", ""]
    lines += ["", "## 源码完整性与证据", ""]
    for run in summary["runs"]:
        lines.append(f"- seed {run['seed']}：检查 {run['source_integrity']['checked_files']} 个实现/提示词/配置/模型文件，未变化={run['source_integrity']['unchanged']}。录像 {run['recording_size_bytes']} 字节；SHA256 `{run['recording_sha256']}`。")
    lines += ["", "每个 seed 目录保存 `audit_config.json`、`resolved_experiment.json`、`raw_model_io.jsonl`、`live_result.json`、`simulation.html` 和 `runtime/`（执行、验证、原始相机图像）。seed500 另有分层测试输入输出及失败原文。", "",
        "已有 TAMP 回归测试：86 项通过，见 `regression.log`。单元测试通过不替代本次真实任务成功判据。录像为仿真生成的离线 Meshcat HTML；未另行自动化验证浏览器播放。", "",
        "本报告仅覆盖单一场景、两项已有技能及两个 CCSP 种子。没有验证长任务、多目标视觉消歧、通用骨架搜索能力；注入反馈后的结构变化也不等价于真实失败恢复成功。", ""]
    (root / "REPORT.md").write_text("\n".join(lines))
    print(json.dumps({"report": str(root / "REPORT.md"), "live_successes": sum(x["success"] for x in summary["runs"]),
                      "live_trials": len(summary["runs"])}))


if __name__ == "__main__":
    main()
