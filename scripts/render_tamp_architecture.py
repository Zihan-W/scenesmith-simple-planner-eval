"""Render the hierarchical TAMP architecture figures as PNG.

The labels are Chinese, so this needs a CJK font. Pass --font or place a
Noto Sans CJK / Source Han Sans file where the script can find it.

    python scripts/render_tamp_architecture.py --output-dir docs/assets
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")

from matplotlib import font_manager, pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


FONT_CANDIDATES = (
    "/tmp/NotoSansCJKsc-Regular.otf",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJKsc-Regular.otf",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
)

STAGE_FILL = "#e8f1fb"
INPUT_FILL = "#eef7ee"
OUTPUT_FILL = "#fdf3e3"
REGISTRY_FILL = "#f2e9f8"
EDGE = "#5b6b7c"


def resolve_font(explicit: str | None):
    """Return a font path that can render Chinese labels."""
    candidates = [explicit] if explicit else []
    candidates += list(FONT_CANDIDATES)
    for path in candidates:
        if path and Path(path).exists():
            name = font_manager.FontProperties(fname=path).get_name()
            if "CJK" in name or "Han" in name or "WenQuan" in name:
                return path
    raise SystemExit(
        "No CJK font found. Pass --font /path/to/NotoSansCJKsc-Regular.otf"
    )


def box(ax, x0, y0, x1, y1, text, *, fill, font, size=8.0, weight="normal",
        align="left", color="#1d2733"):
    ax.add_patch(FancyBboxPatch(
        (x0, y0), x1 - x0, y1 - y0,
        boxstyle="round,pad=0.004,rounding_size=0.012",
        linewidth=1.0, edgecolor=EDGE, facecolor=fill, zorder=1,
    ))
    if align == "center":
        ax.text((x0 + x1) / 2, (y0 + y1) / 2, text, ha="center", va="center",
                fontproperties=font, fontsize=size, weight=weight, color=color,
                zorder=2, linespacing=1.45)
    else:
        ax.text(x0 + 0.012, y1 - 0.012, text, ha="left", va="top",
                fontproperties=font, fontsize=size, weight=weight, color=color,
                zorder=2, linespacing=1.45)


def arrow(ax, start, end, *, color=EDGE, style="-|>", width=1.2, radius=0.0,
          linestyle="-"):
    ax.add_patch(FancyArrowPatch(
        start, end, arrowstyle=style, mutation_scale=11, linewidth=width,
        color=color, zorder=3, linestyle=linestyle,
        connectionstyle=f"arc3,rad={radius}",
    ))


def figure_flow(font, out_path: Path):
    fig, ax = plt.subplots(figsize=(16.5, 10.2), dpi=150)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.text(0.30, 0.985, "SceneSmith 分层 TAMP 架构（hierarchical 主路径）",
            ha="center", va="top", fontproperties=font, fontsize=16, weight="bold",
            color="#16222e")
    ax.text(0.870, 0.972, "运行产物", ha="center", va="top", fontproperties=font,
            fontsize=10.5, weight="bold", color="#8a6d3b")

    # ---------------- inputs ----------------
    box(ax, 0.010, 0.895, 0.170, 0.955, "输入：CLI\n--experiment / --scene-root / --task\n--seed / --config / --output-root",
        fill=INPUT_FILL, font=font, size=7.6)
    box(ax, 0.010, 0.760, 0.170, 0.875, "输入：实验配置\nexperiments/*.json\n场景只读加载 + scene_cache\n(scene_manifest / scene_metadata)",
        fill=INPUT_FILL, font=font, size=7.6)
    box(ax, 0.010, 0.625, 0.170, 0.740, "输入：模型凭据\nOPENAI_BASE_URL\nOPENAI_API_KEY\n(或 --recorded-subgoals 录制回放)",
        fill=INPUT_FILL, font=font, size=7.6)
    box(ax, 0.010, 0.490, 0.170, 0.605, "输入：任务与种子\ntask 自然语言\nseed=500 默认\nskills: picklift_registry()",
        fill=INPUT_FILL, font=font, size=7.6)
    box(ax, 0.010, 0.355, 0.170, 0.470, "输入：模型设置\ntamp_hierarchical_config.json\nsubgoal_model / skill_model\nrecovery / proc3s_ccsp 预算",
        fill=INPUT_FILL, font=font, size=7.6)

    # ---------------- stages ----------------
    sx0, sx1 = 0.215, 0.630
    stages = [
        (0.845, 0.955, "① 观测层  SceneSmithWorldObserver",
         "in : env.reset(seed) → Observation\nout: WorldState(objects, facts) + 头/腕 RGB(bbox 标注)"),
        (0.700, 0.810, "② 语义层  SemanticSubgoalPlanner  [VLM, goals.v3]",
         "in : task, WorldState, predicates/arity, 图像, 抽象反馈\nout: PredicateGoal 元组（只含谓词+对象名）"),
        (0.555, 0.665, "③ 骨架层  SkillProgramGenerator：strips | proc3s",
         "in : WorldState, 目标, SkillRegistry(模型视图 6 字段)\nout: SkillProgram（技能名 + 对象 + 开放变量名）"),
        (0.410, 0.520, "④ 几何层  GeometrySolver：proc3s(CCSP) | cutamp",
         "in : SkillProgram, geometry_state, 失败排除集, SceneSmithPickDomain 检查\nout: ParameterizedSkillPlan（连续赋值 + 约束结果）"),
        (0.265, 0.375, "⑤ 执行层  SceneSmithSkillExecutor",
         "in : ParameterizedSkillAction（单技能）\nout: SkillExecution（成功/原因/耗时/新观测）"),
        (0.120, 0.230, "⑥ 校验  verify_expected_effects → 新 WorldState",
         "in : expected_effects(action) + observe() 后的事实\nout: 通过 / 失败（LowLevel→Program→Semantic）"),
    ]
    for y0, y1, title, body in stages:
        box(ax, sx0, y0, sx1, y1, title + "\n" + body, fill=STAGE_FILL, font=font,
            size=8.2)
    for i in range(len(stages) - 1):
        arrow(ax, (0.4225, stages[i][0] - 0.001), (0.4225, stages[i + 1][1] + 0.001))

    # registry feeding skeleton + geometry + execution
    box(ax, 0.215, 0.020, 0.520, 0.096,
        "SkillRegistry  单一事实来源：picklift_registry()\n③④⑤ 三层共享同一份注册表；五个消费点见图 2",
        fill=REGISTRY_FILL, font=font, size=8.0)

    # inputs -> stages
    arrow(ax, (0.172, 0.925), (0.213, 0.925))
    arrow(ax, (0.172, 0.818), (0.213, 0.885))
    arrow(ax, (0.172, 0.682), (0.213, 0.760))
    arrow(ax, (0.172, 0.548), (0.213, 0.615))
    arrow(ax, (0.172, 0.412), (0.213, 0.470))

    # ---------------- feedback loop ----------------
    arrow(ax, (0.632, 0.175), (0.667, 0.175), color="#b03a2e")
    arrow(ax, (0.667, 0.175), (0.667, 0.610), color="#b03a2e", radius=0.0)
    arrow(ax, (0.667, 0.610), (0.632, 0.610), color="#b03a2e")
    arrow(ax, (0.667, 0.610), (0.667, 0.760), color="#b03a2e")
    arrow(ax, (0.667, 0.760), (0.632, 0.760), color="#b03a2e")
    ax.text(0.686, 0.395, "四级恢复预算  skill_retry → geometry_retry → skill_replan → semantic_replan",
            ha="center", va="center", rotation=90, fontproperties=font, fontsize=7.2,
            color="#b03a2e")

    # ---------------- outputs ----------------
    ox0, ox1 = 0.745, 0.995
    outputs = [
        (0.870, 0.955, "planner_config.json\nresolved_experiment.json\n（planner/seed/backend 标签）"),
        (0.745, 0.855, "tamp_trace.jsonl\n模型请求/响应、注册表快照、\n候选参数、决策、最终结果"),
        (0.620, 0.730, "skill_steps.jsonl\n每个物理步的 policy 诊断、\n动作决策、机器人/物体状态"),
        (0.495, 0.605, "observations/NNN/\n头/腕 RGB + 标注 + manifest\n（时间戳与哈希）"),
        (0.370, 0.480, "skills/NNN/\nskill_bt.json（单技能 BT）\nresult.json\nparameter_consumption.json"),
        (0.245, 0.355, "result.json\nsuccess / reason / metrics\nfinal_facts"),
        (0.120, 0.230, "simulation.html\n（--record-html 时的 Meshcat 回放）"),
    ]
    for y0, y1, body in outputs:
        box(ax, ox0, y0, ox1, y1, body, fill=OUTPUT_FILL, font=font, size=7.6)

    arrow(ax, (0.712, 0.905), (0.743, 0.905))
    arrow(ax, (0.632, 0.800), (0.743, 0.800), color="#8a6d3b")
    arrow(ax, (0.712, 0.290), (0.743, 0.290), color="#8a6d3b")
    arrow(ax, (0.632, 0.425), (0.743, 0.425), color="#8a6d3b")
    arrow(ax, (0.632, 0.550), (0.743, 0.550), color="#8a6d3b")

    fig.savefig(out_path, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def figure_skills(font, out_path: Path):
    fig, ax = plt.subplots(figsize=(14.5, 8.6), dpi=150)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.text(0.5, 0.975, "Skill 注册表（SkillRegistry / SkillSpec）用在哪些地方",
            ha="center", va="top", fontproperties=font, fontsize=16, weight="bold",
            color="#16222e")

    box(ax, 0.320, 0.430, 0.680, 0.640,
        "SkillRegistry  单一事实来源\n"
        "picklift_registry() → SkillSpec 9 个字段\n"
        "name / symbolic_parameters / geometric_parameters\n"
        "preconditions / add_effects / delete_effects\n"
        "constraints / supports_geometric_conditioning / runtime_action",
        fill=REGISTRY_FILL, font=font, size=9.0, weight="normal")

    consumers = [
        (0.020, 0.760, 0.300, 0.930, "① 符号搜索  refine_goals\n读 preconditions / add / delete\n输出 SkillProgram（技能+对象+开放变量）\n不在骨架里搜索物理参数"),
        (0.700, 0.760, 0.980, 0.930, "② 模型输入  proc3s payload\n白名单 6 字段（去掉 constraints /\nruntime_action / conditioning）\n模型只看到符号契约"),
        (0.020, 0.470, 0.300, 0.690, "③ 几何求解\ngeometric_parameters → 开放变量角色\nProc3sCCSPSolver / CuTAMPSolver\n均以注册表校验 program 不被改写"),
        (0.700, 0.470, 0.980, 0.690, "④ 执行绑定  SceneSmithSkillExecutor\nruntime_action → NavigateTo / ExecutePickLift\n并校验名字同时存在于 bt_core.SKILLS"),
        (0.020, 0.170, 0.300, 0.390, "⑤ 记录与产物\nasdict(spec) 全字段写 tamp_trace.jsonl\nskill_registry 事件\n= 事后审计用的快照"),
        (0.700, 0.170, 0.980, 0.390, "注意 constraints 字段\n当前没有任何代码读取\ncorridor / ik / joint_edge 等标签\n只是描述，检查逻辑按技能名硬编码"),
    ]
    for x0, y0, x1, y1, text in consumers:
        fill = "#fdecea" if text.startswith("注意") else STAGE_FILL
        box(ax, x0, y0, x1, y1, text, fill=fill, font=font, size=8.4)

    arrow(ax, (0.500, 0.640), (0.160, 0.755), radius=0.12)
    arrow(ax, (0.500, 0.640), (0.840, 0.755), radius=-0.12)
    arrow(ax, (0.380, 0.430), (0.200, 0.410), radius=0.10)
    arrow(ax, (0.620, 0.430), (0.800, 0.410), radius=-0.10)
    arrow(ax, (0.318, 0.520), (0.302, 0.560))
    arrow(ax, (0.682, 0.520), (0.698, 0.560))

    ax.text(0.5, 0.075, "SkillSpec 的构造期不变量：名字与两套参数名不得重复；"
                        "preconditions/add/delete 中的 $占位符必须属于 symbolic_parameters",
            ha="center", va="center", fontproperties=font, fontsize=8.6, color="#41505f")

    fig.savefig(out_path, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("docs/assets"))
    parser.add_argument("--font", default=None, help="CJK font file")
    args = parser.parse_args()
    font_path = resolve_font(args.font)
    font = font_manager.FontProperties(fname=font_path)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    flow = args.output_dir / "tamp_architecture_flow.png"
    skills = args.output_dir / "tamp_architecture_skills.png"
    figure_flow(font, flow)
    figure_skills(font, skills)
    print(f"font: {font_path}")
    print(f"wrote {flow}\nwrote {skills}")


if __name__ == "__main__":
    main()
