"""Synchronize behavior-tree diagnostics with a saved Meshcat recording."""

from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any


SCHEMA = "scenesmith.behavior_tree_timeline.v1"
MARKER = "scenesmith-bt-overlay"


def _tree_nodes(tree: Mapping[str, Any]) -> list[dict[str, Any]]:
    nodes: list[dict[str, Any]] = []

    def visit(node: Mapping[str, Any], path: str, depth: int) -> None:
        if not isinstance(node, Mapping):
            raise TypeError("BT visualization tree nodes must be mappings")
        kind = node.get("kind")
        name = node.get("name") or kind
        children = node.get("children", ())
        args = node.get("args", ())
        if not isinstance(kind, str) or not kind or not isinstance(name, str):
            raise ValueError("BT visualization nodes require string kind/name")
        if not isinstance(children, Sequence) or isinstance(children, (str, bytes)):
            raise TypeError("BT visualization children must be a sequence")
        if not isinstance(args, Sequence) or isinstance(args, (str, bytes)):
            raise TypeError("BT visualization args must be a sequence")
        label = name
        if args:
            label += "(" + ", ".join(str(value) for value in args) + ")"
        nodes.append({"path": path, "kind": kind, "label": label, "depth": depth})
        for index, child in enumerate(children):
            visit(child, f"{path}.{index}", depth + 1)

    visit(tree, "0", 0)
    return nodes


def _diagnostics(row: Mapping[str, Any]) -> Mapping[str, Any]:
    value = row.get("policy_diagnostics_json", {})
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except json.JSONDecodeError:
            return {}
    return value if isinstance(value, Mapping) else {}


def build_behavior_tree_timeline(
    *,
    definition: Mapping[str, Any],
    trace: Sequence[Mapping[str, Any]],
    recording_start_time_s: float,
) -> dict[str, Any] | None:
    """Build compact time-indexed BT states from policy-period trace rows."""
    if not isinstance(definition, Mapping) or not isinstance(definition.get("tree"), Mapping):
        raise TypeError("BT visualization definition requires a tree mapping")
    nodes = _tree_nodes(definition["tree"])
    by_path = {node["path"]: node for node in nodes}
    samples = []
    previous = None
    for row in trace:
        diagnostics = _diagnostics(row)
        if diagnostics.get("controller") != "behavior_tree":
            continue
        active_path = diagnostics.get("active_path")
        if not isinstance(active_path, str) or not active_path:
            continue
        node = by_path.get(active_path, {})
        expert = diagnostics.get("expert")
        stage = expert.get("stage") if isinstance(expert, Mapping) else None
        sample = {
            "time_s": max(0.0, float(row["simulation_time_s"]) - float(recording_start_time_s)),
            "active_path": active_path,
            "active_label": node.get("label", active_path),
            "active_kind": node.get("kind", "unknown"),
            "tree_status": str(diagnostics.get("tree_status", "UNKNOWN")),
            "reason": str(diagnostics.get("reason", "")),
            "expert_stage": stage,
            "task_reason": str(row.get("task_reason", "running")),
        }
        state = tuple((key, value) for key, value in sample.items() if key != "time_s")
        if state != previous:
            samples.append(sample)
            previous = state
    if not samples:
        return None
    duration = max(
        0.0,
        float(trace[-1]["simulation_time_s"]) - float(recording_start_time_s),
    )
    return {
        "schema": SCHEMA,
        "title": str(definition.get("title", "Behavior Tree")),
        "duration_s": duration,
        "nodes": nodes,
        "samples": samples,
    }


def policy_behavior_tree_timeline(
    *,
    policy: Any,
    trace: Sequence[Mapping[str, Any]],
    recording_start_time_s: float,
) -> dict[str, Any] | None:
    """Read an optional policy definition and build its recording timeline."""
    provider = getattr(policy, "behavior_tree_visualization", None)
    if not callable(provider):
        return None
    definition = provider()
    return build_behavior_tree_timeline(
        definition=definition,
        trace=trace,
        recording_start_time_s=recording_start_time_s,
    )


def _overlay_html(payload: Mapping[str, Any]) -> str:
    data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    data = data.replace("<", "\\u003c").replace(">", "\\u003e")
    return f"""
<!-- {MARKER} -->
<style id="{MARKER}-style">
#{MARKER}{{position:fixed;left:14px;top:14px;width:360px;max-height:calc(100vh - 28px);z-index:2147483646;background:rgba(9,19,33,.93);color:#eef6ff;border:1px solid #60758f;border-radius:12px;box-shadow:0 8px 30px #0008;font:13px/1.35 system-ui,sans-serif;overflow:auto}}
#{MARKER}.collapsed .btov-body{{display:none}} #{MARKER} .btov-head{{position:sticky;top:0;background:#13253b;padding:10px 12px;border-radius:11px 11px 0 0;display:flex;align-items:center;gap:8px}}
#{MARKER} .btov-title{{font-weight:750;flex:1}} #{MARKER} button{{color:#dcecff;background:#29425f;border:1px solid #6e86a0;border-radius:6px;cursor:pointer}}
#{MARKER} .btov-summary{{padding:9px 12px;border-bottom:1px solid #334963;display:grid;grid-template-columns:auto 1fr;gap:3px 8px}}
#{MARKER} .btov-key{{color:#9fb2c9}} #{MARKER} .btov-status{{font-weight:750}} #{MARKER} .btov-tree{{padding:8px}}
#{MARKER} .btov-node{{margin:3px 0;padding:5px 7px;border:1px solid #344b65;border-radius:7px;background:#16283d;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;transition:.12s}}
#{MARKER} .btov-node.ancestor{{border-color:#4b91d1;background:#173652}} #{MARKER} .btov-node.active{{border:2px solid #ffd43b;background:#594916;color:#fff5bf;box-shadow:0 0 0 2px #ffd43b33}}
#{MARKER} .btov-kind{{display:inline-block;min-width:66px;color:#9ec5ef;font-size:11px;text-transform:uppercase}} #{MARKER} .btov-path{{float:right;color:#7890aa;font-size:10px}}
</style>
<aside id="{MARKER}" aria-live="polite">
  <div class="btov-head"><span class="btov-title"></span><button type="button" title="收起或展开">−</button></div>
  <div class="btov-body"><div class="btov-summary">
    <span class="btov-key">时间</span><span class="btov-time"></span>
    <span class="btov-key">当前节点</span><span class="btov-current"></span>
    <span class="btov-key">BT 状态</span><span class="btov-status"></span>
    <span class="btov-key">控制阶段</span><span class="btov-stage"></span>
  </div><div class="btov-tree"></div></div>
</aside>
<script id="{MARKER}-script">
(()=>{{
  const data={data}; window.__SCENESMITH_BT_TIMELINE__=data;
  const panel=document.getElementById('{MARKER}');
  panel.querySelector('.btov-title').textContent=data.title;
  panel.querySelector('button').onclick=()=>{{panel.classList.toggle('collapsed');panel.querySelector('button').textContent=panel.classList.contains('collapsed')?'+':'−';}};
  const tree=panel.querySelector('.btov-tree'), rows=new Map();
  for(const node of data.nodes){{const row=document.createElement('div');row.className='btov-node';row.style.marginLeft=`${{node.depth*14}}px`;const kind=document.createElement('span');kind.className='btov-kind';kind.textContent=node.kind;const label=document.createElement('span');label.textContent=node.label;const path=document.createElement('span');path.className='btov-path';path.textContent=node.path;row.append(kind,label,path);tree.append(row);rows.set(node.path,row);}}
  let last=-1;
  function sampleAt(time){{let lo=0,hi=data.samples.length;while(lo<hi){{const mid=(lo+hi)>>1;if(data.samples[mid].time_s<=time+1e-6)lo=mid+1;else hi=mid;}}const key=Math.max(0,lo-1);return [key,data.samples[key]];}}
  function render(time){{if(!Number.isFinite(time))time=0;const [key,sample]=sampleAt(time);panel.querySelector('.btov-time').textContent=`${{time.toFixed(2)}} / ${{data.duration_s.toFixed(2)}} s`;
    if(key!==last){{last=key;panel.querySelector('.btov-current').textContent=sample.active_label;panel.querySelector('.btov-status').textContent=sample.tree_status;panel.querySelector('.btov-stage').textContent=sample.expert_stage||sample.task_reason||'—';for(const [path,row] of rows){{row.classList.toggle('active',path===sample.active_path);row.classList.toggle('ancestor',sample.active_path.startsWith(path+'.'));}}}}
  }}
  function attach(){{try{{if(typeof viewer!=='undefined'&&viewer.animator){{const animator=viewer.animator;if(!animator.__scenesmithBtOverlayHooked){{const seek=animator.seek.bind(animator),display=animator.display_progress.bind(animator);animator.seek=function(time){{const result=seek(time);render(time);return result;}};animator.display_progress=function(time){{const result=display(time);render(time);return result;}};animator.__scenesmithBtOverlayHooked=true;}}render(animator.time);return;}}}}catch(_error){{}}requestAnimationFrame(attach);}} attach();
}})();
</script>
"""


def inject_behavior_tree_overlay(
    recording_path: Path,
    payload: Mapping[str, Any],
) -> bool:
    """Append a valid in-page overlay before the recording's closing body tag."""
    path = Path(recording_path)
    injection = _overlay_html(payload).lstrip().encode("utf-8")
    marker = f"<!-- {MARKER} -->".encode()
    with path.open("r+b") as stream:
        stream.seek(0, 2)
        size = stream.tell()
        tail_size = min(size, 1024 * 1024)
        stream.seek(size - tail_size)
        tail = stream.read()
        body_offset = tail.rfind(b"</body>")
        if body_offset < 0:
            raise ValueError("Meshcat recording does not contain a closing body tag")
        marker_offset = tail.find(marker)
        if 0 <= marker_offset < body_offset:
            if tail[marker_offset:body_offset] == injection:
                return False
            insertion_offset = marker_offset
        else:
            insertion_offset = body_offset
        insertion_at = size - tail_size + insertion_offset
        remainder = tail[body_offset:]
        stream.seek(insertion_at)
        stream.truncate()
        stream.write(injection)
        stream.write(remainder)
    return True


def write_behavior_tree_timeline(path: Path, payload: Mapping[str, Any]) -> None:
    Path(path).write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
