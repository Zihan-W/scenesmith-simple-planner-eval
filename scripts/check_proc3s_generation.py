"""Live PRoC3S program-generation acceptance; no simulation success claim.

Run from the repository with the authorized model environment already loaded.
Saves model requests, responses and the still-open program without credentials.
"""

import argparse
import dataclasses
import json
import os
from pathlib import Path

from planner.src.bt.generation import OpenAICompatibleChatClient
from planner.src.tamp.hierarchy import PredicateGoal, WorldState, picklift_registry
from planner.src.tamp.online import JsonlTrace
from planner.src.tamp.proc3s import PRoC3SProgramGenerator
from planner.src.tamp.semantic import ModelSettings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=Path("experiments/tamp_hierarchical_config.json"))
    args = parser.parse_args()
    args.output_root.mkdir(parents=True, exist_ok=False)
    config = json.loads(args.config.read_text())
    client = OpenAICompatibleChatClient(base_url=os.environ["OPENAI_BASE_URL"],
                                        api_key=os.environ["OPENAI_API_KEY"])
    trace = JsonlTrace(args.output_root / "model_trace.jsonl")
    world = WorldState({"red_cube": {"category": "cube", "movable": True}}, frozenset({
        PredicateGoal("observed", ("red_cube",)), PredicateGoal("gripper_empty", ())}))
    generator = PRoC3SProgramGenerator(client, ModelSettings(**config["skill_model"]),
                                      picklift_registry(), trace=trace)
    program = generator.generate(world, (PredicateGoal("holding", ("red_cube",)),))
    result = {"acceptance": "live_program_generation", "success": True,
              "simulation_executed": False, "llm_calls": generator.calls,
              "model_settings": config["skill_model"], "program": dataclasses.asdict(program)}
    (args.output_root / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"success": True, "llm_calls": generator.calls,
                      "output_root": str(args.output_root)}))


if __name__ == "__main__":
    main()
