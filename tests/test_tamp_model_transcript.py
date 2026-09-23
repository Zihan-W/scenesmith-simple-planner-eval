"""Full-response replay contracts, independent of provider/network availability."""
import dataclasses
import json
from pathlib import Path
import tempfile
import unittest
from planner.src.bt.generation import ChatCompletion, ProviderError
from planner.src.tamp.model_transcript import TranscriptChatClient, ReplayDivergence
from planner.src.tamp.model_budget import model_stage_budget
from planner.src.tamp.semantic import ModelSettings, SemanticSubgoalPlanner


class Transport:
    max_tokens = 2048
    token_parameter = 'max_completion_tokens'
    base_url = 'https://example.invalid/v1'
    timeout_s = 180.
    def __init__(self, outputs): self.outputs = iter(outputs)
    def set_deadline(self, deadline): self.deadline = deadline
    def complete(self, **kwargs):
        output = next(self.outputs)
        if isinstance(output, Exception): raise output
        return ChatCompletion(output, kwargs['model'], 'id', 'stop', {'total_tokens': 7})


class TranscriptTests(unittest.TestCase):
    def test_duplicate_requests_preserve_distinct_responses_and_provider_errors(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)
            request = dict(model='model', messages=[{'role':'user','content':'plan'}], temperature=.2)
            live = TranscriptChatClient(output=path/'live', client=Transport([
                'invalid response', ProviderError('limited', kind='http', retryable=True, status_code=429), 'valid']))
            with model_stage_budget(live, ModelSettings()):
                first = live.complete(**request)
                self.assertEqual(live.client.timeout_s, 30.)
            self.assertEqual(live.timeout_s, 180.)
            with self.assertRaises(ProviderError): live.complete(**request)
            last = live.complete(**request)
            replay = TranscriptChatClient(output=path/'replayed', replay=path/'live')
            self.assertEqual(replay.complete(**request), first)
            self.assertEqual(replay.evidence()['unused_recorded_calls'], 2)
            self.assertEqual(replay.evidence()['replay_scope'], 'matched_request_prefix')
            with self.assertRaises(ProviderError) as error: replay.complete(**request)
            self.assertEqual(error.exception.status_code, 429)
            self.assertEqual(replay.complete(**request), last)
            replay.assert_consumed()
            self.assertEqual(replay.evidence()['live_provider_calls'], 0)
            self.assertEqual((path/'live').read_bytes(), (path/'replayed').read_bytes())
            with self.assertRaises(ReplayDivergence): replay.complete(**request)

    def test_mismatch_unused_and_corruption_fail_without_fallback(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)
            request=dict(model='m', messages=[])
            live=TranscriptChatClient(output=path/'live', client=Transport(['ok']))
            live.complete(**request)
            replay=TranscriptChatClient(output=path/'replay', replay=path/'live')
            with self.assertRaises(ReplayDivergence): replay.assert_consumed()
            with self.assertRaises(ReplayDivergence): replay.complete(**{**request,'temperature':1.})
            rows=(path/'live').read_text().splitlines()
            row=json.loads(rows[1]);row['response']['content']='tampered'
            (path/'corrupt').write_text(rows[0]+'\n'+json.dumps(row)+'\n')
            with self.assertRaises(ValueError):
                TranscriptChatClient(output=path/'bad',replay=path/'corrupt')

    def test_image_bytes_match_across_output_paths_but_changed_pixels_fail(self):
        import hashlib
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)
            for name in ('a.png','b.png'): (path/name).write_bytes(b'image bytes')
            def request(name):
                return dict(model='m',messages=[{'role':'user','content':[{'type':'image_url',
                    'image_url':{'path':str(path/name),'sha256':hashlib.sha256(b'image bytes').hexdigest()}}]}])
            live=TranscriptChatClient(output=path/'live',client=Transport(['ok']))
            expected=live.complete(**request('a.png'))
            replay=TranscriptChatClient(output=path/'replay',replay=path/'live')
            self.assertEqual(replay.complete(**request('b.png')),expected)
            (path/'b.png').write_bytes(b'changed')
            with self.assertRaises(ReplayDivergence): replay.complete(**request('b.png'))

    def test_fact_set_order_does_not_change_the_actual_semantic_request(self):
        response = json.dumps({'schema':'scenesmith.tamp.goals.v3',
            'subgoals':[{'predicate':'holding','arguments':['target']}]})
        facts = [{'predicate':'gripper_empty','arguments':[]},
                 {'predicate':'observed','arguments':['target']}]
        arguments = dict(task='Pick target',
            predicate_arity={'observed':1,'gripper_empty':0,'holding':1})
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)
            live = TranscriptChatClient(output=path/'live',client=Transport([response]))
            first = SemanticSubgoalPlanner(live,ModelSettings()).propose(**arguments,
                world={'objects':{'target':{}},'facts':facts})
            replay = TranscriptChatClient(output=path/'replay',replay=path/'live')
            second = SemanticSubgoalPlanner(replay,ModelSettings()).propose(**arguments,
                world={'objects':{'target':{}},'facts':list(reversed(facts))})
            self.assertEqual(first,second)
            replay.assert_consumed()
            self.assertEqual(replay.evidence()['live_provider_calls'],0)

    def test_semantic_and_program_requests_match_across_process_hash_seeds(self):
        import os
        import subprocess
        import sys
        code = """
import dataclasses, json
from planner.src.tamp.hierarchy import PredicateGoal, WorldState, picklift_registry
from planner.src.tamp.semantic import SemanticSubgoalPlanner, ModelSettings
from planner.src.tamp.proc3s import PRoC3SProgramGenerator
from planner.src.tamp.model_transcript import canonical_request, digest
class Captured(RuntimeError): pass
class Client:
    timeout_s = 30.
    def set_deadline(self, value): pass
    def complete(self, **request):
        print(digest(canonical_request(**request)))
        raise Captured()
world = WorldState({'target': {'movable': True}},frozenset({
    PredicateGoal('observed',('target',)),PredicateGoal('gripper_empty',())}))
registry = picklift_registry()
try:
    SemanticSubgoalPlanner(Client(),ModelSettings()).propose(task='Pick target',
        world={'objects': world.objects,'facts':[dataclasses.asdict(f) for f in world.facts]},
        predicate_arity=registry.predicate_arity)
except Captured: pass
try:
    PRoC3SProgramGenerator(Client(),ModelSettings(),registry).generate(
        world,(PredicateGoal('at_pick_pose',('target',)),))
except Captured: pass
"""
        outputs = [subprocess.check_output([sys.executable,'-c',code],
            env={**os.environ,'PYTHONHASHSEED':str(seed)},text=True)
            for seed in (1,2,3)]
        self.assertEqual(len(outputs[0].splitlines()),2)
        self.assertEqual(len(set(outputs)),1)
