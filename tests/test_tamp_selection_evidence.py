"""Selection reporting preserves sign, policy, and failed-selection semantics."""
import json
from pathlib import Path
import tempfile
import unittest

from planner.src.tamp.selection_evidence import collect_run_selection_evidence, collect_selection_evidence, selection_evidence


class SelectionEvidenceTests(unittest.TestCase):
    def selection(self, **changes):
        return dict(quality_window_after_first_pass=None, adaptive_budget=None,
                    selected_score=[.00764, -.36, .004], selected_particle=57,
                    candidate_order='cost_quantiles', stop_reason='postcheck_wall_time', **changes)

    def test_named_units_and_margin_sign(self):
        evidence = selection_evidence(self.selection())
        self.assertEqual(evidence['selected_score'], [.00764, -.36, .004])
        self.assertEqual(evidence['selected_quality'], dict(gap_imbalance_m=.00764,
                         min_lift_joint_margin_rad=.36, abs_grasp_lateral_offset_m=.004))
        self.assertEqual(evidence['selection_policy'], 'full_budget')

    def test_adaptive_mode_and_no_pass(self):
        record = self.selection()
        record.update(adaptive_budget={'mode': 'first_pass'}, selected_score=None, selected_particle=None)
        evidence = selection_evidence(record)
        self.assertEqual(evidence['selection_policy'], 'adaptive_full_budget')
        self.assertEqual(evidence['adaptive_final_mode'], 'first_pass')
        self.assertIsNone(evidence['selected_quality'])
        self.assertIsNone(evidence['selected_score'])

    def test_collect_all_solves_and_reject_corruption(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            for number in (2, 1):
                folder = run / 'cutamp' / f'solve_{number:03d}'
                folder.mkdir(parents=True)
                (folder / 'candidate_selection.json').write_text(json.dumps(self.selection()))
            evidence = collect_selection_evidence(run)
            self.assertEqual([item['solve'] for item in evidence], ['solve_001', 'solve_002'])
            (folder / 'candidate_selection.json').write_text('{')
            with self.assertRaises(json.JSONDecodeError):
                collect_selection_evidence(run)

    def test_formal_config_declares_first_pass(self):
        repo = Path(__file__).resolve().parents[1]
        config = json.loads((repo / 'experiments/cutamp/config.json').read_text())
        self.assertEqual(config['postcheck_quality_window'], 0)

    def test_resolved_config_propagates_without_default_substitution(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            self.assertFalse(collect_run_selection_evidence(run)['selection_config_recorded'])
            config = {'cutamp': {'postcheck_quality_window': None},
                      'settings': {'grasp_compensation': {'max_attempts': 3}}}
            (run / 'planner_config.json').write_text(json.dumps(config))
            evidence = collect_run_selection_evidence(run)
            self.assertTrue(evidence['selection_config_recorded'])
            self.assertEqual(evidence['cutamp_settings'], config['cutamp'])
            self.assertEqual(evidence['grasp_compensation'], config['settings']['grasp_compensation'])
            self.assertEqual(evidence['cutamp_selections'], [])

    def test_invalid_rank_rejected(self):
        for score in ([0.], [0., float('nan'), 0.]):
            record = self.selection()
            record['selected_score'] = score
            with self.assertRaises(ValueError):
                selection_evidence(record)
