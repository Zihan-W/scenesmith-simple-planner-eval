"""The modular BT package works from outside the checkout after installation."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class BtPackageEntryTest(unittest.TestCase):
    def test_package_entry_generates_valid_artifacts_outside_repository(self):
        inputs = ROOT / 'experiments/inputs/bt_picklift_first_person'
        recorded = json.loads((inputs / 'generated_plan.json').read_text())
        environment = dict(os.environ)
        environment.pop('PYTHONPATH', None)
        environment.pop('OPENAI_API_KEY', None)
        environment.pop('OPENAI_BASE_URL', None)
        with tempfile.TemporaryDirectory() as folder:
            work = Path(folder)
            response = work / 'response.json'
            response.write_text(recorded['raw_response'])
            completed = subprocess.run([
                sys.executable, '-m', 'planner.src.bt',
                '--request', str(inputs / 'generation_request.json'),
                '--model-response', str(response),
                '--output-dir', str(work / 'bt'),
            ], cwd=work, env=environment, capture_output=True, text=True, check=True)
            receipt = json.loads(completed.stdout)
            result = json.loads(Path(receipt['output']).read_text())
            self.assertEqual(result['tree'], recorded['tree'])
            self.assertEqual(result['generation']['provider_response_id'], 'recorded-response')
            for name in ('generated_plan.json', 'generated_bt.json', 'generated_bt.mdsl',
                         'generated_bt.mmd', 'generated_bt.html'):
                self.assertTrue((work / 'bt' / name).is_file(), name)
            self.assertEqual(completed.stderr, '')


if __name__ == '__main__':
    unittest.main()
