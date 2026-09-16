"""Regression checks for the standalone BT artifact shared by generation CLIs."""

import json
import re
import unittest

from examples.online_manipulation.bt_visualization import render_viewer


class BtVisualizationTest(unittest.TestCase):
    def test_embedded_tree_is_safe_and_offline(self):
        hostile = '</script><script>alert("x")</script>'
        tree = {"kind": "root", "name": "", "args": [], "children": [
            {"kind": "action", "name": hostile, "args": [], "children": []},
        ]}
        page = render_viewer(tree, plan={"parsed_response": {"ULTIMATE_GOAL": hostile}})
        match = re.search(r'<script type="application/json" id="bt-data">(.*?)</script>', page, re.S)
        self.assertIsNotNone(match)
        payload = json.loads(match.group(1))
        self.assertEqual(payload["tree"]["children"][0]["name"], hostile)
        self.assertNotIn(hostile, match.group(1))
        self.assertNotIn("https://", page)

    def test_rejects_missing_root(self):
        with self.assertRaisesRegex(ValueError, "root"):
            render_viewer({"kind": "sequence", "children": []})


if __name__ == "__main__":
    unittest.main()
