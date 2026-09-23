"""Shared MDSL and composite ticking contracts used by both task adapters."""

import unittest

from planner.src.bt.core import (
    MdslParser, Node, Status, TickOutcome, tick_tree, to_dict, to_mdsl, to_mermaid,
)


class BtCoreTest(unittest.TestCase):
    def test_parser_and_serializers_share_one_node_shape(self):
        parser = MdslParser('sequence { action [Wait, "1.0"] }',
                            signatures={"Wait": ("duration_s",), "Done": ()},
                            conditions={"Done"})
        sequence = parser.parse()
        root = Node("root", children=(Node("selector", children=(
            Node("condition", "Done"), sequence)),))
        self.assertIn('action [Wait, "1.0"]', to_mdsl(root))
        self.assertEqual(to_dict(root)["children"][0]["children"][1]["kind"], "sequence")
        self.assertIn("Wait(1.0)", to_mermaid(root))
        self.assertEqual(to_mermaid(root), to_mermaid(to_dict(root)))
        with self.assertRaisesRegex(ValueError, "Wrong BT node class"):
            MdslParser("action [Done]", signatures={"Done": ()},
                       conditions={"Done"}).parse()

    def test_sequence_resumes_and_success_guard_short_circuits(self):
        sequence = Node("sequence", children=(
            Node("action", "First"), Node("action", "Second"),
        ))
        selector = Node("selector", children=(Node("condition", "Done"), sequence))
        root = Node("root", children=(selector,))
        indexes = {}
        calls = []
        observation = {"done": False}

        def condition(node, path, state):
            calls.append(node.name)
            return TickOutcome(Status.SUCCESS if state["done"] else Status.FAILURE, path=path)

        def action(node, path, state):
            calls.append(node.name)
            return TickOutcome(Status.SUCCESS, action=node.name, path=path)

        first = tick_tree(root, "0", observation, indexes, condition, action)
        self.assertEqual((first.status, first.action), (Status.RUNNING, "First"))
        second = tick_tree(root, "0", observation, indexes, condition, action)
        self.assertEqual((second.status, second.action), (Status.SUCCESS, "Second"))
        self.assertEqual(calls, ["Done", "First", "Done", "Second"])
        observation["done"] = True
        third = tick_tree(root, "0", observation, indexes, condition, action)
        self.assertEqual(third.status, Status.SUCCESS)
        self.assertEqual(calls[-1], "Done")


if __name__ == "__main__":
    unittest.main()
