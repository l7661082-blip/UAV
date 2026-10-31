"""Offline regression tests for the mission command parser.

Only the production parser and its constants are loaded, so this test does not
initialize ROS or load model weights.
"""

import ast
import re
import types
import unittest
from pathlib import Path


def load_parser():
    source = Path(__file__).resolve().parents[1] / "mission_once.py"
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    names = {"SIMPLE_COMMANDS", "BALLOON_COMMAND", "FENCED_BLOCK"}
    selected = [
        node for node in tree.body
        if (isinstance(node, ast.Assign)
            and any(isinstance(target, ast.Name) and target.id in names
                    for target in node.targets))
        or (isinstance(node, ast.FunctionDef)
            and node.name == "parse_model_commands")
    ]
    assert len(selected) == 4, "Production parser layout changed; update this test"
    namespace = {"re": re, "rospy": types.SimpleNamespace(logwarn=lambda *_: None)}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(source), "exec"), namespace)
    return namespace["parse_model_commands"]


class MissionCommandFormatsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parse = staticmethod(load_parser())

    def test_command_fence(self):
        self.assertEqual(self.parse("```command\ntakeoff\nland\n```"), "takeoff\nland")

    def test_bare_fence(self):
        self.assertEqual(self.parse("```\ntakeoff\nland\n```"), "takeoff\nland")

    def test_no_fence(self):
        self.assertEqual(self.parse("任务指令：\ntakeoff\nland"), "takeoff\nland")

    def test_blank_lines(self):
        self.assertEqual(self.parse("```command\n\ntakeoff\n\nland\n\n```"),
                         "takeoff\nland")

    def test_crlf_and_cr(self):
        self.assertEqual(self.parse("```command\r\ntakeoff\r\nland\r\n```"),
                         "takeoff\nland")
        self.assertEqual(self.parse("```\rtakeoff\rland\r```"), "takeoff\nland")

    def test_illegal_command(self):
        with self.assertRaisesRegex(ValueError, "不支持的命令"):
            self.parse("```\ntakeoff\nfly_anywhere\n```")

    def test_multiple_code_blocks(self):
        with self.assertRaisesRegex(ValueError, "多个代码块"):
            self.parse("```command\ntakeoff\n```\n```\nland\n```")


if __name__ == "__main__":
    unittest.main()
