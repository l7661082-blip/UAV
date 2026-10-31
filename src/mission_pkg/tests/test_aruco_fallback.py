"""Offline tests for ArUco fallback order and uneven lighting."""

import ast
import types
import unittest
from pathlib import Path
from unittest import mock


SOURCE = Path(__file__).resolve().parents[1] / "scripts" / "aruco_detect_node.py"


def load_detect(cv2_module, rospy_module):
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                and n.name == "ArucoDetectNode")
    method = next(n for n in node.body if isinstance(n, ast.FunctionDef)
                  and n.name == "_detect")
    namespace = {"cv2": cv2_module, "rospy": rospy_module}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"), namespace)
    return namespace["_detect"]


class ArucoFallbackTest(unittest.TestCase):
    def make_node(self):
        node = mock.Mock()
        node.frame_index = 0
        node.binary_interval = 3
        node.binary_fallback = True
        node.clahe_fallback = True
        node.binary_block_size = 31
        node.binary_c = 7
        return node

    def test_original_success_skips_preprocessing(self):
        cv2 = mock.Mock()
        cv2.aruco.detectMarkers.return_value = ("corners", "ids", [])
        detect = load_detect(cv2, mock.Mock())
        self.assertEqual(detect(self.make_node(), "gray")[1], "ids")
        cv2.adaptiveThreshold.assert_not_called()

    def test_fallback_only_on_interval(self):
        cv2 = mock.Mock()
        cv2.aruco.detectMarkers.return_value = ([], None, [])
        node = self.make_node()
        detect = load_detect(cv2, mock.Mock())
        self.assertIsNone(detect(node, "gray")[1])
        cv2.adaptiveThreshold.assert_not_called()

    def test_clahe_after_binary_failure(self):
        cv2 = mock.Mock()
        cv2.aruco.detectMarkers.side_effect = [
            ([], None, []), ([], None, []), ("corners", "ids", [])]
        node = self.make_node()
        node.frame_index = 2
        node.clahe.apply.return_value = "equalized"
        detect = load_detect(cv2, mock.Mock())
        self.assertEqual(detect(node, "gray")[1], "ids")
        node.clahe.apply.assert_called_once_with("gray")
        self.assertEqual(cv2.aruco.detectMarkers.call_count, 3)

    def test_clahe_threshold_is_last_fallback(self):
        cv2 = mock.Mock()
        cv2.aruco.detectMarkers.side_effect = [
            ([], None, []), ([], None, []), ([], None, []),
            ("corners", "ids", [])]
        node = self.make_node()
        node.frame_index = 2
        node.clahe.apply.return_value = "equalized"
        detect = load_detect(cv2, mock.Mock())
        self.assertEqual(detect(node, "gray")[1], "ids")
        self.assertEqual(cv2.adaptiveThreshold.call_count, 2)
        self.assertEqual(cv2.aruco.detectMarkers.call_count, 4)

    def test_uneven_lighting_marker(self):
        try:
            import cv2
            import numpy as np
        except ImportError:
            self.skipTest("OpenCV and NumPy are required for image regression")

        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        if hasattr(cv2.aruco, "generateImageMarker"):
            marker = cv2.aruco.generateImageMarker(dictionary, 42, 160)
        else:
            marker = cv2.aruco.drawMarker(dictionary, 42, 160)
        canvas = np.full((320, 320), 255, dtype=np.uint8)
        canvas[80:240, 80:240] = marker
        illumination = np.linspace(0.45, 1.0, 320, dtype=np.float32)
        shaded = np.uint8(canvas * illumination[np.newaxis, :])

        if hasattr(cv2.aruco, "DetectorParameters_create"):
            params = cv2.aruco.DetectorParameters_create()
            detect_markers = cv2.aruco.detectMarkers
        else:
            params = cv2.aruco.DetectorParameters()
            detect_markers = lambda image, dictionary, parameters: (
                cv2.aruco.ArucoDetector(dictionary, parameters).detectMarkers(image))
        attempts = []

        def detect_after_failed_original_and_binary(image, dictionary, parameters):
            attempts.append(image)
            if len(attempts) <= 2:
                return [], None, []
            return detect_markers(image, dictionary, parameters)

        compat_cv2 = types.SimpleNamespace(
            aruco=types.SimpleNamespace(detectMarkers=detect_after_failed_original_and_binary),
            adaptiveThreshold=cv2.adaptiveThreshold,
            ADAPTIVE_THRESH_GAUSSIAN_C=cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            THRESH_BINARY=cv2.THRESH_BINARY)
        node = self.make_node()
        node.frame_index = 2
        node.aruco_dict = dictionary
        node.params = params
        node.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        corners, ids, _ = load_detect(compat_cv2, mock.Mock())(node, shaded)
        self.assertGreaterEqual(len(attempts), 3)
        self.assertIsNotNone(ids)
        self.assertEqual(int(ids[0][0]), 42)


if __name__ == "__main__":
    unittest.main()
