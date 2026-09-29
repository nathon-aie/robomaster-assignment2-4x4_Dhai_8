"""Checks that ambiguous colors and shapes are rejected."""

import unittest
from pathlib import Path

import cv2
import numpy as np

from detect_camera import classify_color_relative, classify_shape_perspective, detect_signs


class StrictDetectionTest(unittest.TestCase):
    def test_mixed_red_and_blue_is_not_classified_as_green(self):
        roi = np.empty((80, 80, 3), dtype=np.uint8)
        roi[:, :40] = (0, 0, 255)
        roi[:, 40:] = (255, 0, 0)
        color, confidence = classify_color_relative(roi)
        self.assertIsNone(color)
        self.assertEqual(confidence, 0.0)

    def test_ambiguous_rectangle_is_not_called_square(self):
        contour = np.array([[[100, 100]], [[235, 100]],
                            [[235, 200]], [[100, 200]]], dtype=np.int32)
        self.assertIsNone(classify_shape_perspective(contour, (400, 400)))

    def test_very_dark_green_sign_on_light_wall(self):
        frame = np.full((400, 500, 3), 220, dtype=np.uint8)
        cv2.circle(frame, (250, 200), 45, (4, 30, 6), -1)
        detections = detect_signs(frame)[2]
        self.assertIn(("Green", "Circle"),
                      {(item["color"], item["shape"]) for item in detections})

    def test_latest_run_shape_regressions(self):
        captures = (Path(__file__).resolve().parents[1] / "telemetry_logs"
                    / "run_detect_20260930_021710" / "captured_signs")
        if not captures.is_dir():
            self.skipTest("recorded camera frames are unavailable")
        examples = (
            ("c5_3_N_Green_Circle_snap3.jpg", "Green", "Square"),
            ("c5_1_E_Yellow_Square_snap2.jpg", "Yellow", "Vertical_Rect"),
        )
        for filename, color, shape in examples:
            with self.subTest(filename=filename):
                frame = cv2.imread(str(captures / filename))
                self.assertIsNotNone(frame)
                detections = detect_signs(frame)[2]
                self.assertIn((color, shape),
                              {(item["color"], item["shape"]) for item in detections})


if __name__ == "__main__":
    unittest.main()
