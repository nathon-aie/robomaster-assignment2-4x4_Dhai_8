"""Regression checks for camera sign confirmation without robot hardware."""

import threading
import unittest

import cv2
import numpy as np

from detect_camera import detect_signs
from slam_detect_camera import SignConfirmationTracker
from src.target_fire import TargetFireController


def sign(center, confidence=0.9):
    return {"color": "Red", "shape": "Circle", "center": center,
            "confidence": confidence}


class SignConfirmationTests(unittest.TestCase):
    def test_all_color_selection_accepts_four_colors_and_keeps_target_identity(self):
        inspection = {"active": True, "phase": "survey",
                      "camera_yaw": 0, "camera_pitch": -20}
        controller = TargetFireController(
            robot=None, inspection=inspection, lock=threading.Lock(),
            stopped=threading.Event(), mode="water_fire", events=[],
            target_color="All", target_shape="All",
        )
        for color in ("Red", "Yellow", "Blue", "Green"):
            self.assertTrue(controller.matches_target(
                {"color": color, "shape": "Circle"}))
        controller.target_shape = "Circle"
        self.assertFalse(controller.matches_target(
            {"color": "Red", "shape": "Square"}))
        controller.target_shape = "All"
        self.assertFalse(controller._matches_sighting(
            {"color": "Red", "shape": "Circle", "yaw": 10, "pitch": -20},
            {"color": "Blue", "shape": "Circle", "yaw": 10, "pitch": -20}))
        self.assertTrue(controller._matches_sighting(
            {"color": "Red", "shape": "Circle", "yaw": 10, "pitch": -20},
            {"color": "Red", "shape": "Square", "yaw": 12, "pitch": -21}))
        self.assertFalse(controller._matches_sighting(
            {"color": "Red", "shape": "Circle", "yaw": 10, "pitch": -20},
            {"color": "Red", "shape": "Square", "yaw": 30, "pitch": -21}))

    def test_position_jump_missing_frame_and_gimbal_move_restart_streak(self):
        tracker = SignConfirmationTracker()

        def update(detections, yaw=0):
            return tracker.update(detections, (1, 2), 0, yaw, -20, (480, 640, 3))

        self.assertEqual(update([sign((320, 240))])[0][2], 1)
        self.assertEqual(update([sign((323, 241))])[0][2], 2)
        self.assertEqual(update([sign((500, 240))])[0][2], 1)
        self.assertEqual(update([sign((501, 240))])[0][2], 2)
        self.assertEqual(update([sign((502, 240))])[0][2], 3)
        self.assertEqual(update([]), [])
        self.assertEqual(update([sign((502, 240))])[0][2], 1)
        self.assertEqual(update([sign((502, 240))], yaw=18)[0][2], 1)

    def test_real_detector_output_tracks_one_synthetic_card(self):
        tracker = SignConfirmationTracker()

        def frame_with_red_card(x):
            frame = np.full((480, 640, 3), 235, dtype=np.uint8)
            cv2.circle(frame, (x, 240), 55, (0, 0, 255), -1)
            return frame

        for expected, x in ((1, 320), (2, 323), (3, 325), (1, 480)):
            frame = frame_with_red_card(x)
            detections = detect_signs(frame)[2]
            self.assertEqual(len(detections), 1)
            self.assertEqual(detections[0]["shape"], "Circle")
            sightings = tracker.update(detections, (1, 2), 0, 0, -20, frame.shape)
            self.assertEqual(sightings[0][2], expected)

    def test_fire_track_cannot_be_forced_confirmed_from_one_frame(self):
        inspection = {"active": True, "phase": "survey",
                      "camera_yaw": 0, "camera_pitch": -20}
        controller = TargetFireController(
            robot=None, inspection=inspection, lock=threading.Lock(),
            stopped=threading.Event(), mode="water_fire", events=[],
            target_color="Red", target_shape="Circle",
        )
        detection = sign((320, 240))
        controller.observe([detection], (480, 640, 3))
        controller.confirm_detection(detection)
        self.assertFalse(inspection["targets"][0]["confirmed"])

        controller.observe([detection], (480, 640, 3))
        controller.observe([detection], (480, 640, 3))
        controller.confirm_detection(detection)
        self.assertTrue(inspection["targets"][0]["confirmed"])

        controller.observe([], (480, 640, 3))
        self.assertEqual(inspection["targets"], [])


if __name__ == "__main__":
    unittest.main()
