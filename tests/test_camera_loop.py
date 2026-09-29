"""Regression checks for the camera loop without RoboMaster hardware."""

import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import slam_detect_camera as mission


class CameraLoopTest(unittest.TestCase):
    def test_first_frame_delay_then_sign_does_not_stop_motion(self):
        sign_saved = threading.Event()
        explorer = SimpleNamespace(
            output=Path("unused/explored_map.json"),
            status="running",
            error=None,
            moves=0,
            slam=SimpleNamespace(cell=(0, 0), heading=0,
                                 map=SimpleNamespace(visited={(0, 0)})),
        )

        def explore():
            if not sign_saved.wait(timeout=2):
                raise AssertionError("camera loop did not process the sign")
            explorer.status = "completed"
            return True

        explorer.run = explore
        camera = Mock()
        frames = iter((None, object(), object(), object()))
        camera.read_cv2_image.side_effect = lambda **kwargs: next(frames, None)
        stop_motion = Mock()

        def enable_inspection(_explorer, inspection, _lock, _finished):
            inspection.update(active=True, cell=(0, 0), direction=1,
                              tof_distance_mm=200, frame_count=0)

        def check_window(*_args):
            self.assertTrue(imshow.called, "window checked before the first frame")
            return 1

        with (patch.object(mission, "inspect_walls_during_scan", enable_inspection),
              patch.object(mission, "clear_previous_captures"),
              patch.object(mission.Path, "mkdir"),
              patch.object(mission, "detect_signs", return_value=(object(), object(),
                  [{"color": "Red", "shape": "Circle", "confidence": 1.0}])),
              patch.object(mission, "build_side_by_side_view", return_value=object()),
              patch.object(mission.cv2, "putText"),
              patch.object(mission.cv2, "imshow") as imshow,
              patch.object(mission.cv2, "setWindowTitle"),
              patch.object(mission.cv2, "getWindowProperty", side_effect=check_window) as window_property,
              patch.object(mission.cv2, "waitKey", return_value=-1),
              patch.object(mission.cv2, "imwrite", side_effect=lambda path, _frame:
                  (sign_saved.set() if path.endswith("snap3.jpg") else None) or True) as imwrite,
              patch.object(mission.cv2, "destroyAllWindows"),
              patch.object(mission, "save_sign_marks") as save_sign_marks):
            self.assertTrue(mission.run_camera_loop(explorer, camera, True, stop_motion))

        imshow.assert_called()
        window_property.assert_called()
        self.assertEqual(imwrite.call_count, 3)
        self.assertIn("c0_0_E_Red_Circle_snap3.jpg", imwrite.call_args.args[0])
        marks = save_sign_marks.call_args.args[1]
        self.assertEqual(len(marks), 1)
        self.assertEqual(next(iter(marks.values()))["confirmed_frames"], 3)
        stop_motion.assert_not_called()


if __name__ == "__main__":
    unittest.main()
