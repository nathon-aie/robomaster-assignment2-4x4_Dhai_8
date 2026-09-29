"""Camera inspection must allow time at both side walls after Gimbal travel."""

import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import slam_detect_camera as mission


class WallInspectionTest(unittest.TestCase):
    def test_right_and_left_wall_each_get_camera_dwell(self):
        active = {"active": False}
        inspected = set()

        class Finished:
            def clear(self):
                pass

            def is_set(self):
                return False

            def wait(self, seconds):
                if active["active"]:
                    inspected.add(active["direction"])
                time.sleep(seconds)
                return False

        class Action:
            has_succeeded = True

            def wait_for_completed(self, timeout):
                time.sleep(0.03)  # Longer than the inspection dwell.
                return True

        running = threading.Event()
        running.set()
        backend = SimpleNamespace(
            scan=lambda: ({1: 0.25, 3: 0.38}, 0, 0),
            scan_headings={1: 0, 3: 0},
            robot=SimpleNamespace(gimbal=SimpleNamespace(moveto=Mock(return_value=Action()))),
            hub=SimpleNamespace(get_latest_state=lambda: SimpleNamespace(
                vel_vx=0, vel_vy=0, vel_vz=0, yaw=0)),
            controller=SimpleNamespace(_running=running),
        )
        explorer = SimpleNamespace(
            backend=backend,
            slam=SimpleNamespace(cell=(0, 0), pose=(0, 0),
                                 events=[],
                                 offset=lambda heading, direction: (0, 0)),
        )
        original_setting = mission.setting

        def fast_setting(name):
            return 0 if name == "gimbal.settle_sec" else original_setting(name)

        with (patch.object(mission, "SIGN_INSPECTION_TIMEOUT_SEC", 0.02),
              patch.object(mission, "setting", side_effect=fast_setting)):
            mission.inspect_walls_during_scan(explorer, active, threading.Lock(), Finished())
            backend.scan()

        self.assertEqual(inspected, {1, 3})
        yaws = {call.kwargs["yaw"] for call in backend.robot.gimbal.moveto.call_args_list}
        self.assertTrue({72.0, 90.0, 108.0, -72.0, -90.0, -108.0} <= yaws)
        summaries = [event for event in explorer.slam.events
                     if event["type"] == "sign_inspection"]
        self.assertEqual({event["direction"] for event in summaries}, {"E", "W"})


if __name__ == "__main__":
    unittest.main()
