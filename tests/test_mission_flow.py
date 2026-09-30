"""Mission scans, fires, and stores results in one session directory."""

import tempfile
import threading
import time
import unittest
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import slam_detect_camera as mission
from src.slam_hardware import HardwareBackend
from src.telemetry import TelemetryRecorder


class MissionFlowTest(unittest.TestCase):
    def test_hardware_reports_each_measurement_before_next_yaw(self):
        backend = HardwareBackend.__new__(HardwareBackend)
        backend.robot = Mock()
        backend.controller = Mock()
        backend.hub = Mock()
        backend.hub.get_latest_state.return_value = SimpleNamespace(
            pos_x=0.0, pos_y=0.0, yaw=0.0)
        backend.heading = 0
        backend.gimbal_reference_ready = True
        backend.initial_scan_completed = True
        backend.event_log = []
        backend.ensure_running = Mock()
        backend.ensure_gimbal_front = Mock()
        backend.select_robot_mode = Mock()
        backend.aim = Mock(return_value=0.0)
        backend.sample = Mock(return_value=0.2)
        measurements = []

        def measured(*args):
            measurements.append((backend.sample.call_count, args))

        with (patch("src.slam_hardware.cancel_chassis_speed_timer"),
              patch("src.slam_hardware.load_robot_sdk",
                    return_value=SimpleNamespace(FREE="free"))):
            ranges, _, _ = backend.scan(on_measurement=measured)

        self.assertEqual(set(ranges), {0, 1, 3})
        self.assertEqual(measurements, [
            (1, (3, 0.2, 0, -90)),
            (2, (1, 0.2, 0, 90)),
            (3, (0, 0.2, 0, 0)),
        ])

    def test_confirmed_target_fires_before_next_wall(self):
        inspection = {"active": False}
        lock = threading.Lock()
        timeline = []
        inspected = set()

        class Finished:
            def is_set(self):
                return False

            def wait(self, seconds):
                if inspection["active"] and inspection["direction"] not in inspected:
                    direction = inspection["direction"]
                    inspected.add(direction)
                    timeline.append(("inspect", direction))
                    if direction == 3:
                        inspection["targets"].append({
                            "color": "Red", "shape": "Circle", "yaw": -90.0,
                            "pitch": -20.0, "confirmed": True,
                        })
                time.sleep(seconds)
                return False

        def scan(on_measurement=None):
            ranges = {3: 0.38, 1: 0.25, 0: 1.2}
            for direction, distance in ranges.items():
                timeline.append(("measure", direction))
                on_measurement(direction, distance, 0,
                               {3: -90, 1: 90, 0: 0}[direction])
            return ranges, 0, 0

        def fire_confirmed(cell, heading):
            if ("fire", 3) in timeline:
                return
            self.assertEqual((cell, heading), ((0, 0), 0))
            self.assertNotIn(-108.0, [call.kwargs["yaw"]
                                      for call in backend.robot.gimbal.moveto.call_args_list])
            self.assertEqual(
                [(target["direction"], target["color"], target["tof_distance_mm"])
                 for target in inspection["targets"]],
                [(3, "Red", 380.0)],
            )
            timeline.append(("fire", 3))

        running = threading.Event()
        running.set()
        backend = SimpleNamespace(
            scan=scan,
            robot=SimpleNamespace(gimbal=SimpleNamespace(
                moveto=Mock(return_value=SimpleNamespace(
                    has_succeeded=True, wait_for_completed=lambda timeout: True)))),
            hub=SimpleNamespace(get_latest_state=lambda: SimpleNamespace(
                vel_vx=0, vel_vy=0, vel_vz=0, yaw=0)),
            controller=SimpleNamespace(_running=running),
        )
        explorer = SimpleNamespace(
            backend=backend,
            slam=SimpleNamespace(cell=(0, 0), pose=(0, 0), events=[],
                                 offset=lambda heading, direction: (0, 0)),
        )
        original_setting = mission.setting

        def fast_setting(name):
            return 0 if name == "gimbal.settle_sec" else original_setting(name)

        with (patch.object(mission, "SIGN_INSPECTION_TIMEOUT_SEC", 0.02),
              patch.object(mission, "setting", side_effect=fast_setting)):
            mission.inspect_walls_during_scan(
                explorer, inspection, lock, Finished(),
                SimpleNamespace(fire_confirmed=fire_confirmed),
            )
            backend.scan()

        self.assertEqual(timeline, [
            ("measure", 3), ("inspect", 3), ("fire", 3),
            ("measure", 1), ("inspect", 1), ("measure", 0),
        ])
        self.assertEqual(backend.commanded_gimbal_yaw, 90)

    def test_results_share_mission_directory_and_old_captures_remain(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            current = root / "run_detect_current"
            previous = root / "run_detect_previous" / "captured_signs"
            captures = current / "captured_signs"
            previous.mkdir(parents=True)
            captures.mkdir(parents=True)
            old_image = previous / "old.jpg"
            current_image = captures / "old.jpg"
            old_image.write_bytes(b"old")
            current_image.write_bytes(b"current")

            self.assertEqual(mission.clear_previous_captures([captures]), 1)
            self.assertTrue(old_image.exists())
            self.assertFalse(current_image.exists())

            recorder = TelemetryRecorder(output_dir=root, run_name=current.name)
            telemetry_path = recorder.export()
            self.assertEqual(telemetry_path.parent, current)
            self.assertTrue(telemetry_path.with_suffix(".csv").exists())
            self.assertFalse((root / "run1").exists())

    def test_hardware_mission_passes_map_directory_to_telemetry(self):
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "run_detect_current" / "explored_map.json"
            with patch("src.robot_system.RobotSystem") as system_class:
                system_class.return_value.connect_robot.return_value = False
                system_class.return_value.thread_2_controller = None
                with self.assertRaisesRegex(RuntimeError, "Could not connect"):
                    mission.run_hardware("ap", Path(temp) / "calibration.json", output)
            self.assertEqual(system_class.call_args.kwargs["results_dir"], output.parent)

    def test_action_and_event_reports_share_map_directory(self):
        with tempfile.TemporaryDirectory() as temp:
            map_path = Path(temp) / "run_detect_current" / "explored_map.json"
            map_path.parent.mkdir()
            map_path.write_text(json.dumps({
                "start_cell": [0, 0],
                "status": "completed", "visited": [[0, 0]],
                "events": [{
                    "timestamp": 1.0, "type": "target_fire", "direction": 3,
                    "color": "Red", "shape": "Circle", "shots_requested": 3,
                    "fire_commands_sent": 1,
                }],
            }), encoding="utf-8")

            actions, events = mission.save_mission_reports(map_path)
            self.assertEqual(actions.parent, map_path.parent)
            self.assertEqual(events.parent, map_path.parent)
            self.assertIn("FIRE Red Circle on W wall", actions.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
