"""Chassis heading reset after every two successful grid moves."""

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from src.grid_slam import FrontierExplorer
from src.slam_hardware import HardwareBackend
from src.slam_report import action_steps


class TwoCellBackend:
    def __init__(self, aligned_yaw=4.8):
        self.cell = 0
        self.heading = 0
        self.aligned_yaw = aligned_yaw
        self.calls = []

    def scan(self):
        ranges = {0: 1.2 if self.cell < 2 else 0.2,
                  1: 0.2, 2: 1.2 if self.cell > 0 else 0.2, 3: 0.2}
        self.scan_headings = {direction: self.heading for direction in ranges}
        return ranges, self.heading, float(self.heading * 90)

    def move(self, direction):
        self.calls.append(("move", direction))
        self.cell += 1
        self.heading = direction
        return (0.6, 0.0), float(direction * 90)

    def face_zero(self, tolerance_deg):
        self.calls.append(("face_zero", tolerance_deg))
        self.heading = 0
        return self.aligned_yaw

    def stop(self):
        pass


class FaceZeroTest(unittest.TestCase):
    def run_explorer(self, aligned_yaw):
        backend = TwoCellBackend(aligned_yaw)
        with tempfile.TemporaryDirectory() as directory:
            explorer = FrontierExplorer(backend, Path(directory) / "map.json")
            result = explorer.run()
        return explorer, backend, result

    def test_reset_occurs_after_second_successful_move(self):
        explorer, backend, result = self.run_explorer(4.8)
        self.assertTrue(result)
        self.assertEqual(backend.calls, [("move", 0), ("move", 0), ("face_zero", 5)])
        self.assertEqual(explorer.slam.heading, 0)
        self.assertEqual(len([event for event in explorer.slam.events
                              if event["type"] == "face_zero"]), 1)
        steps = action_steps({"events": explorer.slam.events, "start_cell": [0, 0]})
        self.assertTrue(any("ALIGN verified" in action
                            for step in steps for action in step["actions"]))

    def test_out_of_tolerance_reset_fails_mission(self):
        explorer, _, result = self.run_explorer(5.1)
        self.assertFalse(result)
        self.assertEqual(explorer.moves, 2)
        self.assertIn("outside tolerance", explorer.error)

    def test_hardware_turns_from_east_to_initial_heading(self):
        backend = HardwareBackend.__new__(HardwareBackend)
        backend.ensure_running = Mock()
        backend.heading = 1
        backend.controller = Mock(target_heading_deg=90.0)
        backend.controller.turn_to_relative.return_value = SimpleNamespace(yaw=3.2)
        backend.prepare_stationary_scan = Mock(return_value=SimpleNamespace(yaw=3.2))

        yaw = backend.face_zero(5)

        self.assertEqual(yaw, 3.2)
        self.assertEqual(backend.heading, 0)
        self.assertTrue(backend.force_full_scan)
        self.assertEqual(backend.controller.target_heading_deg, 0.0)
        backend.controller.turn_to_relative.assert_called_once_with(90.0, tolerance_deg=5)
        backend.prepare_stationary_scan.assert_called_once_with()

    def test_scan_returns_gimbal_with_position_action(self):
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
        backend.recenter_gimbal = Mock()

        with (patch("src.slam_hardware.cancel_chassis_speed_timer"),
              patch("src.slam_hardware.load_robot_sdk",
                    return_value=SimpleNamespace(FREE="free"))):
            ranges, heading, yaw = backend.scan()

        self.assertEqual(set(ranges), {0, 1, 3})
        self.assertEqual((heading, yaw), (0, 0.0))
        self.assertEqual(backend.aim.call_args.args, (0,))
        self.assertEqual(backend.aim.call_args.kwargs, {"direct": True})
        backend.recenter_gimbal.assert_not_called()

    def test_heading_reset_forces_next_scan_to_include_rear(self):
        backend = HardwareBackend.__new__(HardwareBackend)
        backend.robot = Mock()
        backend.controller = Mock()
        backend.hub = Mock()
        backend.hub.get_latest_state.return_value = SimpleNamespace(
            pos_x=0.0, pos_y=0.0, yaw=0.0)
        backend.heading = 0
        backend.gimbal_reference_ready = True
        backend.initial_scan_completed = True
        backend.force_full_scan = True
        backend.event_log = []
        backend.ensure_running = Mock()
        backend.ensure_gimbal_front = Mock()
        backend.select_robot_mode = Mock()
        backend.aim = Mock(return_value=0.0)
        backend.sample = Mock(return_value=0.2)

        with (patch("src.slam_hardware.cancel_chassis_speed_timer"),
              patch("src.slam_hardware.load_robot_sdk",
                    return_value=SimpleNamespace(FREE="free"))):
            ranges, _, _ = backend.scan()

        self.assertEqual(set(ranges), {0, 1, 2, 3})
        self.assertFalse(backend.force_full_scan)


if __name__ == "__main__":
    unittest.main()
