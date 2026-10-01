import json
import queue
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from src.sdk_connection import require_camera_codec
from src.robot_controller import RobotControllerThread
from src.sensor_pipeline import RobotSensorSnapshot
from src.settings import get as setting
from src.target_navigation import (_prepare_target_wall, _wait_for_first_camera_frame,
                                   load_explore_map, make_plan,
                                   shortest_target_route, sign_key, straight_runs)


class TargetNavigationTests(unittest.TestCase):
    def test_straight_runs_stop_at_signs_and_turns(self):
        plan = {
            "route": [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 1]],
            "directions": [1, 1, 0, 0, 3],
            "targets": [{"cell": [0, 2]}, {"cell": [2, 2]}],
        }
        self.assertEqual(list(straight_runs(plan)),
                         [(0, 2, 1), (2, 4, 0), (4, 5, 3)])

    def test_continuous_run_reports_middle_cell_without_stopping(self):
        positions = iter((0.0, 0.2, 0.61, 0.9, 1.21, 1.21, 1.21))
        controller = RobotControllerThread(SimpleNamespace())
        controller._running.set()
        controller.motion_state = lambda: RobotSensorSnapshot(pos_x=next(positions))
        controller.wall_pid.compute_control_speeds = lambda **kwargs: (
            0.35, 0.0, 0.0, "open", 1, 0.0)
        calls = []
        controller.drive_speed = lambda **kwargs: calls.append("drive")
        controller.stop_chassis = lambda: calls.append("stop")
        controller.align_at_cell_center = lambda **kwargs: calls.append("align")

        result = controller.navigate_straight_cells(
            2, on_cell=lambda count: calls.append("cell{}".format(count)))

        self.assertTrue(result["completed"])
        self.assertEqual(calls.count("cell1"), 1)
        self.assertLess(calls.index("cell1"), calls.index("stop"))
        self.assertEqual(calls.count("align"), 1)

    def test_continuous_run_stops_for_unexpected_front_wall(self):
        states = iter((
            RobotSensorSnapshot(pos_x=0.0),
            RobotSensorSnapshot(pos_x=0.2),
            RobotSensorSnapshot(pos_x=0.4, tof_valid=True, tof_filtered_mm=260.0),
            RobotSensorSnapshot(pos_x=0.4),
        ))
        controller = RobotControllerThread(SimpleNamespace())
        controller._running.set()
        controller.motion_state = lambda: next(states)
        controller.wall_pid.compute_control_speeds = lambda **kwargs: (
            0.35, 0.0, 0.0, "open", 1, 0.0)
        controller.drive_speed = lambda **kwargs: None
        stopped = []
        reached = []
        controller.stop_chassis = lambda: stopped.append(True)

        result = controller.navigate_straight_cells(2, on_cell=reached.append)

        self.assertFalse(result["completed"])
        self.assertEqual(result["reason"], "front_wall")
        self.assertEqual(reached, [])
        self.assertTrue(stopped)

    def test_short_sensor_packet_gap_stops_and_recovers_before_motion(self):
        now = time.monotonic()

        def snapshot(received_at):
            return RobotSensorSnapshot(
                tof_received_at=received_at, position_received_at=received_at,
                attitude_received_at=received_at, gimbal_received_at=received_at,
                sharp_left_received_at=received_at,
                sharp_right_received_at=received_at,
                pos_x=0.0, pos_y=0.0, yaw=0.0, gimbal_yaw=0.0,
                gimbal_pitch=0.0, tof_raw=500.0, tof_valid=True,
                tof_filtered_mm=500.0)

        stale = snapshot(now - 0.4)
        fresh = snapshot(now)
        states = iter((stale, fresh))
        controller = RobotControllerThread(
            SimpleNamespace(get_latest_state=lambda: next(states)))
        controller.strict_sensors = True
        controller.front_ready = True
        controller.calibration_manager = SimpleNamespace(
            raw_to_mm=lambda kind, value: value)
        controller._running.set()
        calls = []
        controller.stop_chassis = lambda: calls.append("stop")
        controller.wall_pid.reset = lambda: calls.append("reset")

        self.assertEqual(controller.motion_state().tof_received_at,
                         fresh.tof_received_at)
        self.assertEqual(calls, ["stop", "reset"])

    def test_sensor_outage_stops_and_fails_after_recovery_timeout(self):
        received_at = time.monotonic() - 1.0
        stale = SimpleNamespace(**{name: received_at for name in (
            "tof_received_at", "position_received_at", "attitude_received_at",
            "gimbal_received_at", "sharp_left_received_at",
            "sharp_right_received_at")})
        controller = RobotControllerThread(
            SimpleNamespace(get_latest_state=lambda: stale))
        controller.strict_sensors = True
        controller._running.set()
        stopped = []
        controller.stop_chassis = lambda: stopped.append(True)

        real_setting = setting
        def short_timeout(name):
            if name == "navigation.sensor_recovery_timeout_sec":
                return 0.02
            return real_setting(name)

        with patch("src.robot_controller.setting", side_effect=short_timeout):
            with self.assertRaisesRegex(RuntimeError, "Stale sensor stream"):
                controller.motion_state()
        self.assertEqual(stopped, [True])

    def test_turn_alignment_waits_stationary_for_fresh_attitude(self):
        calls = []

        def latest_state():
            calls.append("read")
            received = time.monotonic() - 0.4 if len(calls) == 1 else time.monotonic()
            return SimpleNamespace(attitude_received_at=received,
                                   position_received_at=received,
                                   pos_x=0.0, pos_y=0.0, yaw=90.0)

        controller = RobotControllerThread(
            SimpleNamespace(get_latest_state=latest_state))
        controller._running.set()
        controller.stop_chassis = lambda: calls.append("stop")

        state = controller.align_turn_heading(90.0)

        self.assertEqual(state.yaw, 90.0)
        self.assertLess(calls.index("stop"), calls.index("read", 1))

    def test_turn_alignment_fails_if_attitude_does_not_recover(self):
        stale = SimpleNamespace(attitude_received_at=time.monotonic() - 1.0,
                                yaw=90.0)
        controller = RobotControllerThread(
            SimpleNamespace(get_latest_state=lambda: stale))
        controller._running.set()
        stops = []
        controller.stop_chassis = lambda: stops.append(True)

        real_setting = setting
        def short_timeout(name):
            if name == "navigation.sensor_recovery_timeout_sec":
                return 0.02
            return real_setting(name)

        with patch("src.robot_controller.setting", side_effect=short_timeout):
            with self.assertRaisesRegex(RuntimeError, "Stale chassis attitude"):
                controller.align_turn_heading(90.0)
        self.assertTrue(stops)

    def test_navigation_pause_keeps_mission_alive_until_fresh_pose(self):
        paused = threading.Event()
        release = threading.Event()
        phases = []
        stale_at = time.monotonic() - 1.0

        def snapshot():
            received = time.monotonic() if release.is_set() else stale_at
            return RobotSensorSnapshot(
                tof_received_at=received, position_received_at=received,
                attitude_received_at=received, gimbal_received_at=received,
                sharp_left_received_at=received,
                sharp_right_received_at=received,
                pos_x=0.0, pos_y=0.0, yaw=0.0, gimbal_yaw=0.0,
                gimbal_pitch=0.0, tof_raw=500.0, tof_valid=True,
                tof_filtered_mm=500.0)

        controller = RobotControllerThread(
            SimpleNamespace(get_latest_state=snapshot))
        controller.strict_sensors = True
        controller.pause_on_sensor_outage = True
        controller.front_ready = True
        controller.calibration_manager = SimpleNamespace(
            raw_to_mm=lambda kind, value: value)
        controller._running.set()
        controller.stop_chassis = lambda: None
        controller.recovery_callback = lambda phase, detail: (
            phases.append(phase), paused.set() if phase == "paused" else None)
        outcome = []

        real_setting = setting
        def short_grace(name):
            if name == "navigation.sensor_recovery_timeout_sec":
                return 0.02
            return real_setting(name)

        with patch("src.robot_controller.setting", side_effect=short_grace):
            worker = threading.Thread(target=lambda: outcome.append(controller.motion_state()))
            worker.start()
            self.assertTrue(paused.wait(1.0))
            self.assertTrue(worker.is_alive())
            release.set()
            worker.join(1.0)
        self.assertFalse(worker.is_alive())
        self.assertEqual(phases, ["paused", "resumed"])
        self.assertEqual(len(outcome), 1)

    def test_navigation_pause_can_be_cancelled(self):
        stale_at = time.monotonic() - 1.0
        stale = RobotSensorSnapshot(
            tof_received_at=stale_at, position_received_at=stale_at,
            attitude_received_at=stale_at, gimbal_received_at=stale_at,
            sharp_left_received_at=stale_at, sharp_right_received_at=stale_at)
        controller = RobotControllerThread(
            SimpleNamespace(get_latest_state=lambda: stale))
        controller.strict_sensors = True
        controller.pause_on_sensor_outage = True
        controller._running.set()
        controller.stop_chassis = lambda: None
        paused = threading.Event()
        controller.recovery_callback = lambda phase, detail: paused.set()
        outcome = []

        real_setting = setting
        def short_grace(name):
            if name == "navigation.sensor_recovery_timeout_sec":
                return 0.02
            return real_setting(name)

        def attempt():
            try:
                controller.motion_state()
            except KeyboardInterrupt:
                outcome.append("cancelled")

        with patch("src.robot_controller.setting", side_effect=short_grace):
            worker = threading.Thread(target=attempt)
            worker.start()
            self.assertTrue(paused.wait(1.0))
            controller.stop_running()
            worker.join(1.0)
        self.assertFalse(worker.is_alive())
        self.assertEqual(outcome, ["cancelled"])

    def test_turn_alignment_resumes_after_long_attitude_gap(self):
        paused = threading.Event()
        release = threading.Event()
        stale_at = time.monotonic() - 1.0

        def snapshot():
            received = time.monotonic() if release.is_set() else stale_at
            return SimpleNamespace(attitude_received_at=received,
                                   position_received_at=received,
                                   pos_x=0.0, pos_y=0.0, yaw=90.0)

        controller = RobotControllerThread(
            SimpleNamespace(get_latest_state=snapshot))
        controller.pause_on_sensor_outage = True
        controller._running.set()
        controller.stop_chassis = lambda: None
        controller.recovery_callback = lambda phase, detail: (
            paused.set() if phase == "paused" else None)
        outcome = []

        real_setting = setting
        def short_grace(name):
            if name == "navigation.sensor_recovery_timeout_sec":
                return 0.02
            return real_setting(name)

        with patch("src.robot_controller.setting", side_effect=short_grace):
            worker = threading.Thread(
                target=lambda: outcome.append(controller.align_turn_heading(90.0)))
            worker.start()
            self.assertTrue(paused.wait(1.0))
            self.assertTrue(worker.is_alive())
            release.set()
            worker.join(1.0)
        self.assertFalse(worker.is_alive())
        self.assertEqual(outcome[0].yaw, 90.0)

    def test_camera_codec_stub_is_rejected_before_motion(self):
        stub = SimpleNamespace(_camera_codec_unavailable=True)
        with patch("src.sdk_connection.load_robot_sdk"), patch.dict(sys.modules,
                                                                     {"libmedia_codec": stub}):
            with self.assertRaisesRegex(RuntimeError, "libmedia_codec is missing"):
                require_camera_codec()

    def test_camera_must_decode_a_frame_before_motion(self):
        frame = object()
        frames = iter((queue.Empty(), frame))

        def read(**kwargs):
            result = next(frames)
            if isinstance(result, Exception):
                raise result
            return result

        camera = SimpleNamespace(read_cv2_image=read)
        self.assertIs(_wait_for_first_camera_frame(camera, 1.0), frame)
        camera.read_cv2_image = lambda **kwargs: None
        with self.assertRaisesRegex(RuntimeError, "no decoded video frame"):
            _wait_for_first_camera_frame(camera, 0.01)

    def test_straight_second_pass_motion_skips_stationary_yaw_correction(self):
        from src.slam_hardware import HardwareBackend

        calls = []
        pose = SimpleNamespace(yaw=0.0, pos_x=0.0, pos_y=0.0)
        controller = SimpleNamespace(
            target_heading_deg=0.0, wall_pid=SimpleNamespace(front_target_mm=250),
            navigate_single_grid_step=lambda: {"completed": True, "reason": None})
        backend = object.__new__(HardwareBackend)
        backend.robot = SimpleNamespace()
        backend.controller = controller
        backend.hub = SimpleNamespace(get_latest_state=lambda: pose)
        backend.heading = 0
        backend.event_log = []
        backend.hold_front_for_motion = lambda: calls.append("gimbal_front")
        backend.select_robot_mode = lambda mode, phase: calls.append("mode")
        backend.align_heading = lambda: calls.append("align")
        backend.ensure_gimbal_front = lambda *args, **kwargs: None
        backend.fresh = lambda *args: True
        backend.aim = lambda yaw: None
        backend.sample = lambda yaw, after: 1.0
        backend.prepare_stationary_scan = lambda: pose
        with patch("src.slam_hardware.load_robot_sdk",
                   return_value=SimpleNamespace(CHASSIS_LEAD="follow")):
            HardwareBackend.move(backend, 0, align_if_straight=False)
        self.assertNotIn("align", calls)

    def test_target_wall_inspection_does_not_turn_when_already_facing_wall(self):
        calls = []
        backend = SimpleNamespace(
            heading=0,
            select_robot_mode=lambda mode, phase: calls.append(("mode", mode)),
            face=lambda direction: calls.append(("face", direction)),
            prepare_stationary_scan=lambda: (calls.append(("wheel_stop",))
                                             or SimpleNamespace(yaw=0.0)),
        )
        modes = SimpleNamespace(CHASSIS_LEAD="follow", FREE="free")
        _prepare_target_wall(backend, 0, modes)
        self.assertEqual(calls, [("wheel_stop",), ("mode", "free")])

    def test_target_wall_inspection_turns_only_for_another_direction(self):
        calls = []
        backend = SimpleNamespace(
            heading=0,
            select_robot_mode=lambda mode, phase: calls.append(("mode", mode)),
            face=lambda direction: calls.append(("face", direction)),
            prepare_stationary_scan=lambda: (calls.append(("wheel_stop",))
                                             or SimpleNamespace(yaw=90.0)),
        )
        modes = SimpleNamespace(CHASSIS_LEAD="follow", FREE="free")
        _prepare_target_wall(backend, 1, modes)
        self.assertEqual(calls, [("mode", "follow"), ("face", 1),
                                 ("wheel_stop",), ("mode", "free")])

    def test_shortest_route_visits_all_targets_over_confirmed_edges(self):
        cells = {(row, column) for row in range(2) for column in range(3)}
        graph = {cell: set() for cell in cells}
        for row, column in cells:
            for other in ((row + 1, column), (row, column + 1)):
                if other in cells:
                    graph[row, column].add(other)
                    graph[other].add((row, column))
        route = shortest_target_route(graph, (0, 0), {(0, 2), (1, 0)})
        self.assertEqual(len(route) - 1, 4)
        self.assertIn((0, 2), route)
        self.assertIn((1, 0), route)
        self.assertTrue(all(b in graph[a] for a, b in zip(route, route[1:])))

    def test_plan_filters_class_and_keeps_wall_and_trajectory(self):
        map_data = {
            "status": "completed",
            "method": "grid_constrained_range_odometry_slam_frontier_bfs",
            "cell_size_m": setting("navigation.grid_size_m"),
            "map_info": {"rows": 2, "columns": 2},
            "start_cell": [0, 0],
            "visited": [[0, 0], [1, 0], [1, 1]],
            "trajectory": [{"cell": cell} for cell in ([0, 0], [1, 0], [1, 1])],
            "edges": [
                {"cells": [[0, 0], [1, 0]], "wall": False},
                {"cells": [[1, 0], [1, 1]], "wall": False},
                {"cells": [[0, 0], [0, 1]], "wall": True},
                {"cells": [[0, 1], [1, 1]], "wall": True},
            ],
            "signs": [
                {"cell": [0, 0], "direction": 1, "color": "Red", "shape": "Circle"},
                {"cell": [1, 1], "direction": 2, "color": "Blue", "shape": "Square"},
                {"cell": [1, 1], "direction": 2, "color": "Red", "shape": "Circle"},
            ],
        }
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "explored_map.json"
            path.write_text(json.dumps(map_data), encoding="utf-8")
            data, graph, signs = load_explore_map(path)
            self.assertEqual(len(data["trajectory"]), 3)
            self.assertEqual(len(signs), 3)
            self.assertNotIn((0, 1), graph[(0, 0)])
            plan = make_plan(path, {("Blue", "Square")})
            self.assertEqual(plan["route"], [[0, 0], [1, 0], [1, 1]])
            self.assertEqual(len(plan["targets"]), 1)
            individual = make_plan(path, set(), {sign_key(signs[0])})
            self.assertEqual(individual["route"], [[0, 0]])
            self.assertEqual(individual["targets"][0]["color"], "Red")
            grouped = make_plan(path, {("Red", "Circle")})
            self.assertEqual(len(grouped["targets"]), 2)
            with self.assertRaises(ValueError):
                make_plan(path, set(), {((0, 0), 2, "Red", "Circle")})


if __name__ == "__main__":
    unittest.main()
