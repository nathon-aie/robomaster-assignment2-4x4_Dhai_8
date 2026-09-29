"""Heading correction follows yaw feedback instead of trusting a fixed speed sign."""

import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from src.robot_controller import RobotControllerThread


class SimulatedYaw:
    def __init__(self, physical_sign):
        self.now = 100.0
        self.last_sample = self.now
        self.yaw = -4.0
        self.speed = 0.0
        self.physical_sign = physical_sign
        self.commands = []

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds

    def drive_speed(self, **kwargs):
        self.speed = kwargs["z"]
        self.commands.append(self.speed)
        return True

    def get_latest_state(self):
        self.yaw += self.speed * self.physical_sign * (self.now - self.last_sample)
        self.last_sample = self.now
        return SimpleNamespace(yaw=self.yaw, attitude_received_at=self.now)


class YawCorrectionTest(unittest.TestCase):
    def make_controller(self, plant):
        controller = RobotControllerThread.__new__(RobotControllerThread)
        controller.robot = SimpleNamespace(chassis=plant)
        controller.sensor_hub = plant
        controller.yaw_speed_command_sign = 1
        controller._running = threading.Event()
        controller._running.set()
        return controller

    def test_reverses_speed_sign_if_robot_turns_away_from_zero(self):
        plant = SimulatedYaw(physical_sign=-1)
        controller = self.make_controller(plant)

        with (patch("src.robot_controller.time.monotonic", plant.monotonic),
              patch("src.robot_controller.time.sleep", plant.sleep)):
            result = controller.align_turn_heading(0.0, tolerance_deg=1.0)

        self.assertLessEqual(abs(result.yaw), 1.0)
        self.assertEqual(controller.yaw_speed_command_sign, -1)
        self.assertTrue(any(speed > 0 for speed in plant.commands))
        self.assertTrue(any(speed < 0 for speed in plant.commands))

    def test_keeps_speed_sign_when_error_decreases(self):
        plant = SimulatedYaw(physical_sign=1)
        controller = self.make_controller(plant)

        with (patch("src.robot_controller.time.monotonic", plant.monotonic),
              patch("src.robot_controller.time.sleep", plant.sleep)):
            result = controller.align_turn_heading(0.0, tolerance_deg=1.0)

        self.assertLessEqual(abs(result.yaw), 1.0)
        self.assertEqual(controller.yaw_speed_command_sign, 1)
        self.assertFalse(any(speed < 0 for speed in plant.commands))


if __name__ == "__main__":
    unittest.main()
