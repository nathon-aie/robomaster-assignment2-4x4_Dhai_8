"""Shutdown stops workers and drains SDK chassis timers."""

import unittest
from unittest.mock import Mock, patch

from src.robot_system import RobotSystem


class ThreadCleanupTest(unittest.TestCase):
    def test_shutdown_joins_workers_and_cancels_chassis_timer(self):
        system = RobotSystem.__new__(RobotSystem)
        system.robot = Mock()
        system.thread_1_sensor = Mock()
        system.thread_2_controller = Mock()
        system.thread_1_sensor.is_alive.return_value = True
        system.thread_2_controller.is_alive.return_value = True

        with patch("src.robot_system.cancel_chassis_speed_timer") as cancel_timer:
            system.shutdown(save_telemetry=False)

        system.thread_2_controller.stop_running.assert_called_once_with()
        system.thread_1_sensor.stop_collecting.assert_called_once_with()
        system.thread_2_controller.join.assert_called_once()
        system.thread_1_sensor.join.assert_called_once()
        cancel_timer.assert_called_once_with(system.robot.chassis)
        system.robot.close.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
