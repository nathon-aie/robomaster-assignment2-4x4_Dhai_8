"""Regression tests for periodic chassis heading alignment."""

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from src.grid_slam import FrontierExplorer
from src.slam_hardware import HardwareBackend


class HeadingAlignmentTest(unittest.TestCase):
    def test_hardware_aligns_to_current_heading_without_relative_turn(self):
        backend = HardwareBackend.__new__(HardwareBackend)
        backend.heading = 1
        backend.controller = Mock()
        backend.ensure_running = Mock()
        backend.prepare_stationary_scan = Mock(return_value=SimpleNamespace(yaw=90.2))

        self.assertEqual(backend.align_current_heading(5), 90.2)
        backend.controller.align_turn_heading.assert_called_once_with(90, tolerance_deg=5)
        backend.controller.turn_to_relative.assert_not_called()
        self.assertEqual(backend.controller.target_heading_deg, 90)

    def test_periodic_alignment_keeps_east_heading(self):
        backend = SimpleNamespace(
            move=Mock(return_value=((0.0, 0.6), 90.0)),
            align_current_heading=Mock(return_value=90.0),
            stop=Mock(),
        )
        with tempfile.TemporaryDirectory() as temp:
            explorer = FrontierExplorer(backend, Path(temp) / "map.json")
            explorer.slam.map.visited.update({(0, 0), (0, 1), (0, 2)})
            explorer.nearest_frontier_route = Mock(side_effect=[
                [(0, 0), (0, 1)], [(0, 1), (0, 2)], None,
            ])
            explorer.run()

        self.assertEqual(explorer.status, "completed")
        self.assertEqual(explorer.slam.heading, 1)
        self.assertEqual(explorer.slam.pose[2], 90.0)
        backend.align_current_heading.assert_called_once_with(5)
        self.assertEqual(
            [(event["type"], event["heading"])
             for event in explorer.slam.events
             if event["type"] in ("heading_align_start", "heading_align")],
            [("heading_align_start", 1), ("heading_align", 1)],
        )


if __name__ == "__main__":
    unittest.main()
