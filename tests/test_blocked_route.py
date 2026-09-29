"""A mapped open edge is closed and replanned when fresh ToF sees a wall."""

import tempfile
import unittest
from pathlib import Path

from src.grid_slam import BlockedCellError, DFSExplorer


class BlockedRouteTest(unittest.TestCase):
    def test_preflight_obstacle_replans_instead_of_failing_mission(self):
        class Backend:
            scan_headings = {0: 0, 1: 0, 2: 0, 3: 0}

            def scan(self):
                return {0: 1.0, 1: 0.2, 2: 0.2, 3: 0.2}, 0, 0.0

            def move(self, direction):
                raise BlockedCellError(233, 240)

            def stop(self):
                pass

        with tempfile.TemporaryDirectory() as directory:
            explorer = DFSExplorer(Backend(), Path(directory) / "map.json")
            self.assertTrue(explorer.run())

        self.assertEqual(explorer.status, "completed")
        self.assertEqual(explorer.moves, 0)
        self.assertTrue(explorer.slam.map.wall((0, 0), 0))
        self.assertTrue(any(event.get("type") == "front_obstacle"
                            for event in explorer.slam.events))


if __name__ == "__main__":
    unittest.main()
