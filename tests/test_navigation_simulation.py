import json
import tempfile
import unittest
from pathlib import Path

from src.navigation_simulation import load_saved_plan, simulation_frames
from src.settings import get as setting
from src.target_navigation import make_plan, sign_key


class NavigationSimulationTests(unittest.TestCase):
    def test_saved_plan_replays_selected_target_without_robot_commands(self):
        explored = {
            "status": "completed",
            "method": "grid_constrained_range_odometry_slam_frontier_bfs",
            "cell_size_m": setting("navigation.grid_size_m"),
            "map_info": {"rows": 2, "columns": 1},
            "start_cell": [0, 0],
            "visited": [[0, 0], [1, 0]],
            "trajectory": [{"cell": [0, 0]}, {"cell": [1, 0]}],
            "edges": [{"cells": [[0, 0], [1, 0]], "wall": False}],
            "signs": [{"cell": [1, 0], "direction": 0,
                       "color": "Blue", "shape": "Circle"}],
        }
        with tempfile.TemporaryDirectory() as folder:
            map_path = Path(folder) / "explored_map.json"
            map_path.write_text(json.dumps(explored), encoding="utf-8")
            plan = make_plan(map_path, set(), {sign_key(explored["signs"][0])})
            plan_path = Path(folder) / "navigation_plan.json"
            plan_path.write_text(json.dumps(plan), encoding="utf-8")
            loaded, source = load_saved_plan(plan_path)
            self.assertEqual(source, map_path)
            self.assertEqual(loaded["route"], [[0, 0], [1, 0]])
            events = list(simulation_frames(loaded, slices=2))
            self.assertEqual([item[1] for item in events].count("position"), 3)
            self.assertIn("scan", [item[1] for item in events])
            self.assertIn("targets", [item[1] for item in events])
            self.assertEqual([item[4] for item in events if item[1] == "scan_hold"],
                             [0.0, -18.0, 18.0])
            self.assertEqual([item[5] for item in events if item[1] == "scan_hold"],
                             [-20.0, -20.0, -20.0])
            self.assertEqual(events[-2][1], "gimbal_end")
            self.assertEqual(events[-1][1], "complete")

            plan["route"] = [[0, 0], [1, 0], [0, 0], [0, 0]]
            plan_path.write_text(json.dumps(plan), encoding="utf-8")
            with self.assertRaises(ValueError):
                load_saved_plan(plan_path)


if __name__ == "__main__":
    unittest.main()
