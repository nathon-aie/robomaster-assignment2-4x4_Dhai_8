"""Validate saved Navigation plans and generate GUI-only playback events."""

import json
from pathlib import Path

from .settings import PROJECT_ROOT
from .target_navigation import DIRECTIONS, load_explore_map, sign_key


def load_saved_plan(path, source_map=None):
    """Return a saved plan and its Explore map after checking every planned edge."""
    path = Path(path)
    plan = json.loads(path.read_text(encoding="utf-8"))
    source = Path(source_map) if source_map else Path(plan["source_map"])
    if not source.is_absolute():
        source = (path.parent / source).resolve()
    # Existing plans can refer to the checkout that was consolidated into this
    # project. Recover relocated telemetry paths without rewriting run history.
    if source_map is None and not source.is_file() and "telemetry_logs" in source.parts:
        offset = source.parts.index("telemetry_logs")
        relocated = PROJECT_ROOT.joinpath(*source.parts[offset:])
        if relocated.is_file():
            source = relocated
    data, graph, signs = load_explore_map(source)
    route = [tuple(cell) for cell in plan["route"]]
    if (not route or route[0] != tuple(data["start_cell"])
            or any(cell not in graph for cell in route)
            or any(b not in graph[a] for a, b in zip(route, route[1:]))):
        raise ValueError("Saved route does not match the Explore map's open edges")
    directions = [DIRECTIONS.index((b[0] - a[0], b[1] - a[1]))
                  for a, b in zip(route, route[1:])]
    if plan["directions"] != directions or plan["step_count"] != len(directions):
        raise ValueError("Saved route directions or step count are inconsistent")
    available = {sign_key(sign) for sign in signs}
    targets = {sign_key(sign) for sign in plan["targets"]}
    if (not targets or not targets <= available
            or not {key[0] for key in targets} <= set(route)
            or {tuple(cell) for cell in plan["target_cells"]} != {key[0] for key in targets}):
        raise ValueError("Saved targets do not match signs on the Explore map")
    if plan["cell_size_m"] != data["cell_size_m"]:
        raise ValueError("Saved plan uses a different Grid size")
    plan["source_map"] = str(source.resolve())
    return plan, source


def simulation_frames(plan, slices=8, scan_pitch=-20.0):
    """Yield (delay_ms, kind, ...) for a visual rehearsal; no robot calls."""
    route = plan["route"]
    targets_by_cell = {}
    for target in plan["targets"]:
        targets_by_cell.setdefault(tuple(target["cell"]), []).append(target)
    heading = plan["directions"][0] if plan["directions"] else 0
    yield (0, "position", route[0][0], route[0][1], heading * 90)
    for index, cell in enumerate(route):
        if index:
            previous = route[index - 1]
            heading = plan["directions"][index - 1]
            yield (0, "moving", previous, cell, index)
            for part in range(1, slices + 1):
                fraction = part / slices
                yield (75, "position",
                       previous[0] + (cell[0] - previous[0]) * fraction,
                       previous[1] + (cell[1] - previous[1]) * fraction,
                       heading * 90)
            yield (0, "arrive", cell, index)
        targets = targets_by_cell.pop(tuple(cell), [])
        for direction in sorted({target["direction"] for target in targets}):
            expected = [target for target in targets if target["direction"] == direction]
            if heading != direction:
                heading = direction
                yield (300, "face", cell, direction)
            yield (0, "scan", cell, direction, expected)
            for part in range(1, 5):
                yield (60, "gimbal", cell, direction, 0.0,
                       scan_pitch * part / 4)
            gimbal_yaw = 0.0
            for target_yaw in (0.0, -18.0, 18.0):
                parts = max(1, round(abs(target_yaw - gimbal_yaw) / 3))
                for part in range(1, parts + 1):
                    angle = gimbal_yaw + (target_yaw - gimbal_yaw) * part / parts
                    yield (60, "gimbal", cell, direction, angle, scan_pitch)
                yield (300, "scan_hold", cell, direction, target_yaw, scan_pitch)
                gimbal_yaw = target_yaw
            yield (250, "targets", expected)
            for part in range(1, 7):
                yield (60, "gimbal", cell, direction,
                       gimbal_yaw * (1 - part / 6),
                       scan_pitch * (1 - part / 6))
            yield (0, "gimbal_end")
    yield (0, "complete")
