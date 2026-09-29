"""Incremental grid wall mapping, range/odometry localization, and frontier exploration.

Cell coordinates: (row, column); rows along initial forward, columns to the right.
No environment map is supplied to the explorer. Unknown edges are never traversed.
"""
import json
import math
import time
from collections import deque
from pathlib import Path
from .settings import get as setting, map_geometry

DIRECTIONS = ((1, 0), (0, 1), (-1, 0), (0, -1))
NAMES = ('N', 'E', 'S', 'W')


def neighbor(cell, direction):
    drow, dcolumn = DIRECTIONS[direction]
    return cell[0] + drow, cell[1] + dcolumn


def wrap(deg):
    return (deg + 180) % 360 - 180


class GridMap:
    def __init__(self):
        self.edges = {}  # Canonical undirected edge -> wall present/absent.
        self.visited = set()
        self.observations = []

    @staticmethod
    def edge(cell, direction):
        return tuple(sorted((tuple(cell), neighbor(cell, direction))))

    def wall(self, cell, direction):
        return self.edges.get(self.edge(cell, direction))

    def observe(self, cell, direction, wall):
        key = self.edge(cell, direction)
        previous = self.edges.get(key)
        if previous is not None and previous != wall:
            raise RuntimeError('Contradictory wall observation at {} {}'.format(cell, NAMES[direction]))
        self.edges[key] = wall

    def expected_range(self, cell, direction, pose, sensor_offset):
        """Range to the first mapped wall; stop at unknown edge rather than inventing it."""
        current = cell
        drow, dcolumn = DIRECTIONS[direction]
        size = setting('navigation.grid_size_m')
        for _ in range(int(setting('sensors.tof_filter.max_valid') / (size * 1000)) + 2):
            wall = self.wall(current, direction)
            if wall is None:
                return None
            if wall:
                axis = 0 if drow else 1
                sign = drow or dcolumn
                boundary = (current[axis] + sign * 0.5) * size
                inner_face = boundary - sign * setting('slam.wall_thickness_m') / 2
                return sign * (inner_face - pose[axis] - sensor_offset[axis])
            current = neighbor(current, direction)
        return None


class GridSLAM:
    """Grid-constrained SLAM with diagonal covariance and known-wall range updates.

Discrete cell identity is gated by odometry. Revisits reuse existing wall landmarks
and correct the continuous pose; this is not unrestricted metric pose-graph SLAM.
"""
    def __init__(self):
        self.map = GridMap()
        self.rows, self.columns, self.start_cell = map_geometry()
        self.cell = self.start_cell
        self.heading = 0
        self.start_pose = [v * setting('navigation.grid_size_m') for v in self.start_cell] + [0.0]
        self.pose = list(self.start_pose)
        self.variance = [setting('slam.initial_variance_m2')] * 2
        self.trajectory = []
        self.matches = 0
        self.events = []

    def contains(self, cell):
        row, column = cell
        return 0 <= row < self.rows and 0 <= column < self.columns

    def offset(self, heading, direction):
        angle = math.radians(heading * 90)
        pivot_x_m = setting('gimbal.pivot_x_m')
        pivot_y_m = setting('gimbal.pivot_y_m')
        beam = setting('gimbal.beam_offset_m')
        drow, dcolumn = DIRECTIONS[direction]
        return (pivot_x_m * math.cos(angle) - pivot_y_m * math.sin(angle) + beam * drow,
                pivot_x_m * math.sin(angle) + pivot_y_m * math.cos(angle) + beam * dcolumn)

    def predict(self, target, displacement, yaw):
        if not self.contains(target):
            raise RuntimeError('Destination is outside configured map bounds')
        if not all(math.isfinite(v) for v in (*displacement, yaw)):
            raise RuntimeError('Non-finite odometry')
        expected = [v * setting('navigation.grid_size_m') for v in target]
        predicted = [self.pose[i] + displacement[i] for i in (0, 1)]
        error = math.hypot(*(predicted[i] - expected[i] for i in (0, 1)))
        if error > setting('slam.localization_gate_m'):
            # The motion controller already verifies cell travel and front ToF.
            # Accumulated odometry error is diagnostic, not a mission abort.
            self.events.append({'timestamp': time.time(), 'type': 'odometry_mismatch',
                                'cell': list(target), 'predicted_position_m': predicted[:],
                                'expected_position_m': expected, 'error_m': error})
        revisiting = target in self.map.visited
        self.cell = target
        self.pose = predicted + [yaw]
        self.variance = [v + setting('slam.motion_variance_m2') for v in self.variance]
        # New cells get a corrected trajectory point after scanning. Revisits
        # skip the scan, so retain their odometry point here.
        if revisiting:
            self.trajectory.append({'cell': list(self.cell), 'pose': list(self.pose),
                                    'variance_m2': list(self.variance)})

    def update(self, ranges, heading, yaw, scan_headings=None):
        if not math.isfinite(yaw):
            raise RuntimeError('Invalid yaw')
        if any(not math.isfinite(v) or v <= 0 for v in ranges.values()):
            raise RuntimeError('Invalid scan range')
        if scan_headings is None:
            scan_headings = {d: heading for d in ranges}
        if set(scan_headings) != set(ranges) or any(type(v) is not int or v not in range(4) for v in scan_headings.values()):
            raise RuntimeError("Invalid per-ray chassis headings")
        self.heading = heading
        self.pose[2] = yaw
        # Match only walls already in the map, before inserting this scan.
        # Freeze predictions for this batch so opposing observations are consistent.
        original = list(self.pose)
        corrections = [[], []]
        for d, distance in ranges.items():
            offset = self.offset(scan_headings[d], d)
            expected = self.map.expected_range(self.cell, d, original, offset)
            if expected is None:
                continue
            residual = distance - expected
            if abs(residual) > setting('slam.localization_gate_m'):
                self.events.append({'timestamp': time.time(), 'type': 'range_mismatch',
                                    'direction': NAMES[d], 'residual_m': residual})
                continue
            drow, dcolumn = DIRECTIONS[d]
            axis, sign = (0, drow) if drow else (1, dcolumn)
            corrections[axis].append(-sign * residual)
        for axis in (0, 1):
            if corrections[axis]:
                residual = sum(corrections[axis]) / len(corrections[axis])
                noise = setting('slam.range_variance_m2') / len(corrections[axis])
                gain = self.variance[axis] / (self.variance[axis] + noise)
                self.pose[axis] += gain * residual
                self.variance[axis] *= 1 - gain
                self.matches += len(corrections[axis])
        if max(math.sqrt(v) for v in self.variance) > setting('slam.max_pose_std_m'):
            raise RuntimeError('Pose uncertainty exceeded limit')
        size = setting('navigation.grid_size_m')
        observations = []
        for d, distance in ranges.items():
            drow, dcolumn = DIRECTIONS[d]
            axis, sign = (0, drow) if drow else (1, dcolumn)
            offset = self.offset(scan_headings[d], d)
            # ToF always returns a distance. Only a return at the nearby cell
            # boundary means this edge is a wall; a farther return is beyond it.
            wall_limit = (size / 2 - sign * offset[axis]
                          - setting('slam.wall_thickness_m') / 2
                          + setting('slam.wall_margin_m'))
            wall = distance <= wall_limit
            observations.append((d, wall))
        for d, wall in observations:
            previous = self.map.wall(self.cell, d)
            if previous is not None and previous != wall:
                edge = self.map.edge(self.cell, d)
                crossed = any(e.get('type') == 'move' and
                              tuple(sorted((tuple(e['from']), tuple(e['to'])))) == edge
                              for e in self.events)
                self.events.append({'timestamp': time.time(), 'type': 'wall_mismatch',
                                    'cell': list(self.cell), 'direction': NAMES[d],
                                    'previous_wall': previous, 'observed_wall': wall,
                                    'crossed': crossed})
                if wall and not crossed:
                    # A short return closes an edge previously thought open.
                    self.map.edges[edge] = True
                continue
            self.map.observe(self.cell, d, wall)
            if not wall and not self.contains(neighbor(self.cell, d)):
                self.events.append({'timestamp': time.time(), 'type': 'boundary_open',
                                    'cell': list(self.cell), 'direction': NAMES[d],
                                    'range_m': ranges[d], 'traversable': False})
        self.map.visited.add(self.cell)
        self.events.append({'timestamp': time.time(), 'type': 'scan', 'cell': list(self.cell),
                            'pose': list(self.pose), 'ranges_m': dict(ranges), 'scan_headings': dict(scan_headings)})
        self.map.observations.append({'cell': list(self.cell), 'heading': heading,
                                      'ranges_m': {NAMES[d]: v for d, v in ranges.items()},
                                      'scan_headings': {NAMES[d]: v for d, v in scan_headings.items()}})
        self.trajectory.append({'cell': list(self.cell), 'pose': list(self.pose),
                                'variance_m2': list(self.variance)})

    def export(self, output, status, error=None):
        path = Path(output)
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {'schema': 1, 'method': 'grid_constrained_range_odometry_slam_frontier_bfs',
                'status': status, 'error': error, 'cell_size_m': setting('navigation.grid_size_m'),
                'coordinates': 'pose in metres: +x initial forward, +y right; yaw clockwise degrees',
                'cell_coordinates': '[row, column]; row increases forward, column increases right',
                'map_info': {'rows': self.rows, 'columns': self.columns},
                'start_cell': list(self.start_cell), 'start_pose': self.start_pose,
                'cell': list(self.cell), 'pose': self.pose, 'variance_m2': self.variance,
                'events': self.events,
                'landmark_matches': self.matches, 'visited': [list(c) for c in sorted(self.map.visited)],
                'edges': [{'cells': [list(c) for c in key], 'wall': value,
                           'boundary_blocked': not all(self.contains(c) for c in key)}
                          for key, value in sorted(self.map.edges.items())],
                'trajectory': self.trajectory, 'scans': self.map.observations}
        temp = path.with_suffix(path.suffix + '.tmp')
        temp.write_text(json.dumps(data, indent=2) + '\n', encoding='utf-8')
        temp.replace(path)


class FrontierExplorer:
    """Visit the nearest reachable frontier through confirmed open grid edges."""
    def __init__(self, backend, output):
        self.backend = backend
        self.output = output
        self.slam = GridSLAM()
        if hasattr(backend, "event_log"):
            backend.event_log = self.slam.events
        self.moves = 0
        self.status = 'not_started'
        self.error = None

    def nearest_frontier_route(self):
        """Return a shortest cell route to an unvisited neighbor, or None.

        BFS only crosses confirmed open edges. Visited cells can be used as
        transit; an unvisited cell is the terminal frontier, never a shortcut.
        Direction order breaks ties between routes of equal length.
        """
        start = self.slam.cell
        visited = self.slam.map.visited
        queue = deque([start])
        previous = {start: None}
        while queue:
            cell = queue.popleft()
            for direction in setting('slam.direction_order'):
                target = neighbor(cell, direction)
                if (not self.slam.contains(target)
                        or self.slam.map.wall(cell, direction) is not False):
                    continue
                if target not in visited:
                    route = [target]
                    while cell is not None:
                        route.append(cell)
                        cell = previous[cell]
                    route.reverse()
                    return route
                if target not in previous:
                    previous[target] = cell
                    queue.append(target)
        return None

    def run(self):
        self.status = 'running'
        try:
            while True:
                cell = self.slam.cell
                if cell in self.slam.map.visited:
                    # Every edge was mapped on first arrival. The motion
                    # backend still reads fresh front ToF before each step.
                    self.slam.events.append({'timestamp': time.time(), 'type': 'scan_skipped',
                                             'cell': list(cell), 'reason': 'previously_scanned'})
                else:
                    ranges, heading, yaw = self.backend.scan()
                    rear = (heading + 2) % 4
                    required = {(heading + r) % 4 for r in (0, 3, 1)}
                    full_scan = set(ranges) == set(range(4))
                    known_rear_scan = (self.moves > 0 and set(ranges) == required
                                       and self.slam.map.wall(cell, rear) is False)
                    if not (full_scan or known_rear_scan):
                        raise RuntimeError('Incomplete scan; unknown directions cannot be traversed')
                    self.slam.update(ranges, heading, yaw, getattr(self.backend, "scan_headings", None))
                self.slam.export(self.output, self.status)
                # Replan after every arrival. This can use a mapped cross-link
                # instead of retracing the path by which a cell was discovered.
                route = self.nearest_frontier_route()
                if route is None:
                    frontier_exists = any(
                        self.slam.contains(neighbor(seen, d))
                        and self.slam.map.wall(seen, d) is False
                        and neighbor(seen, d) not in self.slam.map.visited
                        for seen in self.slam.map.visited for d in range(4))
                    if frontier_exists:
                        raise RuntimeError('Mapped frontier is unreachable through confirmed open edges')
                    self.status = 'completed'
                    break
                if len(self.slam.map.visited) >= setting('slam.max_cells'):
                    self.status = 'limit_reached'
                    break
                if self.moves >= setting('slam.max_moves'):
                    self.status = 'limit_reached'
                    break
                target = route[1]
                direction = next(d for d in range(4) if neighbor(cell, d) == target)
                backtrack = target in self.slam.map.visited
                self.slam.events.append({'timestamp': time.time(), 'type': 'frontier_plan',
                                         'frontier': list(route[-1]),
                                         'route': [list(step) for step in route]})
                self.slam.events.append({'timestamp': time.time(), 'type': 'motion_start',
                    'step': self.moves + 1, 'from': list(cell), 'to': list(target),
                    'direction': direction, 'backtrack': backtrack})
                displacement, yaw = self.backend.move(direction)
                self.slam.events.append({'timestamp': time.time(), 'type': 'move',
                    'from': list(cell), 'to': list(target), 'direction': direction,
                    'backtrack': backtrack, 'odometry_delta_m': list(displacement), 'yaw': yaw})
                self.slam.predict(target, displacement, yaw)
                self.moves += 1
        except KeyboardInterrupt:
            self.status = 'interrupted'
        except Exception as exc:
            self.status = 'failed'
            self.error = str(exc)
        finally:
            self.slam.events.append({'timestamp': time.time(), 'type': 'finish',
                                     'status': self.status, 'error': self.error})
            try:
                self.backend.stop()
            finally:
                self.slam.export(self.output, self.status, self.error)
        return self.status == 'completed'
