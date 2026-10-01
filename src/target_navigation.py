"""Second pass through selected signs on an Explore map."""

import json
import heapq
import queue
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

from .settings import get as setting, project_path


COLORS = ("Red", "Green", "Blue", "Yellow")
SHAPES = ("Circle", "Square", "Vertical_Rect", "Horizontal_Rect")
DIRECTIONS = ((1, 0), (0, 1), (-1, 0), (0, -1))
NAMES = ("N", "E", "S", "W")


def sign_key(sign):
    """Stable identity for one sign, including its wall and visual class."""
    direction = sign["direction"]
    if direction in NAMES:
        direction = NAMES.index(direction)
    return (tuple(sign["cell"]), direction, sign["color"], sign["shape"])


def _cell(value, rows, columns):
    if (not isinstance(value, (list, tuple)) or len(value) != 2
            or any(type(part) is not int for part in value)
            or not 0 <= value[0] < rows or not 0 <= value[1] < columns):
        raise ValueError("Invalid cell in Explore map: {!r}".format(value))
    return tuple(value)


def _edge(a, b):
    return frozenset((a, b))


def load_explore_map(path):
    """Validate mapped cells and confirmed edges, including partial Explore maps."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if (data.get("status") not in ("running", "completed", "limit_reached",
                                    "interrupted", "failed")
            or data.get("method") != "grid_constrained_range_odometry_slam_frontier_bfs"):
        raise ValueError("Select an Explore map with scanned cells, not a plan or test map")
    info = data.get("map_info", {})
    rows, columns = info.get("rows"), info.get("columns")
    if type(rows) is not int or type(columns) is not int or rows < 1 or columns < 1:
        raise ValueError("Invalid map dimensions")
    if data.get("cell_size_m") != setting("navigation.grid_size_m"):
        raise ValueError("Explore cell size differs from current navigation settings")
    start = _cell(data.get("start_cell"), rows, columns)
    visited = {_cell(cell, rows, columns) for cell in data.get("visited", [])}
    if start not in visited:
        raise ValueError("Start cell was not visited")
    trajectory = [_cell(point.get("cell"), rows, columns)
                  for point in data.get("trajectory", [])]
    if not trajectory or trajectory[0] != start or set(trajectory) != visited:
        raise ValueError("Explore trajectory is missing or incomplete")
    walls = {}
    for item in data.get("edges", []):
        pair = item.get("cells", [])
        if (len(pair) != 2 or any(not isinstance(value, list) or len(value) != 2
                                  or any(type(part) is not int for part in value)
                                  for value in pair)):
            raise ValueError("Invalid wall edge")
        a, b = (tuple(value) for value in pair)
        if abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1 or type(item.get("wall")) is not bool:
            raise ValueError("Invalid wall edge")
        key = _edge(a, b)
        if key in walls and walls[key] != item["wall"]:
            raise ValueError("Conflicting wall observations")
        walls[key] = item["wall"]
    graph = {cell: set() for cell in visited}
    for cell in visited:
        for dr, dc in DIRECTIONS:
            other = cell[0] + dr, cell[1] + dc
            if other in visited and walls.get(_edge(cell, other)) is False:
                graph[cell].add(other)
    if any(b not in graph[a] for a, b in zip(trajectory, trajectory[1:])):
        raise ValueError("Explore trajectory crosses an unknown or blocked edge")
    signs = []
    for item in data.get("signs", []):
        cell = _cell(item.get("cell"), rows, columns)
        direction = item.get("direction")
        direction = NAMES.index(direction) if direction in NAMES else direction
        color, shape = item.get("color"), item.get("shape")
        if (cell not in visited or type(direction) is not int or direction not in range(4)
                or color not in COLORS or shape not in SHAPES):
            raise ValueError("Invalid sign entry: {!r}".format(item))
        dr, dc = DIRECTIONS[direction]
        other = cell[0] + dr, cell[1] + dc
        if 0 <= other[0] < rows and 0 <= other[1] < columns and walls.get(_edge(cell, other)) is not True:
            raise ValueError("Sign at {} {} lacks a confirmed wall".format(cell, NAMES[direction]))
        signs.append(dict(item, cell=list(cell), direction=direction))
    return data, graph, signs


def shortest_target_route(graph, start, goals):
    """A* over (cell, visited target mask), minimizing total grid steps."""
    goals = sorted(set(goals))
    index = {cell: i for i, cell in enumerate(goals)}
    full = (1 << len(goals)) - 1
    first = (start, 1 << index[start] if start in index else 0)
    distances = {}
    for source in graph:
        queue = deque([source])
        distance = {source: 0}
        while queue:
            current = queue.popleft()
            for other in graph[current]:
                if other not in distance:
                    distance[other] = distance[current] + 1
                    queue.append(other)
        distances[source] = distance
    if any(goal not in distances[start] for goal in goals):
        raise ValueError("A selected sign cell cannot be reached through confirmed open edges")
    mst_cache = {0: 0}

    def mst(mask):
        if mask not in mst_cache:
            remaining = [cell for cell in goals if mask & (1 << index[cell])]
            attached = {remaining[0]}
            cost = 0
            while len(attached) < len(remaining):
                steps, cell = min((distances[a][b], b) for a in attached
                                  for b in remaining if b not in attached)
                cost += steps
                attached.add(cell)
            mst_cache[mask] = cost
        return mst_cache[mask]

    def estimate(cell, mask):
        remaining = full ^ mask
        if not remaining:
            return 0
        return min(distances[cell][goal] for goal in goals
                   if remaining & (1 << index[goal])) + mst(remaining)

    best = {first: 0}
    parent = {}
    serial = 0
    heap = [(estimate(*first), 0, first)]
    while heap:
        score, _, state = heapq.heappop(heap)
        steps = best[state]
        if score != steps + estimate(*state):
            continue
        cell, mask = state
        if mask == full:
            route = [cell]
            while state != first:
                state = parent[state]
                route.append(state[0])
            return list(reversed(route))
        for other in sorted(graph[cell]):
            new_mask = mask | (1 << index[other] if other in index else 0)
            next_state = (other, new_mask)
            if steps + 1 < best.get(next_state, float("inf")):
                best[next_state] = steps + 1
                parent[next_state] = state
                serial += 1
                heapq.heappush(heap, (steps + 1 + estimate(*next_state), serial, next_state))
    raise ValueError("No route visits every selected sign cell")


def make_plan(path, selected, selected_signs=None):
    data, graph, signs = load_explore_map(path)
    if selected_signs is None:
        selected = set(selected)
        if not selected or any(color not in COLORS or shape not in SHAPES for color, shape in selected):
            raise ValueError("Select at least one of the 16 color and shape classes")
        targets = [sign for sign in signs if (sign["color"], sign["shape"]) in selected]
    else:
        selected_signs = set(selected_signs)
        available = {sign_key(sign) for sign in signs}
        if not selected_signs or not selected_signs <= available:
            raise ValueError("Select at least one sign shown on the Explore map")
        targets = [sign for sign in signs if sign_key(sign) in selected_signs]
        selected = {(sign["color"], sign["shape"]) for sign in targets}
    if not targets:
        raise ValueError("The map has no signs in the selected classes")
    goal_cells = {tuple(sign["cell"]) for sign in targets}
    route = shortest_target_route(graph, tuple(data["start_cell"]), goal_cells)
    directions = [DIRECTIONS.index((b[0] - a[0], b[1] - a[1]))
                  for a, b in zip(route, route[1:])]
    return {"source_map": str(Path(path).resolve()), "start_cell": list(route[0]),
            "route": [list(cell) for cell in route], "directions": directions,
            "target_cells": sorted(goal_cells),
            "targets": targets, "selected": sorted([list(key) for key in selected]),
            "step_count": len(directions), "cell_size_m": data["cell_size_m"]}


def _prepare_target_wall(backend, direction, robot_modes):
    """Face a mapped wall only when a turn is needed, then stop the wheels."""
    if direction != backend.heading:
        backend.select_robot_mode(robot_modes.CHASSIS_LEAD, "face target wall")
        backend.face(direction)
    pose = backend.prepare_stationary_scan()
    target_yaw = (direction * 90 + 180) % 360 - 180
    error = (pose.yaw - target_yaw + 180) % 360 - 180
    if abs(error) > setting("navigation.max_heading_error_deg"):
        raise RuntimeError("Cannot inspect target wall: chassis heading differs by {:.1f} deg".format(error))
    backend.select_robot_mode(robot_modes.FREE, "inspect target wall")
    return pose


def straight_runs(plan):
    """Group same-heading steps, ending at every selected sign or turn."""
    route = plan["route"]
    directions = plan["directions"]
    targets = {tuple(item["cell"]) for item in plan["targets"]}
    start = 0
    while start < len(directions):
        direction = directions[start]
        end = start + 1
        while (end < len(directions) and directions[end] == direction
               and tuple(route[end]) not in targets):
            end += 1
        yield start, end, direction
        start = end


def _wait_for_first_camera_frame(camera, timeout_sec, cancel=None):
    """Require a decoded frame before any Navigation chassis motion."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if cancel is not None and cancel.is_set():
            raise KeyboardInterrupt
        try:
            frame = camera.read_cv2_image(
                strategy="newest", timeout=min(0.5, max(0.01, deadline - time.monotonic())))
        except queue.Empty:
            continue
        if frame is not None:
            return frame
    raise RuntimeError("Camera started but no decoded video frame arrived within {:.1f}s".format(timeout_sec))


def execute(plan, on_sensor=None, on_progress=None, on_target=None,
            on_system_ready=None, cancel=None, on_frame=None, fire_type="water_fire",
            on_phase=None):
    """Drive one mapped edge at a time; visually reacquire each sign before firing."""
    from .detect_camera import detect_signs
    from .robot_system import RobotSystem
    from .sdk_connection import load_robot_sdk, require_camera_codec
    from .second_pass_control import SmoothWallCenteringPID
    from .navigation_hardware import NavigationBackend
    from .navigation_controller import NavigationController
    from .target_fire import TargetFireController

    run_dir = Path(project_path("paths.telemetry")) / ("navigate_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    run_dir.mkdir(parents=True)
    result_file = run_dir / "navigation_result.json"
    (run_dir / "navigation_plan.json").write_text(
        json.dumps(plan, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    result = {"status": "running", "source_map": plan["source_map"],
              "route": plan["route"], "completed_steps": 0,
              "current_cell": plan["start_cell"], "targets": [], "error": None,
              "result_file": str(result_file)}
    system = RobotSystem(results_dir=run_dir)
    backend = None
    camera = None
    camera_thread = None
    camera_stop = cancel if cancel is not None else threading.Event()
    lock = threading.Lock()
    inspection = {"active": False, "phase": "idle", "targets": []}
    events = []
    camera_errors = []
    last_camera_frame_at = None
    shooter = None
    motion_may_have_started = False

    def save():
        result_file.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    def fail_camera(detail):
        camera_errors.append(detail)
        events.append({"type": "camera_error", "error": detail})
        try:
            system.thread_2_controller.stop_running()
        except Exception as stop_error:
            events.append({"type": "camera_stop_error", "error": str(stop_error)})
        finally:
            camera_stop.set()

    def observe_camera():
        nonlocal last_camera_frame_at
        while not camera_stop.is_set():
            try:
                frame = camera.read_cv2_image(strategy="newest", timeout=0.5)
                if frame is None:
                    if time.monotonic() - last_camera_frame_at > setting("navigation.camera_frame_timeout_sec"):
                        raise RuntimeError("Camera video frames stopped arriving")
                    continue
                last_camera_frame_at = time.monotonic()
                with lock:
                    active = inspection["active"]
                    wanted = inspection.get("wanted", set())
                if not active:
                    if on_frame is not None:
                        on_frame(frame)
                    continue
                overlay, _, detections = detect_signs(frame)
                detections = [item for item in detections
                              if (item["color"], item["shape"]) in wanted]
                shooter.observe(detections, frame.shape)
                if on_frame is not None:
                    on_frame(overlay)
            except queue.Empty:
                if camera_stop.is_set():
                    break
                if time.monotonic() - last_camera_frame_at <= setting("navigation.camera_frame_timeout_sec"):
                    continue
                fail_camera("Camera video frames stopped arriving")
            except Exception as exc:
                if camera_stop.is_set():
                    break
                fail_camera(str(exc) or type(exc).__name__)

    def check_running():
        if camera_errors:
            raise RuntimeError("Camera stream failed: " + camera_errors[-1])
        if cancel is not None and cancel.is_set():
            raise KeyboardInterrupt
        if backend is not None and backend.scan_origin is not None:
            backend.ensure_running()

    def inspect_cell(cell):
        cell_targets = [target for target in plan["targets"] if tuple(target["cell"]) == cell]
        for direction in sorted({target["direction"] for target in cell_targets}):
            check_running()
            expected = [target for target in cell_targets if target["direction"] == direction]
            if on_phase is not None:
                on_phase("กำลังตรวจป้าย Grid {} ด้าน {}".format(cell, "NESW"[direction]))
            pose = _prepare_target_wall(backend, direction, load_robot_sdk())
            backend.scan_origin = (pose.pos_x, pose.pos_y, pose.yaw)
            wall_distance_mm = backend.sample(0, backend.aim(0)) * 1000
            wall_limit_mm = 1000 * (
                setting("navigation.grid_size_m") / 2
                + setting("slam.wall_margin_m") + setting("slam.localization_gate_m"))
            if wall_distance_mm > wall_limit_mm:
                raise RuntimeError("Mapped sign wall not confirmed by current ToF")
            wanted = {(item["color"], item["shape"]) for item in expected}
            found = set()
            for yaw in (0, -18, 18):
                check_running()
                action = system.robot.gimbal.moveto(
                    pitch=setting("fire.scan_pitch_deg"), yaw=yaw,
                    pitch_speed=setting("gimbal.pitch_speed_dps"),
                    yaw_speed=setting("gimbal.yaw_speed_dps"))
                if action.wait_for_completed(timeout=setting("gimbal.action_timeout_sec")) is False:
                    raise RuntimeError("Gimbal target sweep failed")
                with lock:
                    inspection.update(active=True, phase="survey", camera_yaw=yaw,
                                      camera_pitch=setting("fire.scan_pitch_deg"),
                                      targets=[], wanted=wanted - found)
                deadline = time.monotonic() + 1.8
                while time.monotonic() < deadline and not camera_stop.is_set():
                    check_running()
                    with lock:
                        confirmed = [dict(item) for item in inspection["targets"]
                                     if item["confirmed"] and
                                     (item["color"], item["shape"]) not in found]
                        if confirmed:
                            inspection["active"] = False
                            inspection["targets"] = confirmed
                    if confirmed:
                        for target in confirmed:
                            target["tof_distance_mm"] = wall_distance_mm
                        with lock:
                            inspection["targets"] = confirmed
                        shooter.fire_confirmed(cell, direction)
                        found.update((item["color"], item["shape"]) for item in confirmed)
                        break
                    time.sleep(0.03)
                with lock:
                    inspection["active"] = False
                if found == wanted:
                    break
            for target in expected:
                attempts = [event for event in events if event.get("type") == "target_fire"
                            and event.get("cell") == list(cell)
                            and event.get("direction") == direction
                            and event.get("color") == target["color"]
                            and event.get("shape") == target["shape"]]
                status = "fired" if any(event.get("fired") for event in attempts) else "not_fired"
                record = {"cell": list(cell), "direction": direction,
                          "color": target["color"], "shape": target["shape"],
                          "status": status}
                result["targets"].append(record)
                if on_target is not None:
                    on_target(record)
            with lock:
                inspection["active"] = False
            action = system.robot.gimbal.moveto(
                pitch=setting("gimbal.pitch_deg"), yaw=0,
                pitch_speed=setting("gimbal.pitch_speed_dps"),
                yaw_speed=setting("gimbal.yaw_speed_dps"))
            if action.wait_for_completed(timeout=setting("gimbal.action_timeout_sec")) is False:
                raise RuntimeError("Could not restore front Gimbal position")
            backend.commanded_gimbal_yaw = 0.0
            save()
            check_running()
            backend.scan_origin = None

    try:
        require_camera_codec()
        if not system.connect_robot():
            raise RuntimeError("Cannot connect to RoboMaster EP")
        system.setup_threads(controller_class=NavigationController)
        if on_sensor is not None:
            system.sensor_hub.add_listener(on_sensor)
        system.thread_1_sensor.start_collecting()
        system.thread_2_controller.base_speed = setting("second_pass.base_speed_mps")
        system.thread_2_controller.wall_pid = SmoothWallCenteringPID()
        system.thread_2_controller.pause_on_sensor_outage = True

        def recovery_update(phase, detail):
            events.append({"timestamp": time.time(), "type": "sensor_recovery_" + phase,
                           "detail": detail, "cell": result["current_cell"],
                           "completed_steps": result["completed_steps"]})
            result["status"] = "paused" if phase == "paused" else "running"
            save()
            if on_phase is not None:
                on_phase("พัก Navigation · หยุดล้อ รอข้อมูลเซนเซอร์ (กด หยุด เพื่อยกเลิก)"
                         if phase == "paused" else
                         "ข้อมูลเซนเซอร์กลับมาแล้ว · ตรวจตำแหน่งและทิศทางผ่าน กำลังเดินต่อ")

        system.thread_2_controller.recovery_callback = recovery_update
        backend = NavigationBackend(system)
        if on_system_ready is not None:
            on_system_ready(system)
        deadline = time.monotonic() + setting("slam.sensor_timeout_sec")
        while system.sensor_hub.get_latest_state().frame_index == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError("No initial sensor data")
            time.sleep(0.01)
        camera = system.robot.camera
        if camera.start_video_stream(display=False) is False:
            raise RuntimeError("RoboMaster camera stream could not start")
        _wait_for_first_camera_frame(camera, setting("navigation.camera_start_timeout_sec"), cancel)
        last_camera_frame_at = time.monotonic()
        shooter = TargetFireController(system.robot, inspection, lock, camera_stop,
                                       fire_type, events,
                                       pose_provider=system.sensor_hub.get_latest_state,
                                       target_color=None)
        camera_thread = threading.Thread(target=observe_camera, daemon=True)
        camera_thread.start()
        motion_may_have_started = True
        system.thread_2_controller.enable_motion()
        targets_by_cell = {tuple(item["cell"]) for item in plan["targets"]}
        if tuple(plan["start_cell"]) in targets_by_cell:
            inspect_cell(tuple(plan["start_cell"]))
        for start_index, end_index, direction in straight_runs(plan):
            check_running()
            source, target = plan["route"][start_index], plan["route"][end_index]
            if on_progress is not None:
                on_progress(start_index, source, target)
            if direction != backend.heading and on_phase is not None:
                on_phase("กำลังหมุน Chassis ไปทิศ {} เพื่อเดินสู่ Grid {}".format(
                    "NESW"[direction], tuple(target)))

            def reached_cell(count):
                step = start_index + count
                cell = plan["route"][step]
                result["completed_steps"] = step
                result["current_cell"] = cell
                save()
                if on_progress is not None:
                    on_progress(step, cell, None)

            backend.move(direction, align_if_straight=False,
                         cells=end_index - start_index, on_cell=reached_cell)
            if tuple(target) in targets_by_cell:
                inspect_cell(tuple(target))
        check_running()
        result["status"] = ("completed" if all(item["status"] == "fired"
                                            for item in result["targets"]) else "partial")
    except KeyboardInterrupt:
        result["status"] = "failed" if camera_errors else "interrupted"
        result["error"] = ("Camera stream failed: " + camera_errors[-1]
                           if camera_errors else "Stopped by operator")
        if motion_may_have_started:
            result["error"] += "; physical position may be uncertain"
    except Exception as exc:
        stopped_by_operator = cancel is not None and cancel.is_set() and not camera_errors
        result["status"] = "interrupted" if stopped_by_operator else "failed"
        result["error"] = "Stopped by operator" if stopped_by_operator else str(exc)
        if motion_may_have_started:
            result["error"] += "; physical position may be uncertain"
    finally:
        camera_stop.set()
        if backend is not None:
            try:
                backend.stop()
            except Exception:
                pass
        if camera_thread is not None:
            camera_thread.join(timeout=2)
        if camera is not None:
            try:
                camera.stop_video_stream()
            except Exception:
                pass
        system.shutdown(save_telemetry=True, run_analysis=False)
        (run_dir / "target_events.json").write_text(
            json.dumps(events, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        save()
    return result
