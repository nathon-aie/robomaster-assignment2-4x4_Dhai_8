import argparse
import json
import math
import statistics
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from detect_camera import detect_signs, build_side_by_side_view
try:
    from src.grid_slam import DIRECTIONS, NAMES, DFSExplorer, wrap
    from src.settings import get as setting
    from src.settings import project_path
    from src.target_fire import TargetFireController
except ImportError:
    from SLAM.src.grid_slam import DIRECTIONS, NAMES, DFSExplorer, wrap
    from SLAM.src.settings import get as setting
    from SLAM.src.settings import project_path
    from SLAM.src.target_fire import TargetFireController


PROJECT_ROOT = Path(__file__).resolve().parent
MAX_SENSOR_AGE_SEC = setting("slam.max_sensor_age_sec")
FRONT_VIEW_TOLERANCE_DEG = 12.0
MAX_STATIONARY_SPEED_MPS = 0.03
SIGN_CONFIRMATION_FRAMES = 3
MAX_TOF_MARK_SAMPLES = 15
SIGN_LOOK_DOWN_PITCH_DEG = -20.0
SIGN_INSPECTION_TIMEOUT_SEC = 5.4
SIGN_CAMERA_SWEEP_OFFSETS_DEG = (0.0, -18.0, 18.0)
FRONT_STOP_TARGET_MM = 220.0


def front_wall_measurement(slam, state, now=None):
    if now is None:
        now = time.monotonic()

    if not state.tof_valid or state.tof_filtered_mm is None:
        return None
    if not 0 < state.tof_received_at <= now:
        return None
    if now - state.tof_received_at > MAX_SENSOR_AGE_SEC:
        return None
    if not 0 < state.gimbal_received_at <= now:
        return None
    if now - state.gimbal_received_at > MAX_SENSOR_AGE_SEC:
        return None
    if abs(state.gimbal_yaw) > FRONT_VIEW_TOLERANCE_DEG:
        return None
    chassis_heading_error = wrap(state.yaw - wrap(slam.heading * 90))
    if abs(chassis_heading_error) > FRONT_VIEW_TOLERANCE_DEG:
        return None
    if not state.is_static or any(
        abs(speed) > MAX_STATIONARY_SPEED_MPS
        for speed in (state.vel_vx, state.vel_vy, state.vel_vz)
    ):
        return None

    position_error = math.hypot(
        state.pos_x - slam.pose[0], state.pos_y - slam.pose[1]
    )
    if position_error > setting("slam.localization_gate_m") + 0.05:
        return None

    return front_wall_distance(slam, slam.heading, state.tof_filtered_mm / 1000.0)


def front_wall_distance(slam, direction, distance_m, sensor_heading=None):
    if direction not in range(4) or not math.isfinite(distance_m) or distance_m <= 0:
        return None
    if sensor_heading is None:
        sensor_heading = direction

    dx, dy = DIRECTIONS[direction]
    axis, sign = (0, dx) if dx else (1, dy)
    offset = slam.offset(sensor_heading, direction)
    boundary = (slam.cell[axis] + sign * 0.5) * setting("navigation.grid_size_m")
    expected_mm = 1000 * (
        sign * (boundary - slam.pose[axis] - offset[axis])
        - setting("slam.wall_thickness_m") / 2
    )
    if expected_mm <= 0:
        return None

    distance_mm = distance_m * 1000
    # Odometry may still be off-centre before SLAM corrects this scan.
    # Keep nearby side walls eligible for camera inspection while excluding
    # the next cell's distant wall.
    nominal_limit_mm = 1000 * (
        setting("navigation.grid_size_m") / 2
        - sign * offset[axis]
        - setting("slam.wall_thickness_m") / 2
        + setting("slam.wall_margin_m")
        + setting("slam.localization_gate_m")
    )
    if distance_mm > max(expected_mm + setting("slam.wall_margin_m") * 1000,
                         nominal_limit_mm):
        return None
    return distance_mm


def inspect_walls_during_scan(explorer, inspection, lock, finished, shooter=None):
    backend = explorer.backend
    original_scan = backend.scan
    robot = backend.robot

    def scan_with_sign_inspection(*args, **kwargs):
        scan_result = original_scan(*args, **kwargs)
        ranges, heading, _ = scan_result
        scan_headings = getattr(backend, "scan_headings", None) or {}
        walls = []
        for direction, distance_m in ranges.items():
            sensor_heading = scan_headings.get(direction, heading)
            distance_mm = front_wall_distance(
                explorer.slam, direction, distance_m, sensor_heading
            )
            if distance_mm is not None:
                walls.append((direction, sensor_heading, distance_mm))

        print("[Sign scan] Nearby walls: {}".format(
            ", ".join(NAMES[direction] for direction, _, _ in walls) or "none"))
        state = backend.hub.get_latest_state()
        if (any(abs(speed) > MAX_STATIONARY_SPEED_MPS
                for speed in (state.vel_vx, state.vel_vy, state.vel_vz))
                or abs(wrap(state.yaw - wrap(heading * 90))) > FRONT_VIEW_TOLERANCE_DEG):
            print("[Sign scan] Skipped: chassis is moving or heading is not aligned")
            return scan_result

        yaw_for_relative_direction = {0: 0, 1: 90, 2: -180, 3: -90}
        pending_targets = {}
        for direction, sensor_heading, distance_mm in walls:
            relative_direction = (direction - heading) % 4
            camera_yaw = yaw_for_relative_direction[relative_direction]
            sweep_yaws = []
            with lock:
                inspection.update({
                    "active": False,
                    "cell": tuple(explorer.slam.cell),
                    "direction": direction,
                    "frame_count": 0,
                    "detected_frames": 0,
                    "targets": [],
                    "phase": "idle",
                })
            try:
                # 1. หมุนแนวนอน (Yaw) ไปที่กำแพงด้านนั้นก่อนในระดับสายตาปกติ
                turn_action = robot.gimbal.moveto(
                    pitch=setting("gimbal.pitch_deg"),
                    yaw=camera_yaw,
                    pitch_speed=setting("gimbal.pitch_speed_dps"),
                    yaw_speed=setting("gimbal.yaw_speed_dps"),
                )
                turn_completed = turn_action.wait_for_completed(
                    timeout=setting("gimbal.action_timeout_sec")
                )
                if turn_completed is False or getattr(turn_action, "has_succeeded", True) is False:
                    print("Gimbal horizontal turn failed for {} wall.".format(NAMES[direction]))
                    continue

                # 2. เมื่อถึงแนวกำแพงแล้ว จึงก้มหน้าลงตรวจจับเป้าหมาย
                action = robot.gimbal.moveto(
                    pitch=SIGN_LOOK_DOWN_PITCH_DEG,
                    yaw=camera_yaw,
                    pitch_speed=setting("gimbal.pitch_speed_dps"),
                    yaw_speed=setting("gimbal.yaw_speed_dps"),
                )
                completed = action.wait_for_completed(
                    timeout=setting("gimbal.action_timeout_sec")
                )
                if completed is False or getattr(action, "has_succeeded", True) is False:
                    print("Sign look-down action failed for {} wall.".format(NAMES[direction]))
                    continue

                with lock:
                    inspection.update({
                        "active": False,
                        "cell": tuple(explorer.slam.cell),
                        "direction": direction,
                        "tof_distance_mm": distance_mm,
                        "frame_count": 0,
                        "detected_frames": 0,
                        "targets": [],
                    })

                sweep_yaws = [
                    camera_yaw + offset
                    for offset in SIGN_CAMERA_SWEEP_OFFSETS_DEG
                    if -250 <= camera_yaw + offset <= 250
                ]
                print("Inspecting {} wall in cell {} at yaw {} for {:.1f}s...".format(
                    NAMES[direction], tuple(explorer.slam.cell),
                    sweep_yaws, SIGN_INSPECTION_TIMEOUT_SEC,
                ))
                dwell_per_yaw = SIGN_INSPECTION_TIMEOUT_SEC / max(1, len(sweep_yaws))
                for sweep_yaw in sweep_yaws:
                    if finished.is_set() or not backend.controller._running.is_set():
                        break
                    with lock:
                        inspection["active"] = False
                    try:
                        if sweep_yaw != camera_yaw:
                            sweep_action = robot.gimbal.moveto(
                                pitch=SIGN_LOOK_DOWN_PITCH_DEG,
                                yaw=sweep_yaw,
                                pitch_speed=setting("gimbal.pitch_speed_dps"),
                                yaw_speed=setting("gimbal.yaw_speed_dps"),
                            )
                            sweep_completed = sweep_action.wait_for_completed(
                                timeout=setting("gimbal.action_timeout_sec")
                            )
                            if (sweep_completed is False
                                    or getattr(sweep_action, "has_succeeded", True) is False):
                                continue
                        time.sleep(setting("gimbal.settle_sec"))
                        with lock:
                            inspection["cell"] = tuple(explorer.slam.cell)
                            inspection["direction"] = direction
                            inspection["tof_distance_mm"] = distance_mm
                            inspection["camera_yaw"] = sweep_yaw
                            inspection["camera_pitch"] = SIGN_LOOK_DOWN_PITCH_DEG
                            inspection["phase"] = "survey"
                            inspection["active"] = True
                        dwell_deadline = time.monotonic() + dwell_per_yaw
                        while time.monotonic() < dwell_deadline:
                            if finished.wait(min(0.02, dwell_deadline - time.monotonic())):
                                break
                    except Exception as error:
                        print("Gimbal sweep warning ({} deg): {}".format(
                            sweep_yaw, error
                        ))
            except Exception as error:
                print("Sign look-down warning ({}): {}".format(NAMES[direction], error))
            finally:
                with lock:
                    inspection["active"] = False
                    inspected_frames = inspection.get("frame_count", 0)
                    detected_frames = inspection.get("detected_frames", 0)
                    confirmed_targets = [dict(target) for target in inspection.get("targets", [])
                                         if target["confirmed"]]
                for target in confirmed_targets:
                    key = (direction, target["color"], target["shape"])
                    pending_targets.setdefault(key, []).append(target)
                if hasattr(explorer.slam, "events"):
                    explorer.slam.events.append({
                        "timestamp": time.time(), "type": "sign_inspection",
                        "cell": list(explorer.slam.cell),
                        "direction": NAMES[direction],
                        "camera_yaws": sweep_yaws,
                        "frames": inspected_frames,
                        "detected_frames": detected_frames,
                    })
                # ตรวจด้านนี้เสร็จแล้ว ให้เงยหน้าขึ้นมาก่อน (Pitch up to level)
                try:
                    pitch_up_action = robot.gimbal.moveto(
                        pitch=setting("gimbal.pitch_deg"),
                        yaw=camera_yaw,
                        pitch_speed=setting("gimbal.pitch_speed_dps"),
                        yaw_speed=setting("gimbal.yaw_speed_dps"),
                    )
                    pitch_up_action.wait_for_completed(
                        timeout=setting("gimbal.action_timeout_sec")
                    )
                    with lock:
                        inspection["camera_yaw"] = camera_yaw
                        inspection["camera_pitch"] = setting("gimbal.pitch_deg")
                except Exception as error:
                    print("Gimbal pitch-up warning: {}".format(error))

        # Finish every wall sweep before moving the Gimbal to shoot. The map
        # also identifies a sign by cell, direction, color and shape, so use
        # that same identity to avoid another burst at the same sign.
        if shooter is not None and not finished.is_set():
            targets_to_fire = []
            for (direction, color, shape), sightings in pending_targets.items():
                targets_to_fire.append({
                    "direction": direction, "color": color, "shape": shape,
                    "yaw": statistics.median(item["yaw"] for item in sightings),
                    "pitch": statistics.median(item["pitch"] for item in sightings),
                    "confirmed": True,
                })
            with lock:
                inspection.update(active=False, phase="idle", targets=targets_to_fire)
            explorer.slam.events.append({
                "timestamp": time.time(), "type": "target_scan_complete",
                "cell": list(explorer.slam.cell),
                "targets": [
                    {"direction": target["direction"], "color": target["color"],
                     "shape": target["shape"], "yaw": round(target["yaw"], 2),
                     "pitch": round(target["pitch"], 2)}
                    for target in targets_to_fire
                ],
            })
            print("[Target fire] Scan complete: {} distinct sign(s) to fire at.".format(
                len(targets_to_fire)))
            try:
                shooter.fire_confirmed(tuple(explorer.slam.cell), heading)
            except Exception as error:
                print("[Target fire] Warning: {}".format(error))

        # ตรวจครบทุกด้านแล้ว หมุนกลับมาหน้าตรง
        try:
            restore_action = robot.gimbal.moveto(
                pitch=setting("gimbal.pitch_deg"),
                yaw=0,
                pitch_speed=setting("gimbal.pitch_speed_dps"),
                yaw_speed=setting("gimbal.yaw_speed_dps"),
            )
            restored = restore_action.wait_for_completed(
                timeout=setting("gimbal.action_timeout_sec")
            )
            if restored is False or getattr(restore_action, "has_succeeded", True) is False:
                print("Warning: could not restore the configured gimbal pose.")
        except Exception as error:
            print("Gimbal restore warning: {}".format(error))

        return scan_result

    backend.scan = scan_with_sign_inspection


def save_sign_marks(output, marks):
    path = Path(output)
    data = json.loads(path.read_text(encoding="utf-8"))
    data["signs"] = sorted(
        marks.values(),
        key=lambda item: (item["cell"][0], item["cell"][1], item["direction"],
                          item["color"], item["shape"]),
    )
    data["sign_count"] = len(data["signs"])
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    temp.replace(path)


def render_map_image(map_file):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Circle, Patch, Rectangle

    path = Path(map_file)
    data = json.loads(path.read_text(encoding="utf-8"))
    cell_size = data["cell_size_m"]
    rows = data["map_info"]["rows"]
    columns = data["map_info"]["columns"]
    figure, axis = plt.subplots(figsize=(10, 8))

    for cell_x, cell_y in data["visited"]:
        axis.add_patch(Rectangle(
            (cell_y * cell_size - cell_size / 2,
             cell_x * cell_size - cell_size / 2),
            cell_size, cell_size, facecolor="#e7f1f7",
            edgecolor="#aab9c4", linewidth=0.8, zorder=0,
        ))

    for edge in data["edges"]:
        if not edge["wall"]:
            continue
        (cell_a, cell_b) = edge["cells"]
        if cell_a[0] != cell_b[0]:
            wall_x = (cell_a[1] + cell_b[1]) * cell_size / 2
            wall_y = (cell_a[0] + cell_b[0]) * cell_size / 2
            axis.plot([wall_x - cell_size / 2, wall_x + cell_size / 2],
                      [wall_y, wall_y], color="#263746", linewidth=3.2,
                      solid_capstyle="round", zorder=2)
        else:
            wall_x = (cell_a[1] + cell_b[1]) * cell_size / 2
            wall_y = (cell_a[0] + cell_b[0]) * cell_size / 2
            axis.plot([wall_x, wall_x],
                      [wall_y - cell_size / 2, wall_y + cell_size / 2],
                      color="#263746", linewidth=3.2,
                      solid_capstyle="round", zorder=2)

    trajectory = data.get("trajectory", [])
    if trajectory:
        axis.plot(
            [point["pose"][1] for point in trajectory],
            [point["pose"][0] for point in trajectory],
            "o-", color="#1486bd", markersize=3.5, linewidth=1.5,
            label="Robot path", zorder=3,
        )

    start = data.get("start_pose", [0.0, 0.0, 0.0])
    axis.scatter([start[1]], [start[0]], marker="s", s=105,
                 color="#23a66f", edgecolors="white", linewidths=1,
                 label="Start {}".format(tuple(data.get("start_cell", [0, 0]))),
                 zorder=6)
    axis.scatter([data["pose"][1]], [data["pose"][0]], marker="X", s=100,
                 color="#e4473d", edgecolors="white", linewidths=0.8,
                 label="Last estimated pose", zorder=7)

    palette = {
        "Red": "#e3423a", "Green": "#28a765",
        "Blue": "#2776c9", "Yellow": "#e2bd00",
    }
    marker_labels = set()
    for sign in data.get("signs", []):
        cell_x, cell_y = sign["cell"]
        direction = NAMES.index(sign["direction"])
        color = palette.get(sign["color"], "#8b5fbf")
        offset = cell_size * 0.34
        center_x, center_y = cell_y * cell_size, cell_x * cell_size
        if direction == 0:
            center_y += offset
            label_y = center_y + cell_size * 0.10
            label_x = center_x
            vertical_align = "bottom"
        elif direction == 1:
            center_x += offset
            label_x = center_x + cell_size * 0.09
            label_y = center_y
            vertical_align = "center"
        elif direction == 2:
            center_y -= offset
            label_y = center_y - cell_size * 0.10
            label_x = center_x
            vertical_align = "top"
        else:
            center_x -= offset
            label_x = center_x - cell_size * 0.09
            label_y = center_y
            vertical_align = "center"

        shape = sign["shape"]
        if shape == "Circle":
            patch = Circle((center_x, center_y), cell_size * 0.065)
        else:
            width = cell_size * (0.16 if shape == "Horizontal_Rect" else 0.11)
            height = cell_size * (0.16 if shape == "Vertical_Rect" else 0.11)
            patch = Rectangle((center_x - width / 2, center_y - height / 2),
                              width, height)
        patch.set_facecolor(color)
        patch.set_edgecolor("#14212b")
        patch.set_linewidth(1.4)
        patch.set_zorder(8)
        axis.add_patch(patch)

        marker_labels.add((sign["color"], color))

    handles, labels = axis.get_legend_handles_labels()
    handles.insert(0, Line2D([0], [0], color="#263746", linewidth=3.2, label="Wall"))
    labels.insert(0, "Wall")
    for color_name, color in sorted(marker_labels):
        handles.append(Patch(facecolor=color, edgecolor="#14212b"))
        labels.append("{} target".format(color_name))
    axis.legend(
        handles,
        labels,
        bbox_to_anchor=(1.02, 1.0),
        loc="upper left",
        fontsize=8.5,
        framealpha=0.96,
        borderaxespad=0,
    )

    axis.set_xlim(-cell_size / 2, (columns - 0.5) * cell_size)
    axis.set_ylim(-cell_size / 2, (rows - 0.5) * cell_size)
    axis.set_xticks([(index - 0.5) * cell_size for index in range(columns + 1)])
    axis.set_yticks([(index - 0.5) * cell_size for index in range(rows + 1)])
    axis.set_aspect("equal")
    axis.set_axisbelow(True)
    axis.grid(color="#aab9c4", linewidth=0.7, alpha=0.7)
    axis.set_xlabel("Initial right axis y (m)")
    axis.set_ylabel("Initial forward axis x (m)")
    axis.set_title(
        "Exploration map | {} | {} / {} cells visited | {} sign marks".format(
            data["status"], len(data["visited"]), rows * columns,
            len(data.get("signs", [])),
        )
    )
    figure.tight_layout()
    image_path = path.with_name("map.png")
    figure.savefig(str(image_path), dpi=180, bbox_inches="tight")
    plt.close(figure)
    return image_path


def clear_previous_captures(dirs=None):
    """Deletes old captured target images from previous runs."""
    deleted_count = 0
    clean_dirs = []
    if dirs:
        if isinstance(dirs, (str, Path)):
            dirs = [dirs]
        clean_dirs.extend([Path(d) for d in dirs])

    # Remove legacy root captured_signs folder if present
    legacy_root = Path("captured_signs")
    if legacy_root.is_dir():
        import shutil
        shutil.rmtree(legacy_root, ignore_errors=True)

    # Search for all existing captured_signs directories in telemetry_logs and mission_maps
    for parent_folder in ("telemetry_logs", "mission_maps"):
        p = Path(parent_folder)
        if p.is_dir():
            for sub_captures in p.glob("**/captured_signs"):
                if sub_captures.is_dir() and sub_captures not in clean_dirs:
                    clean_dirs.append(sub_captures)

    for d in clean_dirs:
        if d.is_dir():
            for f in d.glob("*.*"):
                if f.suffix.lower() in (".jpg", ".jpeg", ".png"):
                    try:
                        f.unlink()
                        deleted_count += 1
                    except Exception:
                        pass
    if deleted_count > 0:
        print("[Cleanup] ลบภาพที่แคปจากการสำรวจครั้งก่อนหน้าทิ้งแล้ว {} รูป".format(deleted_count))
    return deleted_count


def run_camera_loop(explorer, camera, camera_is_robot, stop_motion=None,
                    control=None, fire_type="water_fire", on_frame=None):
    outcome = {"completed": False, "error": None}
    sign_marks = {}
    confirmation_streaks = {}
    inspection = {"active": False}
    inspection_lock = threading.Lock()
    inspection_finished = threading.Event()

    captured_signs_dir = Path(explorer.output).parent / "captured_signs"
    captured_signs_dir.mkdir(parents=True, exist_ok=True)
    clear_previous_captures([captured_signs_dir])
    captured_snapshots = {}

    shooter = None
    if camera_is_robot:
        robot = getattr(getattr(explorer, "backend", None), "robot", None)
        if robot is not None and getattr(robot, "blaster", None) is not None:
            shooter = TargetFireController(
                robot, inspection, inspection_lock, inspection_finished,
                fire_type, explorer.slam.events,
                pose_provider=explorer.backend.hub.get_latest_state,
            )
            inspect_walls_during_scan(
                explorer, inspection, inspection_lock, inspection_finished, shooter
            )
        else:
            inspect_walls_during_scan(
                explorer, inspection, inspection_lock, inspection_finished
            )

    def explore():
        try:
            outcome["completed"] = explorer.run()
        except BaseException as error:
            outcome["error"] = error

    worker = threading.Thread(target=explore, name="maze-explorer", daemon=True)
    worker.start()
    announced = False
    window_created = False

    try:
        print("Camera is live. {}".format(
            "Use the GUI Stop button or close its camera window."
            if on_frame is not None else "Press 'q' to stop/close the mission window."))
        while True:
            if camera_is_robot:
                frame = camera.read_cv2_image(strategy="newest", timeout=0.5)
            else:
                ok, frame = camera.read()
                if not ok:
                    frame = None

            if frame is None:
                time.sleep(0.01)
            else:
                result, mask, detections = detect_signs(frame)
                valid_wall_distance = None
                active_inspection = None
                if camera_is_robot:
                    with inspection_lock:
                        if inspection["active"]:
                            active_inspection = dict(inspection)
                    if active_inspection is not None:
                        if shooter is not None:
                            shooter.observe(detections, frame.shape)
                        if active_inspection.get("phase") == "survey":
                            valid_wall_distance = active_inspection["tof_distance_mm"]

                current_cell = (
                    active_inspection["cell"] if active_inspection is not None
                    else tuple(explorer.slam.cell)
                )
                current_direction = (
                    active_inspection["direction"] if active_inspection is not None
                    else explorer.slam.heading
                )

                valid_candidates = set()
                if valid_wall_distance is not None:
                    # เก็บและตรวจทุกเป้าที่กล้องเห็นใน Cell นี้
                    sorted_dets = sorted(detections, key=lambda d: d.get("confidence", 0.0), reverse=True)
                    target_detections = sorted_dets
                    for detection in target_detections:
                        key = (current_cell, current_direction,
                               detection["color"], detection["shape"])
                        if key in valid_candidates:
                            continue
                        valid_candidates.add(key)
                        streak = confirmation_streaks.get(key, 0) + 1
                        confirmation_streaks[key] = streak

                        # Capture up to 3 confirmation snapshot images
                        if key not in captured_snapshots:
                            captured_snapshots[key] = []
                        if len(captured_snapshots[key]) < SIGN_CONFIRMATION_FRAMES:
                            snap_idx = len(captured_snapshots[key]) + 1
                            cell_str = "c{}_{}".format(current_cell[0], current_cell[1])
                            img_name = "{}_{}_{}_{}_snap{}.jpg".format(
                                cell_str, NAMES[current_direction], detection["color"],
                                detection["shape"], snap_idx
                            )
                            img_path = captured_signs_dir / img_name
                            try:
                                cv2.imwrite(str(img_path), frame)
                                captured_snapshots[key].append(img_name)
                                print("[Snapshot] Saved target photo {}/{}: {}".format(
                                    snap_idx, SIGN_CONFIRMATION_FRAMES, img_name
                                ))
                            except Exception as write_err:
                                print("[Snapshot] Warning saving photo {}: {}".format(img_name, write_err))

                        if streak >= SIGN_CONFIRMATION_FRAMES:
                            if shooter is not None:
                                shooter.confirm_detection(detection)
                            mark = sign_marks.get(key)
                            if mark is None:
                                mark = {
                                    "color": detection["color"],
                                    "shape": detection["shape"],
                                    "cell": list(current_cell),
                                    "direction": NAMES[current_direction],
                                    "tof_distances_mm": [],
                                    "confirmed_frames": streak,
                                    "captured_images": captured_snapshots.get(key, []),
                                }
                                sign_marks[key] = mark
                            elif streak > SIGN_CONFIRMATION_FRAMES:
                                mark["confirmed_frames"] += 1
                                mark["captured_images"] = captured_snapshots.get(key, [])
                            mark["tof_distances_mm"].append(
                                round(valid_wall_distance, 1)
                            )
                            mark["tof_distances_mm"] = mark["tof_distances_mm"][-MAX_TOF_MARK_SAMPLES:]
                            mark["tof_distance_mm"] = round(
                                float(np.median(mark["tof_distances_mm"])), 1
                            )
                            mark["last_seen_timestamp"] = time.time()
                    confirmation_streaks = {
                        key: streak for key, streak in confirmation_streaks.items()
                        if key in valid_candidates
                    }
                else:
                    confirmation_streaks.clear()

                if active_inspection is not None:
                    with inspection_lock:
                        if inspection.get("active"):
                            inspection["frame_count"] += 1
                            if detections:
                                inspection["detected_frames"] = inspection.get("detected_frames", 0) + 1

                gate_text = (
                    "WALL VERIFIED - LOOKING DOWN: sign {}/{} frames | marked {}"
                    .format(max(confirmation_streaks.values(), default=0),
                            SIGN_CONFIRMATION_FRAMES, len(sign_marks))
                    if valid_wall_distance is not None
                    else "WAITING FOR FRONT WALL SCAN - signs are not marked"
                )
                cv2.putText(result, gate_text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (0, 255, 255), 2, cv2.LINE_AA)
                if on_frame is not None:
                    on_frame(build_side_by_side_view(result, mask))
                else:
                    combined = build_side_by_side_view(result, mask)
                    cv2.imshow("RoboMaster - Camera & Sign Mask", combined)
                    window_created = True
                if detections and on_frame is None:
                    labels = sorted(set(
                        "{} {}".format(item["color"], item["shape"])
                        for item in detections
                    ))
                    cv2.setWindowTitle(
                        "RoboMaster - Camera & Sign Mask",
                        "RoboMaster - " + ", ".join(labels),
                    )

            if not worker.is_alive() and not announced:
                status = explorer.status
                print("Maze exploration {} after {} move(s), {} cell(s) visited."
                      .format(status, explorer.moves, len(explorer.slam.map.visited)))
                print("Map saved to: {}".format(explorer.output))
                announced = True

            # Let OpenCV create/update the window before checking whether it was closed.
            key = cv2.waitKey(1) & 0xFF if on_frame is None else -1
            # Check if user closed the OpenCV window via the [X] title-bar button
            try:
                if window_created and cv2.getWindowProperty("RoboMaster - Camera & Sign Mask", cv2.WND_PROP_VISIBLE) < 1:
                    print("[Camera] Window closed by user.")
                    inspection_finished.set()
                    if worker.is_alive() and stop_motion is not None:
                        stop_motion()
                    break
            except Exception:
                pass

            if control and control.cancel.is_set():
                print("[Camera] Cancel signal received from control.")
                inspection_finished.set()
                if worker.is_alive() and stop_motion is not None:
                    stop_motion()
                break

            if key == ord("q"):
                inspection_finished.set()
                if worker.is_alive() and stop_motion is not None:
                    print("Stopping maze motion...")
                    stop_motion()
                break

            if not worker.is_alive() and outcome["error"] is not None:
                break
            if not worker.is_alive():
                break
    except Exception:
        print("[Camera] Unexpected error; stopping motion. Traceback:")
        traceback.print_exc()
        raise
    finally:
        inspection_finished.set()
        if worker.is_alive() and stop_motion is not None:
            stop_motion()
        worker.join()
        if on_frame is None:
            cv2.destroyAllWindows()
        if explorer.error:
            print("[SLAM Error] {}".format(explorer.error))
        if camera_is_robot:
            save_sign_marks(explorer.output, sign_marks)
            print("Saved {} confirmed sign mark(s) to the SLAM map."
                  .format(len(sign_marks)))

    if outcome["error"] is not None:
        raise outcome["error"]
    return outcome["completed"]


def run_simulation(camera_index, output, control=None, on_frame=None):
    from SLAM.src.slam_simulation import SimulationBackend

    camera = cv2.VideoCapture(camera_index)
    if not camera.isOpened():
        camera.release()
        raise RuntimeError("Cannot open webcam/camera index {}".format(camera_index))

    explorer = DFSExplorer(SimulationBackend(), output)
    try:
        return run_camera_loop(explorer, camera, camera_is_robot=False,
                               control=control, on_frame=on_frame)
    finally:
        camera.release()


def run_hardware(conn_type, calibration_path, output, control=None,
                 fire_type="water_fire", on_frame=None):
    try:
        from src.robot_system import RobotSystem
        from src.slam_hardware import HardwareBackend
    except ImportError:
        from SLAM.src.robot_system import RobotSystem
        from SLAM.src.slam_hardware import HardwareBackend

    system = RobotSystem(calibration_file=str(calibration_path), conn_type=conn_type)
    if control:
        control.set_system(system)
    camera = None
    stream_started = False
    try:
        if not system.connect_robot():
            raise RuntimeError("Could not connect to RoboMaster EP; refusing to use mock mode")
        if control and control.cancel.is_set():
            return False

        system.setup_threads()
        system.thread_2_controller.wall_pid.front_target_mm = max(
            system.thread_2_controller.wall_pid.front_target_mm,
            FRONT_STOP_TARGET_MM,
        )
        print("Front-wall stopping target: {:.0f} mm (plus configured stop tolerance)."
              .format(system.thread_2_controller.wall_pid.front_target_mm))
        system.thread_1_sensor.start_collecting()
        system.thread_2_controller._running.set()

        deadline = time.monotonic() + setting("slam.sensor_timeout_sec")
        while system.sensor_hub.get_latest_state().frame_index == 0:
            if control and control.cancel.is_set():
                return False
            if time.monotonic() >= deadline:
                raise RuntimeError("No initial sensor data from RoboMaster EP")
            time.sleep(0.01)

        camera = system.robot.camera
        camera.start_video_stream(display=False)
        stream_started = True
        explorer = DFSExplorer(HardwareBackend(system), output)
        stop_motion = system.thread_2_controller.stop_running
        return run_camera_loop(
            explorer, camera, camera_is_robot=True, stop_motion=stop_motion,
            control=control, fire_type=fire_type, on_frame=on_frame
        )
    finally:
        if system.thread_2_controller is not None:
            try:
                system.thread_2_controller.stop_running()
            except Exception as error:
                print("Motion stop warning: {}".format(error))
        try:
            if stream_started:
                camera.stop_video_stream()
        finally:
            system.shutdown(run_analysis=False)


def main():
    parser = argparse.ArgumentParser(
        description="Explore a maze with Grid SLAM while detecting signs from the camera"
    )
    parser.add_argument("--mock", action="store_true",
                        help="Run the SLAM simulator and show a PC webcam")
    parser.add_argument("--camera-index", type=int, default=0,
                        help="PC webcam index used with --mock")
    parser.add_argument("--conn-type", choices=("ap", "sta"),
                        default=setting("robot.conn_type"))
    parser.add_argument("--fire-type", choices=("water_fire", "infared_fire", "infrared_fire"),
                        default="water_fire", help="Blaster mode for every confirmed target")
    parser.add_argument("--calibration", default=project_path("paths.calibration"))
    parser.add_argument("--output-dir", default=str(PROJECT_ROOT / "mission_maps"),
                        help="Folder for timestamped mission results (outside SLAM)")
    parser.add_argument("--output", default=None,
                        help="Optional explicit map JSON path; overrides --output-dir")
    args = parser.parse_args()

    calibration_path = Path(args.calibration)
    if not calibration_path.is_absolute():
        if (PROJECT_ROOT / calibration_path).is_file():
            calibration_path = PROJECT_ROOT / calibration_path
        elif (PROJECT_ROOT / "SLAM" / calibration_path).is_file():
            calibration_path = PROJECT_ROOT / "SLAM" / calibration_path
        else:
            calibration_path = PROJECT_ROOT / calibration_path

    if args.output:
        output_path = Path(args.output)
        if not output_path.is_absolute():
            output_path = PROJECT_ROOT / output_path
        output_path.parent.mkdir(parents=True, exist_ok=True)
    else:
        output_root = Path(args.output_dir)
        if not output_root.is_absolute():
            output_root = PROJECT_ROOT / output_root
        run_dir = output_root / datetime.now().strftime("run_%Y%m%d_%H%M%S_%f")
        run_dir.mkdir(parents=True, exist_ok=False)
        output_path = run_dir / "explored_map.json"
    print("Mission output folder: {}".format(output_path.parent))

    try:
        if args.mock:
            success = run_simulation(args.camera_index, output_path)
        else:
            success = run_hardware(args.conn_type, calibration_path, output_path,
                                   fire_type=args.fire_type)
        image_path = render_map_image(output_path)
        print("Map image saved to: {}".format(image_path))
        return 0 if success else 1
    except (OSError, RuntimeError, ValueError) as error:
        print("Mission error: {}".format(error))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
