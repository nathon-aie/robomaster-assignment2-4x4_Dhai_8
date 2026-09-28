"""Load shared project defaults, independent of the current working directory."""
from pathlib import Path
import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent
SETTINGS_FILE = PROJECT_ROOT / "config" / "settings.yaml"
try:
    SETTINGS = yaml.safe_load(SETTINGS_FILE.read_text(encoding="utf-8"))
except (OSError, yaml.YAMLError) as exc:
    raise ValueError("Cannot load settings from {}: {}".format(SETTINGS_FILE, exc)) from exc
if not isinstance(SETTINGS, dict):
    raise ValueError("settings.yaml must contain a mapping")


def get(key):
    """Read a dotted key; missing settings fail explicitly rather than silently fallback."""
    value = SETTINGS
    try:
        for part in key.split("."):
            value = value[part]
    except (KeyError, TypeError) as exc:
        raise ValueError("Missing setting: {}".format(key)) from exc
    return value


def project_path(key):
    path = Path(get(key))
    return str(path if path.is_absolute() else PROJECT_ROOT / path)

# Reject invalid basic settings before any hardware connection is attempted.
for key in ("sensors.rate_hz", "sensors.history_capacity",
            "navigation.grid_size_m", "navigation.base_speed_mps",
            "navigation.step_test_speed_mps", "navigation.control_rate_hz",
            "navigation.nominal_side_mm", "navigation.front_target_mm",
            "navigation.emergency_front_mm", "navigation.end_deceleration_m",
            "telemetry.buffer_capacity"):
    value = get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 < value < float("inf"):
        raise ValueError("Setting {} must be a finite positive number".format(key))
if get("robot.conn_type") not in ("ap", "sta"):
    raise ValueError("robot.conn_type must be ap or sta")
if get("navigation.emergency_front_mm") > get("navigation.front_target_mm"):
    raise ValueError("emergency_front_mm must not exceed front_target_mm")
for key in ("sensors.sharp_filter", "sensors.tof_filter"):
    config = get(key)
    if not 0 < config["ema_alpha"] <= 1 or config["min_valid"] >= config["max_valid"]:
        raise ValueError("Invalid filter settings: {}".format(key))
    if type(config["median_window"]) is not int or config["median_window"] < 1:
        raise ValueError("{}.median_window must be a positive integer".format(key))

for key in ("slam.sensor_timeout_sec", "slam.max_sensor_age_sec", "slam.scan_pose_recovery_timeout_sec", "slam.wall_margin_m",
            "slam.localization_gate_m", "slam.initial_variance_m2", "slam.motion_variance_m2",
            "slam.range_variance_m2", "slam.max_pose_std_m", "slam.cell_arrival_tolerance_m",
            "slam.heading_align_timeout_sec", "gimbal.action_timeout_sec",
            "gimbal.yaw_speed_dps", "gimbal.pitch_speed_dps"):
    value = get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 < value < float("inf"):
        raise ValueError("Setting {} must be a finite positive number".format(key))
for key in ("slam.scan_samples", "slam.max_cells", "slam.max_moves", "gimbal.test_cycles"):
    if type(get(key)) is not int or get(key) < 1:
        raise ValueError("Setting {} must be a positive integer".format(key))
if sorted(get("slam.direction_order")) != [0, 1, 2, 3]:
    raise ValueError("slam.direction_order must contain 0, 1, 2, 3 exactly once")
if get("robot.yaw_command_sign") not in (-1, 1):
    raise ValueError("robot.yaw_command_sign must be -1 or 1")
if not -25 <= get("gimbal.pitch_deg") <= 30:
    raise ValueError("gimbal.pitch_deg is outside SDK limits")
if not 0 <= get("slam.wall_thickness_m") < get("navigation.grid_size_m"):
    raise ValueError("Wall thickness must be smaller than a cell")
if get("slam.cell_arrival_tolerance_m") >= get("navigation.grid_size_m") / 2:
    raise ValueError("Cell arrival tolerance must be less than half a cell")
if get("slam.localization_gate_m") >= get("navigation.grid_size_m") / 2:
    raise ValueError("Localization gate must be less than half a cell")
if get("gimbal.settle_sec") < 0:
    raise ValueError("gimbal.settle_sec must not be negative")
if get("sensors.rate_hz") not in (1, 5, 10, 20, 50):
    raise ValueError("Sensor rate must be supported by RoboMaster SDK: 1, 5, 10, 20, 50")

if get("slam.sensor_timeout_sec") <= get("slam.scan_samples") / get("sensors.rate_hz"):
    raise ValueError("Scan sensor timeout must allow enough fresh samples")
for key in ("gimbal.pivot_x_m", "gimbal.pivot_y_m", "gimbal.beam_offset_m"):
    value = get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not -float("inf") < value < float("inf"):
        raise ValueError("Setting {} must be finite".format(key))
for key in ("sensors.history_capacity", "telemetry.buffer_capacity", "sensors.tof_index",
            "sensors.sharp_left.adc_index", "sensors.sharp_right.adc_index"):
    if type(get(key)) is not int or get(key) < (0 if key.endswith("index") else 1):
        raise ValueError("Invalid integer setting: {}".format(key))


def map_geometry():
    """Return validated dimensions and start cell as (row, column)."""
    rows, columns = get("map.rows"), get("map.columns")
    row, column = get("map.start.row"), get("map.start.column")
    if type(rows) is not int or rows < 1 or type(columns) is not int or columns < 1:
        raise ValueError("map.rows and map.columns must be positive integers")
    if type(row) is not int or type(column) is not int or not (0 <= row < rows and 0 <= column < columns):
        raise ValueError("map.start.row/column must be integer cell coordinates inside the map")
    return rows, columns, (row, column)


map_geometry()  # Validate before connecting to hardware.

if not 0 < get("gimbal.move_step_deg") <= 90:
    raise ValueError("gimbal.move_step_deg must be within (0, 90]")

if get("gimbal.scan_mode") not in ("chassis", "gimbal"):
    raise ValueError("gimbal.scan_mode must be chassis or gimbal")

if type(get("robot.yaw_speed_command_sign")) is not int or get("robot.yaw_speed_command_sign") not in (-1, 1):
    raise ValueError("robot.yaw_speed_command_sign must be -1 or 1")
for key in ("navigation.max_heading_error_deg", "navigation.max_lateral_deviation_m"):
    value = get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 < value < float("inf"):
        raise ValueError("Setting {} must be finite and positive".format(key))

for key in ("scan_guard.max_heading_drift_deg", "scan_guard.max_position_drift_m"):
    value = get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 < value < float("inf"):
        raise ValueError("Setting {} must be finite and positive".format(key))

for key in ("slam.stationary_settle_sec", "slam.stationary_timeout_sec"):
    value = get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 < value < float("inf"):
        raise ValueError("Setting {} must be finite and positive".format(key))
if get("slam.stationary_settle_sec") >= get("slam.stationary_timeout_sec"):
    raise ValueError("Stationary settle time must be below stationary timeout")

if not 0 < get("slam.startup_settle_sec") < get("slam.stationary_timeout_sec"):
    raise ValueError("Startup settle time must be positive and below stationary timeout")

if type(get("gimbal.hold_front_on_move")) is not bool:
    raise ValueError("gimbal.hold_front_on_move must be true or false")

value = get("navigation.front_wall_arrival_min_fraction")
if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 < value <= 1:
    raise ValueError("navigation.front_wall_arrival_min_fraction must be within (0, 1]")

for key in ("gimbal.recenter_yaw_speed_dps", "gimbal.recenter_pitch_speed_dps"):
    value = get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 < value <= 360:
        raise ValueError("Setting {} must be in (0, 360] degrees/sec".format(key))
