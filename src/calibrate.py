#!/usr/bin/env python3
"""Step 1 calibration tools for RoboMaster EP sensors.

CSV input columns:
    sensor,raw_value,reference_mm[,sample_id]

Select Calibration from ``python main.py``. Fitting works offline using
measurements already collected from the robot.
"""

try:
    from .settings import get as setting, project_path
    from .sdk_connection import initialize_robot, load_robot_sdk
except ImportError:
    from settings import get as setting, project_path
    from sdk_connection import initialize_robot, load_robot_sdk

import csv
import json
import math
import time
from pathlib import Path


SENSORS = ("sharp_left", "sharp_right", "tof")
DEFAULT_DEGREES = setting("calibration.degrees")


def read_measurements(path):
    rows = []
    with Path(path).open(newline="", encoding="utf-8") as stream:
        for number, row in enumerate(csv.DictReader(stream), start=2):
            raw_text = (row.get("raw_value") or "").strip()
            reference_text = (row.get("reference_mm") or "").strip()
            if not raw_text and not reference_text:
                continue
            try:
                sensor = row["sensor"].strip().lower()
                raw = float(raw_text)
                reference = float(reference_text)
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError("invalid CSV row {}: {}".format(number, exc))
            if sensor not in SENSORS:
                raise ValueError("row {}: sensor must be one of {}".format(number, SENSORS))
            if not math.isfinite(raw) or not math.isfinite(reference) or reference <= 0:
                raise ValueError("row {}: values must be finite and reference_mm > 0".format(number))
            rows.append((sensor, raw, reference))
    if not rows:
        raise ValueError("CSV contains no measurements")
    return rows


def fit_polynomial(rows, degree):
    import numpy as np

    if len(rows) < degree + 1:
        raise ValueError("need at least {} measurements for degree {}".format(degree + 1, degree))
    raw = np.asarray([item[1] for item in rows], dtype=float)
    reference = np.asarray([item[2] for item in rows], dtype=float)
    coefficients = np.polyfit(raw, reference, degree)
    predicted = np.polyval(coefficients, raw)
    residual = reference - predicted
    ss_res = float(np.sum(residual ** 2))
    ss_tot = float(np.sum((reference - np.mean(reference)) ** 2))
    return {
        "degree": degree,
        "coefficients": [float(value) for value in coefficients],
        "rmse_mm": float(np.sqrt(np.mean(residual ** 2))),
        "r2": 1.0 if ss_tot == 0 else 1.0 - ss_res / ss_tot,
        "samples": len(rows),
        "raw_min": float(np.min(raw)),
        "raw_max": float(np.max(raw)),
        "reference_min_mm": float(np.min(reference)),
        "reference_max_mm": float(np.max(reference)),
    }


def plot_sensor(sensor, rows, fit, output):
    import matplotlib.pyplot as plt
    import numpy as np

    raw = np.asarray([item[1] for item in rows])
    reference = np.asarray([item[2] for item in rows])
    order = np.argsort(raw)
    x = np.linspace(raw.min(), raw.max(), 200)
    y = np.polyval(fit["coefficients"], x)
    plt.figure(figsize=(7, 4.5))
    plt.scatter(raw, reference, label="measurement")
    plt.plot(x, y, label="fit (degree {})".format(fit["degree"]))
    plt.xlabel("raw sensor value")
    plt.ylabel("reference distance (mm)")
    plt.title("{} calibration | RMSE {:.2f} mm | R² {:.4f}".format(sensor, fit["rmse_mm"], fit["r2"]))
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output, dpi=160)
    plt.close()


def append_measurement(path, sensor, raw_value, reference_mm, sample_id):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    new_file = not target.exists() or target.stat().st_size == 0
    with target.open("a", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        if new_file:
            writer.writerow(("sensor", "raw_value", "reference_mm", "sample_id"))
        writer.writerow((sensor, raw_value, reference_mm, sample_id))


def _reference_distance(sensor, sample_id):
    while True:
        value = input("{} sample {} reference distance (mm) [q = กลับเมนู]: ".format(sensor, sample_id)).strip()
        if value.lower() == "q":
            return None
        try:
            reference = float(value)
            if math.isfinite(reference) and reference > 0:
                return reference
        except ValueError:
            pass
        print("กรุณาใส่ระยะเป็นตัวเลขมากกว่า 0 หรือ q เพื่อกลับเมนู")


def _close_robot(robot):
    try:
        robot.close()
    except Exception as exc:
        print("[calibration] Cleanup warning: {}".format(exc), file=sys.stderr)


class CalibrationSession:
    """Keep one SDK connection across collection jobs until the menu session ends."""
    def __init__(self, conn_type):
        self.conn_type = conn_type
        self.robot = None

    def connect(self):
        if self.robot is None:
            candidate = load_robot_sdk().Robot()
            try:
                initialize_robot(candidate, self.conn_type)
            except BaseException:
                _close_robot(candidate)
                raise
            self.robot = candidate
        return self.robot

    def collect(self, sensor, output, board_id, port, tof_index, samples, reference_provider=None):
        return collect_live(sensor, output, board_id, port, tof_index, samples,
                            self.conn_type, ep_robot=self.connect(),
                            reference_provider=reference_provider)

    def close(self):
        if self.robot is not None:
            robot, self.robot = self.robot, None
            _close_robot(robot)


def collect_live(sensor, output, board_id, port, tof_index, samples, conn_type,
                 ep_robot=None, reference_provider=None):
    """Collect samples; q stops the job. Borrowed session connections stay open."""
    if sensor not in ("sharp_left", "sharp_right", "tof"):
        raise ValueError("live collection supports sharp_left, sharp_right, and tof")
    owns_robot = ep_robot is None
    if owns_robot:
        ep_robot = load_robot_sdk().Robot()
    latest_tof = [None]
    callback_error = [None]
    subscribed = False
    collected = 0

    def tof_callback(distance):
        if not isinstance(distance, (list, tuple)) or not 0 <= tof_index < len(distance):
            callback_error[0] = "tof index {} is not present in {}".format(tof_index, distance)
            return
        latest_tof[0] = distance[tof_index]

    try:
        if owns_robot:
            initialize_robot(ep_robot, conn_type)
        if sensor == "tof":
            subscribed = True  # Cleanup even if a subscription fails partway through.
            result = ep_robot.sensor.sub_distance(freq=10, callback=tof_callback)
            if result is False:
                raise RuntimeError("ToF subscription failed")
            time.sleep(0.5)
        else:
            sensor_id = board_id if board_id is not None else setting("sensors." + sensor + ".board_id")
            sensor_port = port if port is not None else setting("sensors." + sensor + ".port")
        for sample_id in range(1, samples + 1):
            reference = (reference_provider(sensor, sample_id) if reference_provider
                         else _reference_distance(sensor, sample_id))
            if reference is None:
                print("[calibration] หยุดเก็บค่า บันทึกแล้ว {} ตัวอย่าง".format(collected))
                return collected
            if sensor == "tof":
                time.sleep(0.2)
                if callback_error[0]:
                    raise RuntimeError(callback_error[0])
                raw = latest_tof[0]
                if raw is None:
                    raise RuntimeError("no ToF callback value received")
            else:
                raw = ep_robot.sensor_adaptor.get_adc(id=sensor_id, port=sensor_port)
                if raw is None:
                    raise RuntimeError("sensor adapter returned no ADC value")
            append_measurement(output, sensor, raw, reference, sample_id)
            collected += 1
            print("saved raw={} reference={}mm".format(raw, reference))
        return collected
    finally:
        if subscribed:
            try:
                ep_robot.sensor.unsub_distance()
            except Exception as exc:
                print("[calibration] ToF unsubscribe warning: {}".format(exc), file=sys.stderr)
        if owns_robot:
            _close_robot(ep_robot)


def fit_command(input_path, output_dir):
    rows = read_measurements(input_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    calibration = {"schema": 1, "source_csv": str(input_path), "sensors": {}}
    for sensor in SENSORS:
        sensor_rows = [row for row in rows if row[0] == sensor]
        if not sensor_rows:
            continue
        fit = fit_polynomial(sensor_rows, DEFAULT_DEGREES[sensor])
        calibration["sensors"][sensor] = fit
        plot_sensor(sensor, sensor_rows, fit, output_dir / (sensor + "_calibration.png"))
    if not calibration["sensors"]:
        raise ValueError("CSV has no supported sensor measurements")
    result = output_dir / "calibration.json"
    result.write_text(json.dumps(calibration, indent=2) + "\n", encoding="utf-8")
    print("wrote {}".format(result))
    for sensor, fit in calibration["sensors"].items():
        print("{}: {} samples, RMSE {:.2f} mm, R² {:.4f}".format(sensor, fit["samples"], fit["rmse_mm"], fit["r2"]))
