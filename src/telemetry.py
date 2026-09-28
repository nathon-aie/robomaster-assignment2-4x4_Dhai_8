#!/usr/bin/env python3
"""Telemetry recorder and post-run analysis tools for RoboMaster EP.

Records time-series sensor data from Thread 1, exports JSON and CSV runs,
and generates statistical reports & matplotlib analysis charts.
"""

try:
    from .settings import get as setting, project_path
except ImportError:
    from settings import get as setting, project_path

import csv
import json
import math
import os
import re
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


def get_next_run_number(base_dir: Path, prefix: str = "run") -> int:
    """Finds the next sequential run number by scanning existing subdirectories (e.g. run1, run2, ...)."""
    if not base_dir.exists():
        return 1

    pattern = re.compile(rf"^{re.escape(prefix)}(\d+)$", re.IGNORECASE)
    run_numbers = []
    for entry in base_dir.iterdir():
        if entry.is_dir():
            match = pattern.match(entry.name)
            if match:
                try:
                    run_numbers.append(int(match.group(1)))
                except ValueError:
                    pass

    return max(run_numbers) + 1 if run_numbers else 1


def write_telemetry_csv(records, csv_path):
    """Write sensor records in the same column order as their JSON snapshots."""
    path = Path(csv_path)
    with path.open('w', newline='', encoding='utf-8') as stream:
        if records:
            writer = csv.DictWriter(stream, fieldnames=list(records[0]))
            writer.writeheader()
            writer.writerows(records)
    return path


class TelemetryRecorder:
    """Thread-safe time-series telemetry logger for online mapping & post-run analysis.

    Saves session data into sequential run directories (run1, run2, run3, ...) inside base_dir,
    with files timestamped (e.g. run1_20260826_133706.json).
    """

    def __init__(
        self,
        output_dir: str = project_path("paths.telemetry"),
        buffer_capacity: int = setting("telemetry.buffer_capacity"),
        run_name: Optional[str] = None,
    ):
        self.base_dir = Path(output_dir)
        self.base_dir.mkdir(parents=True, exist_ok=True)
        self.buffer_capacity = buffer_capacity
        self._lock = threading.Lock()
        self._records: List[Dict[str, Any]] = []
        self._start_time = time.time()
        self.timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")

        if run_name:
            self.run_name = run_name
        else:
            next_num = get_next_run_number(self.base_dir, prefix="run")
            self.run_name = f"run{next_num}"

        self.run_dir = self.base_dir / self.run_name
        self.output_dir = self.run_dir  # For backward compatibility
        self._session_id = f"{self.run_name}_{self.timestamp_str}"

    def record_snapshot(self, snapshot: Any):
        """Thread-safe push of sensor snapshot."""
        data = snapshot.to_dict() if hasattr(snapshot, "to_dict") else dict(snapshot)
        data["elapsed_sec"] = time.time() - self._start_time
        with self._lock:
            if len(self._records) < self.buffer_capacity:
                self._records.append(data)

    def get_records(self) -> List[Dict[str, Any]]:
        with self._lock:
            return list(self._records)

    def export(self, custom_name: Optional[str] = None) -> Path:
        """Save sensor records as JSON and CSV inside the run directory."""
        with self._lock:
            records = list(self._records)

        self.run_dir.mkdir(parents=True, exist_ok=True)
        base_filename = custom_name if custom_name else self.run_name
        name = f"{base_filename}_{self.timestamp_str}"
        json_path = self.run_dir / f"{name}.json"

        # Export JSON
        summary = {
            "session_id": self._session_id,
            "run_folder": self.run_name,
            "recorded_at": datetime.now().isoformat(),
            "sample_count": len(records),
            "duration_sec": records[-1]["elapsed_sec"] if records else 0.0,
            "records": records,
        }
        with json_path.open("w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)

        csv_path = write_telemetry_csv(records, self.run_dir / f"{name}.csv")
        print(f"[TelemetryRecorder] Saved {len(records)} samples -> {json_path} & {csv_path}")
        return json_path


class TelemetryAnalyzer:
    """Post-run analysis tool: computes sensor health, stability, stats, and plots."""

    @staticmethod
    def analyze_file(file_or_dir_path: str, save_plot: bool = True) -> Dict[str, Any]:
        path = Path(file_or_dir_path)
        if path.is_dir():
            # If a folder was given (e.g. telemetry_logs/run1), find the matching JSON file
            json_candidates = sorted(p for p in path.glob("*.json") if not p.stem.endswith("_map"))
            if not json_candidates:
                print(f"[TelemetryAnalyzer] No .json files found in directory: {path}")
                return {"error": f"No .json files found in {path}"}
            # Prefer JSON matching prefix (e.g. run1_*.json or run1.json)
            matching = [p for p in json_candidates if p.stem.startswith(path.name)]
            path = matching[-1] if matching else json_candidates[-1]

        if not path.exists():
            print(f"[TelemetryAnalyzer] File does not exist: {path}")
            return {"error": f"File not found: {path}"}

        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)

        records = data.get("records", [])
        if not records:
            return {"error": "No records in telemetry file"}

        def calc_stats(values: List[Optional[float]]) -> Dict[str, float]:
            clean = [float(v) for v in values if v is not None and math.isfinite(v)]
            if not clean:
                return {"count": 0, "mean": 0.0, "std": 0.0, "min": 0.0, "max": 0.0}
            mean = sum(clean) / len(clean)
            variance = sum((x - mean) ** 2 for x in clean) / len(clean)
            return {
                "count": len(clean),
                "mean": round(mean, 2),
                "std": round(math.sqrt(variance), 2),
                "min": round(min(clean), 2),
                "max": round(max(clean), 2),
            }

        sl_mm = [r.get("sharp_left_mm") for r in records]
        sr_mm = [r.get("sharp_right_mm") for r in records]
        tof_mm = [r.get("tof_filtered_mm") for r in records]
        diff_mm = [r.get("sharp_diff_mm") for r in records]
        yaws = [r.get("yaw", 0.0) for r in records]

        stats = {
            "session_id": data.get("session_id"),
            "total_duration_sec": round(records[-1].get("elapsed_sec", 0.0), 2),
            "sample_count": len(records),
            "sample_rate_hz": round(len(records) / max(0.01, records[-1].get("elapsed_sec", 1.0)), 1),
            "sharp_left_mm_stats": calc_stats(sl_mm),
            "sharp_right_mm_stats": calc_stats(sr_mm),
            "tof_mm_stats": calc_stats(tof_mm),
            "sharp_diff_stats": calc_stats(diff_mm),
            "yaw_stats": calc_stats(yaws),
        }

        # Print summary table
        print("\n" + "=" * 60)
        print(f"📊 POST-RUN TELEMETRY ANALYSIS: {path.name}")
        print("=" * 60)
        print(f"Duration: {stats['total_duration_sec']} s | Samples: {stats['sample_count']} ({stats['sample_rate_hz']} Hz)")
        print("-" * 60)
        print(f"{'Sensor':<18} | {'Mean (mm)':<10} | {'Std (mm)':<10} | {'Min':<8} | {'Max':<8}")
        print("-" * 60)
        for name, key in [("Sharp Left", "sharp_left_mm_stats"), ("Sharp Right", "sharp_right_mm_stats"), ("ToF Front", "tof_mm_stats"), ("Sharp Diff |L-R|", "sharp_diff_stats")]:
            s = stats[key]
            print(f"{name:<18} | {s['mean']:<10} | {s['std']:<10} | {s['min']:<8} | {s['max']:<8}")
        print("=" * 60 + "\n")

        if save_plot:
            plot_path = path.parent / f"{path.stem}_plot.png"
            TelemetryAnalyzer.generate_plots(records, str(plot_path))
            stats["plot_path"] = str(plot_path)

        return stats

    @staticmethod
    def generate_plots(records: List[Dict[str, Any]], output_path: str):
        """Generates a 4-panel analysis dashboard plot."""
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("[TelemetryAnalyzer] Matplotlib not installed; skipping plot generation.")
            return

        t = [r.get("elapsed_sec", 0.0) for r in records]
        sl_raw = [r.get("sharp_left_raw") for r in records]
        sl_mm = [r.get("sharp_left_mm") for r in records]
        sr_mm = [r.get("sharp_right_mm") for r in records]
        tof_mm = [r.get("tof_filtered_mm") for r in records]
        yaw = [r.get("yaw", 0.0) for r in records]
        pos_x = [r.get("pos_x", 0.0) for r in records]
        pos_y = [r.get("pos_y", 0.0) for r in records]

        fig, axs = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle("RoboMaster EP Step 2 Multi-Threading Telemetry Analysis", fontsize=14, fontweight="bold")

        # 1. Sharp Left & Right Distances
        ax1 = axs[0, 0]
        ax1.plot(t, sl_mm, label="Sharp Left (mm)", color="tab:blue")
        ax1.plot(t, sr_mm, label="Sharp Right (mm)", color="tab:orange")
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("Distance (mm)")
        ax1.set_title("Sharp IR Sensors (Calibrated & Filtered)")
        ax1.grid(True, alpha=0.3)
        ax1.legend()

        # 2. ToF Front Distance
        ax2 = axs[0, 1]
        ax2.plot(t, tof_mm, label="ToF Distance (mm)", color="tab:green")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Distance (mm)")
        ax2.set_title("Front ToF Sensor")
        ax2.grid(True, alpha=0.3)
        ax2.legend()

        # 3. Robot Yaw / Heading
        ax3 = axs[1, 0]
        ax3.plot(t, yaw, label="Yaw Angle (deg)", color="tab:purple")
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("Angle (deg)")
        ax3.set_title("Chassis Attitude / Yaw")
        ax3.grid(True, alpha=0.3)
        ax3.legend()

        # 4. Robot 2D Trajectory / Odometry
        ax4 = axs[1, 1]
        ax4.plot(pos_x, pos_y, "o-", markersize=3, label="Trajectory (X-Y)", color="tab:red")
        ax4.set_xlabel("X Position (m)")
        ax4.set_ylabel("Y Position (m)")
        ax4.set_title("Chassis 2D Odometry Trace")
        ax4.grid(True, alpha=0.3)
        ax4.axis("equal")
        ax4.legend()

        plt.tight_layout()
        plt.savefig(output_path, dpi=160)
        plt.close()
        print(f"[TelemetryAnalyzer] Analysis plot generated -> {output_path}")
