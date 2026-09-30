#!/usr/bin/env python3
"""Master Robot System Coordinator for RoboMaster EP Multi-Threading.

Manages Robot connection, Thread 1 (Sensor Collection & Filtering),
Thread 2 (Robot Motion Controller), and Telemetry Logging.
"""

try:
    from .settings import get as setting, project_path
    from .sdk_connection import cancel_chassis_speed_timer, initialize_robot, load_robot_sdk
except ImportError:
    from settings import get as setting, project_path
    from sdk_connection import cancel_chassis_speed_timer, initialize_robot, load_robot_sdk

import time
from pathlib import Path
from typing import Optional

try:
    from .robot_controller import RobotControllerThread
    from .sensor_pipeline import CalibrationManager, SensorCollectorThread, SensorHub
    from .telemetry import TelemetryAnalyzer, TelemetryRecorder
except (ImportError, ValueError):
    from robot_controller import RobotControllerThread
    from sensor_pipeline import CalibrationManager, SensorCollectorThread, SensorHub
    from telemetry import TelemetryAnalyzer, TelemetryRecorder


class RobotSystem:
    """Master orchestrator for Step 2 Multi-Threading Architecture."""

    def __init__(
        self,
        calibration_file: str = project_path("paths.calibration"),
        telemetry_dir: str = project_path("paths.telemetry"),
        sensor_rate_hz: float = setting("sensors.rate_hz"),
        conn_type: str = setting("robot.conn_type"),
        results_dir=None,
    ):
        self.conn_type = conn_type
        self.robot = None

        # Core subsystems
        self.calibration_mgr = CalibrationManager(calibration_file)
        if results_dir is None:
            self.telemetry = TelemetryRecorder(output_dir=telemetry_dir)
        else:
            results_dir = Path(results_dir)
            self.telemetry = TelemetryRecorder(
                output_dir=results_dir.parent, run_name=results_dir.name
            )
        self.sensor_hub = SensorHub(max_history=setting("sensors.history_capacity"))

        # Multi-threading workers
        self.thread_1_sensor: Optional[SensorCollectorThread] = None
        self.thread_2_controller: Optional[RobotControllerThread] = None
        self.sensor_rate_hz = sensor_rate_hz

    def connect_robot(self) -> bool:
        """Initializes connection to RoboMaster EP hardware."""
        print(f"[RobotSystem] Connecting to RoboMaster EP via {self.conn_type.upper()}...")
        try:
            robot_mod = load_robot_sdk()
            self.robot = robot_mod.Robot()
            initialize_robot(self.robot, self.conn_type)
            if self.robot.set_robot_mode(mode=robot_mod.CHASSIS_LEAD) is False:
                raise RuntimeError("Cannot set CHASSIS_LEAD mode")
            print("[RobotSystem] Successfully connected to RoboMaster EP!")
            return True
        except Exception as exc:
            print(f"[RobotSystem] Connection failed: {exc}")
            if self.robot is not None:
                try:
                    self.robot.close()
                except Exception as close_error:
                    print("[RobotSystem] Cleanup warning: {}".format(close_error))
            self.robot = None
            return False

    def setup_threads(self):
        """Spawns Thread 1 and Thread 2 with thread-safe shared memory."""
        if self.robot is None:
            raise RuntimeError("Robot is not connected")
        # Thread 1: Sensor Collection + Filtering
        self.thread_1_sensor = SensorCollectorThread(
            sensor_hub=self.sensor_hub,
            robot=self.robot,
            calibration_manager=self.calibration_mgr,
            telemetry_recorder=self.telemetry,
            update_rate_hz=self.sensor_rate_hz,
        )

        # Thread 2: Robot Motion Controller
        self.thread_2_controller = RobotControllerThread(
            sensor_hub=self.sensor_hub,
            robot=self.robot,
        )

    def start(self):
        """Starts both Thread 1 and Thread 2."""
        if not self.thread_1_sensor or not self.thread_2_controller:
            self.setup_threads()

        print("[RobotSystem] Starting Thread 1 (Sensor Collection & Filtering)...")
        self.thread_1_sensor.start_collecting()

        # Let sensor buffers warm up
        time.sleep(setting("system.sensor_warmup_sec"))

        print("[RobotSystem] Starting Thread 2 (Robot Motion Controller)...")
        self.thread_2_controller.start_running()

    def wait_for_completion(self, timeout: Optional[float] = None) -> bool:
        """Blocks until Thread 2 finishes all queued commands or timeout."""
        start_t = time.time()
        while True:
            if self.thread_2_controller and self.thread_2_controller.failure:
                raise RuntimeError(self.thread_2_controller.failure)
            if self.thread_2_controller and self.thread_2_controller.commands_completed:
                return True
            if timeout and (time.time() - start_t) > timeout:
                return False
            time.sleep(0.1)

    def shutdown(self, save_telemetry: bool = True, run_analysis: bool = True):
        """Gracefully shuts down both threads and exports run telemetry."""
        print("\n[RobotSystem] Shutting down multi-threading workers...")
        try:
            for worker, stop in ((self.thread_2_controller, 'stop_running'),
                                 (self.thread_1_sensor, 'stop_collecting')):
                if worker is not None:
                    try:
                        getattr(worker, stop)()
                    except Exception as exc:
                        print('[RobotSystem] {} warning: {}'.format(stop, exc))

            for worker in (self.thread_2_controller, self.thread_1_sensor):
                if worker is not None and worker.is_alive():
                    try:
                        worker.join(timeout=setting("gimbal.action_timeout_sec") + 1)
                    except Exception as exc:
                        print('[RobotSystem] Worker join warning: {}'.format(exc))
            if self.robot is not None:
                try:
                    cancel_chassis_speed_timer(self.robot.chassis)
                except Exception as exc:
                    print("[RobotSystem] Chassis timer cleanup warning: {}".format(exc))
                try:
                    self.robot.close()
                    print("[RobotSystem] RoboMaster SDK connection closed.")
                except Exception as exc:
                    print('[RobotSystem] SDK close warning: {}'.format(exc))
        finally:
            if save_telemetry:
                json_p = self.telemetry.export()
                if run_analysis:
                    TelemetryAnalyzer.analyze_file(str(json_p), save_plot=True)
