#!/usr/bin/env python3
"""Thread 2: Robot Controller and Grid-by-Grid Motion Execution with PID Control.

Step 3 Requirements (WORK.md):
- เดินทีละ Grid (60x60 cm, Wall 7.5 cm)
- PID Control ปรับการเคลื่อนที่แกน Y ให้อยู่ตรงกลางระหว่างกำแพง (ดึงค่าจาก Thread 1)
- 8 Wall Alignment Cases (มี/ไม่มีกำแพงหน้า x 2ข้าง/ซ้าย/ขวา/ไม่มี)
"""

try:
    from .settings import get as setting, project_path
    from .sdk_connection import cancel_chassis_speed_timer, chassis_speed_has_no_ack
except ImportError:
    from settings import get as setting, project_path
    from sdk_connection import cancel_chassis_speed_timer, chassis_speed_has_no_ack

from dataclasses import replace
import math
import threading
import time
from typing import Any, List

try:
    from .pid_controller import WallCenteringPID
    from .sensor_pipeline import RobotSensorSnapshot, SensorHub
except (ImportError, ValueError):
    from pid_controller import WallCenteringPID
    from sensor_pipeline import RobotSensorSnapshot, SensorHub


class RobotControllerThread(threading.Thread):
    """Thread 2: Consumes filtered sensor data from Thread 1 (SensorHub) and
    executes step-by-step Grid navigation with PID centering.
    """

    def __init__(
        self,
        sensor_hub: SensorHub,
        robot: Any = None,
        grid_size_m: float = setting("navigation.grid_size_m"),
        nominal_side_dist_mm: float = setting("navigation.nominal_side_mm"),
        base_speed: float = setting("navigation.base_speed_mps"),
    ):
        super().__init__(name="RobotControllerThread-2", daemon=True)
        self.sensor_hub = sensor_hub
        self.robot = robot
        self.grid_size_m = grid_size_m
        self.base_speed = base_speed
        self.target_heading_deg = 0.0
        self.yaw_speed_command_sign = setting("robot.yaw_speed_command_sign")
        self.strict_sensors = False
        self.recovery_wait_total_sec = 0.0
        self.front_ready = False
        self.calibration_manager = None

        self._running = threading.Event()
        self._pause_event = threading.Event()
        self._pause_event.set()

        # Step 3 PID controller for 8 wall cases
        self.wall_pid = WallCenteringPID(
            nominal_side_dist_mm=nominal_side_dist_mm,
            tolerance_mm=setting("navigation.tolerance_mm"),  # 2cm tolerance as per REQ
        )

        self.command_queue: List[str] = []
        self.current_action: str = "IDLE"
        self.current_step: int = 0
        self.commands_completed: bool = False
        self.failure = None
        self.step_pause_sec: float = setting("navigation.step_pause_sec")  # 1.0s pause between states on live robot

    def set_commands(self, commands: List[str]):
        self.command_queue = list(commands)

    # -----------------------------------------------------------------------
    # Actuator Low-level Drivers
    # -----------------------------------------------------------------------

    def drive_speed(self, vx: float, vy: float, vz: float):
        """Drives robot chassis with holonomic velocities (m/s, m/s, deg/s)."""
        try:
            # RoboMaster EP chassis drive_speed
            # Note: drive_speed accepts x=vx(m/s), y=vy(m/s), z=vz(deg/s)
            sent = self.robot.chassis.drive_speed(x=vx, y=vy, z=vz * self.yaw_speed_command_sign,
                                                  timeout=setting("slam.max_sensor_age_sec"))
            if sent is False and not chassis_speed_has_no_ack(self.robot.chassis):
                raise RuntimeError("Chassis rejected drive command")
            # The SDK speed protocol is PUSH/no ACK: its False result is
            # not an explicit rejection. Sensor freshness, heading feedback,
            # front braking and odometry timeout still verify actual motion.
        except Exception as e:
            raise RuntimeError("Chassis drive failed: {}".format(e)) from e

    def stop_chassis(self):
        """Stops chassis motors."""
        try:
            self.robot.chassis.drive_speed(x=0, y=0, z=0)
        except Exception:
            pass

    def brake_chassis(self):
        """Send zero speed immediately, then require a wheel-stop ACK."""
        self.stop_chassis()
        acknowledged = self.robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        cancel_chassis_speed_timer(self.robot.chassis)
        if acknowledged is not True:
            raise RuntimeError("Chassis brake was not acknowledged")

    # -----------------------------------------------------------------------
    # Step 3: Grid-by-Grid Navigation & PID Centering
    # -----------------------------------------------------------------------

    def motion_state(self):
        state = self.sensor_hub.get_latest_state()
        if not self.strict_sensors:
            return state
        now = time.monotonic()
        age = setting("slam.max_sensor_age_sec")
        for name in ("tof_received_at", "position_received_at", "attitude_received_at", "gimbal_received_at", "sharp_left_received_at", "sharp_right_received_at"):
            if not 0 < getattr(state, name) <= now or now - getattr(state, name) > age:
                raise RuntimeError("Stale sensor stream: {}".format(name))
        if not all(math.isfinite(v) for v in (state.pos_x, state.pos_y, state.yaw, state.gimbal_yaw, state.gimbal_pitch)):
            raise RuntimeError("Non-finite pose or Gimbal angle")
        if abs(state.gimbal_yaw) > setting("gimbal.front_yaw_tolerance_deg"):
            raise RuntimeError("Gimbal no longer faces chassis front during motion")
        raw_tof = state.tof_raw
        far_out_of_range = (isinstance(raw_tof, (int, float))
                            and math.isfinite(raw_tof)
                            and raw_tof > setting("sensors.tof_filter.max_valid"))
        if not self.front_ready or not (state.tof_valid or far_out_of_range):
            raise RuntimeError("ToF is not ready and facing forward")
        distance = self.calibration_manager.raw_to_mm("tof", raw_tof)
        if distance is None or not math.isfinite(distance) or distance <= 0:
            raise RuntimeError("Invalid front ToF")
        filtered = state.tof_filtered_mm
        if filtered is None or not math.isfinite(filtered) or filtered <= 0:
            raise RuntimeError("Invalid filtered front ToF")
        # A fresh reading beyond the far limit means no nearby wall. Keep the
        # last filtered distance for braking, including if it is near.
        # Close invalid readings and stale packets still fail above.
        return replace(state, tof_valid=True, tof_filtered_mm=min(distance, filtered))

    def confirm_front_clear(self, last_tof_at, clear_distance_mm):
        """Keep the chassis stopped until two new ToF packets show a clear path."""
        self.stop_chassis()
        deadline = time.monotonic() + setting("navigation.obstacle_recheck_timeout_sec")
        clear_samples = 0
        while self._running.is_set() and time.monotonic() < deadline:
            state = self.motion_state()
            if state.tof_received_at > last_tof_at:
                last_tof_at = state.tof_received_at
                if state.tof_valid and state.tof_filtered_mm > clear_distance_mm:
                    clear_samples += 1
                    if clear_samples >= 2:
                        return True
                else:
                    clear_samples = 0
            time.sleep(1.0 / setting("navigation.control_rate_hz"))
        return False

    def align_at_cell_center(self, duration_sec: float = setting("navigation.align_default_duration_sec")):
        """In-place PID fine alignment to ensure robot is centered (|L-R| < 2cm or L/R +- 2cm)."""
        t_end = time.monotonic() + duration_sec
        self.wall_pid.reset()

        while time.monotonic() < t_end and self._running.is_set():
            state = self.motion_state()
            _, vy, vz, case_name, case_id, err_y = self.wall_pid.compute_control_speeds(
                state=state,
                target_yaw_deg=self.target_heading_deg,
                base_vx=0.0,
                dt=1.0 / setting("navigation.control_rate_hz"),
            )

            # If error is within 20mm deadband, vy will be 0.0
            if abs(err_y) < self.wall_pid.tolerance_mm and abs(vz) < 1.0:
                # Already centered!
                self.stop_chassis()
                break

            self.drive_speed(vx=0.0, vy=vy, vz=vz)
            time.sleep(1.0 / setting("navigation.control_rate_hz"))

        self.stop_chassis()

    def navigate_single_grid_step(self, step_idx: int = 1, total_steps: int = 1):
        try:
            return self._navigate_single_grid_step(step_idx, total_steps)
        finally:
            self.stop_chassis()

    def _navigate_single_grid_step(self, step_idx: int = 1, total_steps: int = 1):
        """Navigates exactly 1 grid cell (60 cm) using closed-loop PID lateral centering."""
        self.current_action = f"NAVIGATE_GRID_{step_idx}_OF_{total_steps}"
        print(f"\n  [Grid Step {step_idx}/{total_steps}] Moving 1 cell forward ({self.grid_size_m:.2f} m)...")

        # Snapshot start position & orientation from Thread 1
        initial_state = self.motion_state()
        start_x, start_y = initial_state.pos_x, initial_state.pos_y

        self.wall_pid.reset()
        dist_traveled = 0.0
        control_loop_hz = setting("navigation.control_rate_hz")
        dt = 1.0 / control_loop_hz
        max_duration = (self.grid_size_m / max(0.1, self.base_speed)) * setting("navigation.timeout_multiplier") + setting("navigation.timeout_extra_sec")
        t_start = time.monotonic()
        initial_recovery_wait = self.recovery_wait_total_sec

        last_case_id = None
        reason = "interrupted"
        false_obstacle_count = 0
        stop_tof_mm = None

        while dist_traveled < self.grid_size_m and self._running.is_set():
            loop_t0 = time.monotonic()
            if (loop_t0 - t_start - (self.recovery_wait_total_sec - initial_recovery_wait)) > max_duration:
                print(f"  [Warning] Grid step reached timeout limit ({max_duration:.1f}s).")
                reason = "timeout"
                break

            # 1. Pull clean, pre-filtered sensor snapshot from Thread 1 (Zero hardware overhead)
            state = self.motion_state()
            if (state.tof_valid and state.tof_filtered_mm is not None
                    and state.tof_filtered_mm <= setting("navigation.emergency_front_mm")):
                stop_tof_mm = state.tof_filtered_mm
                self.brake_chassis()
                reason = "emergency_obstacle"
                break

            # 2. Update forward distance traveled along target heading (60 cm / 0.60 m)
            dx = state.pos_x - start_x
            dy = state.pos_y - start_y
            rad = math.radians(self.target_heading_deg)
            forward_step_m = dx * math.cos(rad) + dy * math.sin(rad)
            dist_traveled = max(0.0, forward_step_m)
            lateral_step_m = -dx * math.sin(rad) + dy * math.cos(rad)
            heading_error = (state.yaw - self.target_heading_deg + 180.0) % 360.0 - 180.0
            if not all(math.isfinite(v) for v in (dist_traveled, lateral_step_m, heading_error)):
                reason = "invalid_pose"
                break
            if abs(heading_error) > setting("navigation.max_heading_error_deg"):
                reason = "heading_deviation"
                break
            if abs(lateral_step_m) > setting("navigation.max_lateral_deviation_m"):
                reason = "lateral_deviation"
                break

            # 3. Check remaining distance to grid cell boundary (60 cm)
            rem_dist = self.grid_size_m - dist_traveled
            cur_vx = self.base_speed
            if rem_dist < setting("navigation.end_deceleration_m"):
                # Decelerate smoothly at end of 60cm cell
                cur_vx = max(setting("navigation.minimum_forward_speed_mps"), self.base_speed * (rem_dist / setting("navigation.end_deceleration_m")))

            # 4. Compute PID control commands for the 8 wall cases
            vx, vy, vz, case_name, case_id, err_y = self.wall_pid.compute_control_speeds(
                state=state,
                target_yaw_deg=self.target_heading_deg,
                base_vx=cur_vx,
                dt=dt,
            )

            if case_id != last_case_id:
                print(f"  [PID Centering] {case_name} | Lat Err: {err_y:+.1f} mm | vy: {vy:+.2f} m/s | L: {state.sharp_left_mm} mm | R: {state.sharp_right_mm} mm")
                last_case_id = case_id

            stop_distance = self.wall_pid.front_target_mm + setting("navigation.front_stop_tolerance_mm")
            if state.tof_valid and state.tof_filtered_mm is not None:
                premature_wall = (state.tof_filtered_mm <= stop_distance
                                  and dist_traveled < self.grid_size_m
                                  * setting("navigation.front_wall_arrival_min_fraction"))
                if premature_wall:
                    self.brake_chassis()
                    if (false_obstacle_count < 2
                            and self.confirm_front_clear(state.tof_received_at, stop_distance)):
                        false_obstacle_count += 1
                        # Time spent stopped for verification is not travel time.
                        t_start += time.monotonic() - loop_t0
                        continue
                    stop_tof_mm = state.tof_filtered_mm
                    reason = "front_wall"
                    break
            if state.tof_valid and state.tof_filtered_mm is not None:
                if state.tof_filtered_mm <= stop_distance:
                    stop_tof_mm = state.tof_filtered_mm
                    self.brake_chassis()
                    reason = "front_wall"
                    break
            if dist_traveled >= self.grid_size_m:
                reason = "distance_reached"
                break
            self.drive_speed(vx=vx, vy=vy, vz=vz)

            # Sleep remaining loop dt
            loop_elapsed = time.monotonic() - loop_t0
            if dt > loop_elapsed:
                time.sleep(dt - loop_elapsed)

        self.stop_chassis()

        def arrival_errors(state):
            dx, dy = state.pos_x - start_x, state.pos_y - start_y
            rad = math.radians(self.target_heading_deg)
            return (dx * math.cos(rad) + dy * math.sin(rad),
                    -dx * math.sin(rad) + dy * math.cos(rad),
                    (state.yaw - self.target_heading_deg + 180.0) % 360.0 - 180.0)

        end_state = self.motion_state()
        emergency_limit = setting("navigation.emergency_front_mm")
        if (end_state.tof_valid and end_state.tof_filtered_mm is not None
                and end_state.tof_filtered_mm <= emergency_limit):
            self.brake_chassis()
            reason = "emergency_obstacle"
        forward, lateral, heading_error = arrival_errors(end_state)
        def arrived(forward_distance):
            if reason == "front_wall":
                # ToF defines the stopping position near the far end of a cell.
                # Reject a wall at the start or motion beyond the cell limit.
                return (self.grid_size_m * setting("navigation.front_wall_arrival_min_fraction")
                        <= forward_distance <= self.grid_size_m + setting("slam.cell_arrival_tolerance_m"))
            return (reason == "distance_reached" and abs(forward_distance - self.grid_size_m)
                    <= setting("slam.cell_arrival_tolerance_m"))

        completed = self._running.is_set() and arrived(forward)
        if completed and reason != "front_wall":
            self.align_at_cell_center(duration_sec=setting("navigation.align_duration_sec"))
            end_state = self.motion_state()
            forward, lateral, heading_error = arrival_errors(end_state)
            completed = self._running.is_set() and arrived(forward)
            if (end_state.tof_valid and end_state.tof_filtered_mm is not None
                    and end_state.tof_filtered_mm <= emergency_limit):
                self.brake_chassis()
                reason = "emergency_obstacle"
                completed = False

        if not completed and reason == "distance_reached":
            reason = "arrival_pose_outside_tolerance"

        # Verify the final pose again after centering, not only forward distance.
        diff_str = f"{end_state.sharp_diff_mm:+.1f} mm" if (end_state.wall_left_detected and end_state.wall_right_detected) else "N/A"
        print(f"  [Grid Step {step_idx}/{total_steps} Done] Local Pos: ({end_state.pos_x:+.2f}m, {end_state.pos_y:+.2f}m) | Yaw: {end_state.yaw:+.1f}° | Sharp L: {end_state.sharp_left_mm} mm | R: {end_state.sharp_right_mm} mm | Diff: {diff_str} | ToF: {end_state.tof_filtered_mm} mm")

        return {"completed": completed, "reason": reason, "distance_m": forward,
                "false_obstacle_count": false_obstacle_count,
                "stop_tof_mm": stop_tof_mm, "end_tof_mm": end_state.tof_filtered_mm,
                "front_stop_limit_mm": self.wall_pid.front_target_mm + setting("navigation.front_stop_tolerance_mm"),
                "lateral_deviation_m": lateral, "heading_error_deg": heading_error}

    def move_forward_grid(self, cells: int = 1):
        """Executes multi-cell forward motion grid-by-grid with closed-loop PID centering."""
        print(f"\n[Controller] Starting {cells}-Grid Forward Motion with Step 3 PID...")
        for i in range(1, cells + 1):
            if not self._running.is_set():
                break
            result = self.navigate_single_grid_step(step_idx=i, total_steps=cells)
            if not result["completed"]:
                raise RuntimeError("Grid motion failed: {}".format(result["reason"]))
            if i < cells and self.step_pause_sec > 0:
                print(f"[Controller] ⏸️ Pausing {self.step_pause_sec:.1f}s before next grid step...")
                time.sleep(self.step_pause_sec)

    def turn_to_relative(self, deg: float, speed: float = setting("navigation.turn_speed_dps"),
                         tolerance_deg=None):
        """Closed-loop relative in-place turn (+90 Left, -90 Right, 180 Around)."""
        # In DJI SDK: z=+90 rotates CCW (yaw becomes -90°), z=-90 rotates CW (yaw becomes +90°)
        expected_yaw_delta = deg * setting("robot.yaw_command_sign")
        target_heading = (self.target_heading_deg + expected_yaw_delta + 180.0) % 360.0 - 180.0
        self.current_action = f"TURN_{deg:+.0f}_DEG"
        dir_name = "Left (เลี้ยวซ้าย z=+90)" if deg > 0 else ("Right (เลี้ยวขวา z=-90)" if deg < 0 else "Around (กลับหลัง z=180)")
        print(f"\n[Controller] 🔄 Executing Turn {dir_name}: z={deg:+.0f}° -> Target Heading: {target_heading:.0f}°...")

        try:
            action = self.robot.chassis.move(x=0, y=0, z=deg, z_speed=speed)
            if not action.wait_for_completed(timeout=setting("gimbal.action_timeout_sec")) or not action.has_succeeded:
                raise RuntimeError("Chassis turn failed or timed out")

            self.stop_chassis()
            time.sleep(setting("navigation.turn_settle_sec"))
            end_state = self.align_turn_heading(target_heading, tolerance_deg=tolerance_deg)
            self.target_heading_deg = target_heading
            print(f"[Controller] ✅ Turn Completed: Current Yaw = {end_state.yaw:+.1f}° (Target Grid Heading = {target_heading:.0f}°)\n")
            return end_state
        finally:
            self.stop_chassis()
            self.wall_pid.reset()

    def align_turn_heading(self, target_heading: float, tolerance_deg=None):
        """Correct a completed turn using fresh yaw until the chassis settles on target."""
        deadline = time.monotonic() + setting("slam.heading_align_timeout_sec")
        tolerance = (setting("slam.heading_tolerance_deg")
                     if tolerance_deg is None else tolerance_deg)
        # Require feedback received after entering the correction phase.
        last_attitude_at = time.monotonic()
        within_tolerance = 0
        probe_yaw = None
        probe_error = None
        probe_started = None
        reversed_sign = False
        recovery_started = None

        while self._running.is_set() and (time.monotonic() < deadline or recovery_started is not None):
            state = self.sensor_hub.get_latest_state()
            now = time.monotonic()
            if not 0 < state.attitude_received_at <= now or now - state.attitude_received_at > setting("slam.max_sensor_age_sec"):
                if recovery_started is None:
                    self.stop_chassis()
                    recovery_started = now
                    within_tolerance = 0
                    probe_yaw = None
                    print("[Controller] Yaw feedback paused; waiting with chassis stopped")
                if now - recovery_started >= setting("slam.heading_feedback_recovery_timeout_sec"):
                    raise RuntimeError("Stale chassis attitude during turn alignment after recovery timeout")
                time.sleep(0.01)
                continue
            if not math.isfinite(state.yaw):
                raise RuntimeError("Invalid chassis yaw during turn alignment")
            if state.attitude_received_at <= last_attitude_at:
                time.sleep(0.01)
                continue
            if recovery_started is not None:
                deadline += now - recovery_started
                recovery_started = None
                probe_yaw = None
                print("[Controller] Fresh yaw feedback restored; resuming heading alignment")
            last_attitude_at = state.attitude_received_at

            error = (target_heading - state.yaw + 180.0) % 360.0 - 180.0
            if abs(error) <= tolerance:
                self.stop_chassis()
                probe_yaw = None
                within_tolerance += 1
                if within_tolerance >= 3:
                    return state
            else:
                within_tolerance = 0
                if probe_yaw is None:
                    probe_yaw, probe_error, probe_started = state.yaw, error, now
                elif now - probe_started >= 0.20:
                    yaw_change = (state.yaw - probe_yaw + 180.0) % 360.0 - 180.0
                    if yaw_change * probe_error < -0.5:
                        self.stop_chassis()
                        if reversed_sign:
                            raise RuntimeError("Yaw correction moved away from target in both directions")
                        self.yaw_speed_command_sign *= -1
                        reversed_sign = True
                        print("[Controller] Yaw correction reversed after feedback showed increasing error")
                        probe_yaw, probe_error, probe_started = state.yaw, error, now
                    elif abs(yaw_change) >= 0.5:
                        probe_yaw, probe_error, probe_started = state.yaw, error, now
                correction_speed = max(4.0, min(15.0, abs(error) * 1.8))
                self.drive_speed(0.0, 0.0, math.copysign(correction_speed, error))
            time.sleep(0.01)

        if not self._running.is_set():
            raise RuntimeError("Chassis turn was interrupted")
        if recovery_started is not None:
            raise RuntimeError("Stale chassis attitude during turn alignment after recovery timeout")
        raise RuntimeError("Chassis heading alignment timed out")

    def turn_left(self, deg: float = 90.0, speed: float = setting("navigation.turn_speed_dps")):
        """เลี้ยวซ้าย z = +90 องศา."""
        self.turn_to_relative(deg=+abs(deg), speed=speed)

    def turn_right(self, deg: float = 90.0, speed: float = setting("navigation.turn_speed_dps")):
        """เลี้ยวขวา z = -90 องศา."""
        self.turn_to_relative(deg=-abs(deg), speed=speed)

    def turn_around(self, speed: float = setting("navigation.turn_speed_dps")):
        """กลับหลังหัน z = 180 องศา."""
        self.turn_to_relative(deg=180.0, speed=speed)

    def emergency_stop(self):
        """Stops all robot motion immediately."""
        self.current_action = "EMERGENCY_STOP"
        try:
            self.brake_chassis()
        except Exception as exc:
            print("[Controller] Emergency brake warning: {}".format(exc))
            self.stop_chassis()

    def enable_motion(self):
        """Enable synchronous motion/scan actions without starting the queue worker."""
        self._running.set()

    def is_running(self) -> bool:
        """Whether motion/scan actions are enabled and have not been stopped."""
        return self._running.is_set()

    def start_running(self):
        self.enable_motion()
        self.start()

    def stop_running(self):
        self._running.clear()
        self._pause_event.set()
        self.emergency_stop()

    def pause(self):
        self._pause_event.clear()

    def resume(self):
        self._pause_event.set()

    def execute_command(self, cmd_text: str):
        """Parses and executes a command string from the motion command queue."""
        cmd = cmd_text.strip()
        print(f"\n==================================================")
        print(f"[Thread 2 Action] Executing: {cmd}")
        print(f"==================================================")

        if "Move Forward:" in cmd:
            parts = cmd.split("Move Forward:")
            cells = int(parts[1].replace("cells", "").replace("cell", "").strip())
            self.move_forward_grid(cells=cells)
        elif "Turn Right (90 deg)" in cmd:
            self.turn_right()
        elif "Turn Left (90 deg)" in cmd:
            self.turn_left()
        elif "Turn Around (180 deg)" in cmd:
            self.turn_around()
        else:
            print(f"  [Warning] Unknown command format: {cmd}")

        # Sleep 1.0s before proceeding to next state
        if self.step_pause_sec > 0:
            print(f"[Controller] ⏸️ Pausing {self.step_pause_sec:.1f}s before next state...")
            time.sleep(self.step_pause_sec)

    def run(self):
        try:
            self._run_commands()
        except Exception as exc:
            self.failure = str(exc)
            self.stop_running()

    def _run_commands(self):
        """Thread 2 main execution loop."""
        while self._running.is_set():
            self._pause_event.wait()

            if self.current_step < len(self.command_queue):
                cmd = self.command_queue[self.current_step]
                self.execute_command(cmd)
                self.current_step += 1
            else:
                if not self.commands_completed:
                    self.commands_completed = True
                    self.current_action = "COMPLETED"
                    print("\n[RobotControllerThread] All queued commands executed successfully with Step 3 PID!")
                time.sleep(0.1)
