"""Saved-map motion with sensor recovery and continuous straight runs."""
import math
import time
from dataclasses import replace

from .robot_controller import RobotControllerThread
from .settings import get as setting


class NavigationController(RobotControllerThread):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.pause_on_sensor_outage = False
        self.recovery_callback = None

    def _wait_for_sensor_recovery(self, state, fields, failure, newer_attitude_than=None):
        """Wait stationary for fresh packets; Navigation may remain paused until Stop."""
        self.stop_chassis()
        started = time.monotonic()
        grace = setting("navigation.sensor_recovery_timeout_sec")
        paused = False
        while self._running.is_set():
            now = time.monotonic()
            fresh = all(0 < getattr(state, name) <= now
                        and now - getattr(state, name) <= setting("slam.max_sensor_age_sec")
                        for name in fields)
            if newer_attitude_than is not None:
                fresh = fresh and state.attitude_received_at > newer_attitude_than
            if fresh:
                waited = now - started
                self.recovery_wait_total_sec += waited
                return state, waited, paused
            if now - started >= grace:
                if not self.pause_on_sensor_outage:
                    raise RuntimeError(failure)
                if not paused:
                    paused = True
                    if self.recovery_callback is not None:
                        self.recovery_callback("paused", failure)
                    print("[Controller] Navigation paused; waiting for fresh sensor packets. Press Stop to cancel.")
            time.sleep(0.02)
            state = self.sensor_hub.get_latest_state()
        raise KeyboardInterrupt

    def _validate_recovered_pose(self, before, after, target_heading):
        values = (before.pos_x, before.pos_y, after.pos_x, after.pos_y, after.yaw)
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError("Invalid chassis pose after sensor recovery")
        drift = math.hypot(after.pos_x - before.pos_x, after.pos_y - before.pos_y)
        if drift > setting("slam.localization_gate_m"):
            raise RuntimeError("Chassis moved {:.3f} m during sensor recovery; position uncertain".format(drift))
        heading_error = (after.yaw - target_heading + 180.0) % 360.0 - 180.0
        if abs(heading_error) > setting("navigation.max_heading_error_deg"):
            raise RuntimeError("Chassis heading changed during sensor recovery: {:.1f} deg".format(heading_error))

    def _recovery_completed(self, paused):
        if paused and self.recovery_callback is not None:
            self.recovery_callback("resumed", "Fresh pose and heading verified")

    def motion_state(self):
        state = self.sensor_hub.get_latest_state()
        if not self.strict_sensors:
            return state
        now = time.monotonic()
        age = setting("slam.max_sensor_age_sec")
        sensor_times = ("tof_received_at", "position_received_at", "attitude_received_at",
                        "gimbal_received_at", "sharp_left_received_at",
                        "sharp_right_received_at")

        def stale_fields(snapshot, checked_at):
            return [name for name in sensor_times
                    if not 0 < getattr(snapshot, name) <= checked_at
                    or checked_at - getattr(snapshot, name) > age]

        stale = stale_fields(state, now)
        paused = False
        if stale:
            before = state
            state, _, paused = self._wait_for_sensor_recovery(
                state, sensor_times, "Stale sensor stream: {}".format(stale[0]))
            self._validate_recovered_pose(before, state, self.target_heading_deg)
            self.wall_pid.reset()
            print("[Controller] Sensor stream recovered; resuming from fresh pose and ToF.")
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
        self._recovery_completed(paused)
        return replace(state, tof_valid=True, tof_filtered_mm=min(distance, filtered))

    def navigate_straight_cells(self, cells: int, on_cell=None):
        """Drive a straight map run without stopping at intermediate cell centers."""
        if cells < 1:
            raise ValueError("cells must be positive")
        if cells == 1:
            result = self.navigate_single_grid_step()
            return result

        self.current_action = "NAVIGATE_STRAIGHT_{}_CELLS".format(cells)
        total_distance = cells * self.grid_size_m
        print("\n  [Straight Run] Moving {} cells ({:.2f} m) without intermediate stops..."
              .format(cells, total_distance))
        initial = self.motion_state()
        start_x, start_y = initial.pos_x, initial.pos_y
        self.wall_pid.reset()
        control_hz = setting("navigation.control_rate_hz")
        dt = 1.0 / control_hz
        max_duration = (total_distance / max(0.1, self.base_speed)
                        * setting("navigation.timeout_multiplier")
                        + setting("navigation.timeout_extra_sec"))
        started = time.monotonic()
        initial_recovery_wait = self.recovery_wait_total_sec
        reached_cells = 0
        distance = 0.0
        reason = "interrupted"
        last_case_id = None

        try:
            while distance < total_distance and self._running.is_set():
                loop_started = time.monotonic()
                if (loop_started - started
                        - (self.recovery_wait_total_sec - initial_recovery_wait) > max_duration):
                    reason = "timeout"
                    break
                state = self.motion_state()
                dx, dy = state.pos_x - start_x, state.pos_y - start_y
                radians = math.radians(self.target_heading_deg)
                distance = max(0.0, dx * math.cos(radians) + dy * math.sin(radians))
                lateral = -dx * math.sin(radians) + dy * math.cos(radians)
                heading_error = (state.yaw - self.target_heading_deg + 180.0) % 360.0 - 180.0
                if not all(math.isfinite(value) for value in (distance, lateral, heading_error)):
                    reason = "invalid_pose"
                    break
                if abs(heading_error) > setting("navigation.max_heading_error_deg"):
                    reason = "heading_deviation"
                    break
                if abs(lateral) > setting("navigation.max_lateral_deviation_m"):
                    reason = "lateral_deviation"
                    break
                if state.tof_valid and state.tof_filtered_mm is not None:
                    if state.tof_filtered_mm <= setting("navigation.emergency_front_mm"):
                        self.brake_chassis()
                        reason = "emergency_obstacle"
                        break
                    stop_distance = (self.wall_pid.front_target_mm
                                     + setting("navigation.front_stop_tolerance_mm"))
                    if state.tof_filtered_mm <= stop_distance:
                        reason = "front_wall"
                        break
                if distance >= total_distance:
                    reason = "distance_reached"
                    break

                while (reached_cells < cells - 1
                       and distance >= (reached_cells + 1) * self.grid_size_m):
                    reached_cells += 1
                    if on_cell is not None:
                        on_cell(reached_cells)

                remaining = total_distance - distance
                forward_speed = self.base_speed
                if remaining < setting("navigation.end_deceleration_m"):
                    forward_speed = max(
                        setting("navigation.minimum_forward_speed_mps"),
                        self.base_speed * remaining / setting("navigation.end_deceleration_m"))
                vx, vy, vz, case_name, case_id, error_y = self.wall_pid.compute_control_speeds(
                    state=state, target_yaw_deg=self.target_heading_deg,
                    base_vx=forward_speed, dt=dt)
                if case_id != last_case_id:
                    print("  [PID Centering] {} | Lat Err: {:+.1f} mm | vy: {:+.2f} m/s"
                          .format(case_name, error_y, vy))
                    last_case_id = case_id
                self.drive_speed(vx=vx, vy=vy, vz=vz)
                remaining_loop = dt - (time.monotonic() - loop_started)
                if remaining_loop > 0:
                    time.sleep(remaining_loop)

            self.stop_chassis()
            end_state = self.motion_state()
            dx, dy = end_state.pos_x - start_x, end_state.pos_y - start_y
            radians = math.radians(self.target_heading_deg)
            forward = dx * math.cos(radians) + dy * math.sin(radians)
            lateral = -dx * math.sin(radians) + dy * math.cos(radians)
            heading_error = (end_state.yaw - self.target_heading_deg + 180.0) % 360.0 - 180.0
            if reason == "front_wall":
                minimum = (cells - 1 + setting("navigation.front_wall_arrival_min_fraction"))
                arrived = (minimum * self.grid_size_m <= forward
                           <= total_distance + setting("slam.cell_arrival_tolerance_m"))
            else:
                arrived = (reason == "distance_reached" and
                           abs(forward - total_distance) <= setting("slam.cell_arrival_tolerance_m"))
            completed = self._running.is_set() and arrived
            if completed:
                self.align_at_cell_center(duration_sec=setting("navigation.align_duration_sec"))
                end_state = self.motion_state()
                dx, dy = end_state.pos_x - start_x, end_state.pos_y - start_y
                forward = dx * math.cos(radians) + dy * math.sin(radians)
                lateral = -dx * math.sin(radians) + dy * math.cos(radians)
                heading_error = (end_state.yaw - self.target_heading_deg + 180.0) % 360.0 - 180.0
                completed = (self._running.is_set() and
                             (minimum * self.grid_size_m <= forward
                              <= total_distance + setting("slam.cell_arrival_tolerance_m") if reason == "front_wall"
                              else abs(forward - total_distance)
                              <= setting("slam.cell_arrival_tolerance_m")))
            if not completed and reason == "distance_reached":
                reason = "arrival_pose_outside_tolerance"
            return {"completed": completed, "reason": reason, "distance_m": forward,
                    "lateral_deviation_m": lateral, "heading_error_deg": heading_error}
        finally:
            self.stop_chassis()

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

        while time.monotonic() < deadline and self._running.is_set():
            state = self.sensor_hub.get_latest_state()
            now = time.monotonic()
            if not 0 < state.attitude_received_at <= now or now - state.attitude_received_at > setting("slam.max_sensor_age_sec"):
                before = state
                state, waited, paused = self._wait_for_sensor_recovery(
                    state, ("attitude_received_at", "position_received_at"),
                    "Stale chassis attitude during turn alignment",
                    newer_attitude_than=last_attitude_at)
                self._validate_recovered_pose(before, state, target_heading)
                self._recovery_completed(paused)
                now = time.monotonic()
                deadline += waited
                within_tolerance = 0
                probe_yaw = None
                print("[Controller] Attitude stream recovered; resuming turn alignment.")
            if not math.isfinite(state.yaw):
                raise RuntimeError("Invalid chassis yaw during turn alignment")
            if state.attitude_received_at <= last_attitude_at:
                time.sleep(0.01)
                continue
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
        raise RuntimeError("Chassis heading alignment timed out")

