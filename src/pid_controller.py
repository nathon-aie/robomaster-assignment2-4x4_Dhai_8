#!/usr/bin/env python3
"""PID Controllers and Wall-Centering Logic for Step 3 Grid Navigation.

Step 3 Requirements (WORK.md):
- เดินทีละ Grid (Grid size 60x60 cm, Wall thickness 7.5 cm)
- PID Control ปรับการเคลื่อนที่แกน Y ให้อยู่ตรงกลางระหว่างกำแพง (ดึงค่าจาก Thread 1)
- 8 Cases:
  1. มีกำแพงข้างหน้า (วัดระยะจาก ToF ว่าอยู่ตรงกลาง Grid)
     1.1 มีกำแพง 2 ข้าง: Sharp |L - R| < 2 cm (20 mm)
     1.2 มีกำแพงแค่ข้างซ้าย: Sharp L +- 2 cm จากค่าปกติ
     1.3 มีกำแพงแค่ข้างขวา: Sharp R +- 2 cm จากค่าปกติ
     1.4 ไม่มีกำแพง
  2. ไม่มีกำแพงข้างหน้า (เดินไปข้างหน้า 1 Grid 60 cm)
     2.1 มีกำแพง 2 ข้าง: Sharp |L - R| < 2 cm (20 mm)
     2.2 มีกำแพงแค่ข้างซ้าย: Sharp L +- 2 cm จากค่าปกติ
     2.3 มีกำแพงแค่ข้างขวา: Sharp R +- 2 cm จากค่าปกติ
     2.4 ไม่มีกำแพง
"""

try:
    from .settings import get as setting, project_path
except ImportError:
    from settings import get as setting, project_path

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

try:
    from .sensor_pipeline import RobotSensorSnapshot
except (ImportError, ValueError):
    from sensor_pipeline import RobotSensorSnapshot


@dataclass
class PIDGains:
    """PID Gain parameters."""
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    max_output: float = 0.5
    min_output: float = -0.5
    integral_limit: float = 0.2
    deadband: float = 0.0  # Output 0 if |error| < deadband


class PIDController:
    """General purpose PID Controller with Anti-Windup and Deadband."""

    def __init__(self, gains: PIDGains):
        self.gains = gains
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.last_error: Optional[float] = None
        self.last_time: Optional[float] = None

    def compute(self, error: float, dt: Optional[float] = None) -> float:
        # Apply deadband
        if abs(error) < self.gains.deadband:
            return 0.0

        current_time = time.monotonic()
        if dt is None:
            if self.last_time is not None:
                dt = current_time - self.last_time
            else:
                dt = 0.05  # Default nominal dt (20Hz)
        self.last_time = current_time

        # P term
        p_term = self.gains.kp * error

        # I term with Anti-Windup clamping
        if dt > 0:
            self.integral += error * dt
            # Clamp integral
            self.integral = max(
                -self.gains.integral_limit,
                min(self.gains.integral_limit, self.integral),
            )
        i_term = self.gains.ki * self.integral

        # D term
        d_term = 0.0
        if self.last_error is not None and dt > 0:
            derivative = (error - self.last_error) / dt
            d_term = self.gains.kd * derivative
        self.last_error = error

        output = p_term + i_term + d_term
        # Clamp output
        return max(self.gains.min_output, min(self.gains.max_output, output))


class WallCenteringPID:
    """Implements Step 3 Wall Centering across the 8 REQ cases."""

    # Default parameters based on 60x60cm Grid & 25cm Robot
    # Corridor inner width ~ 525mm, Robot width 250mm -> ~137.5mm each side
    DEFAULT_NOMINAL_SIDE_MM = setting("navigation.nominal_side_mm")
    DEADBAND_TOLERANCE_MM = setting("navigation.tolerance_mm")  # 2 cm tolerance as specified in REQ (|L-R| < 2cm, L/R +- 2cm)
    WALL_DETECT_THRESHOLD_MM = setting("navigation.side_wall_threshold_mm")  # Max distance to consider side wall present
    FRONT_WALL_STOP_MM = setting("navigation.front_target_mm")  # Distance from front ToF to front wall at grid center

    def __init__(
        self,
        nominal_side_dist_mm: float = DEFAULT_NOMINAL_SIDE_MM,
        tolerance_mm: float = DEADBAND_TOLERANCE_MM,
        front_target_mm: float = FRONT_WALL_STOP_MM,
        lateral_kp: float = setting("pid.lateral.kp"),  # Smooth lateral centering (50mm error -> ~0.09 m/s)
        lateral_ki: float = setting("pid.lateral.ki"),
        lateral_kd: float = setting("pid.lateral.kd"),  # Damping to prevent oscillating across corridor
        max_lateral_speed: float = setting("pid.lateral.max_speed"),  # Max vy m/s (gentle correction)
        yaw_kp: float = setting("pid.yaw.kp"),  # Active Heading Hold: 1 deg error -> 1.8 deg/s vz
        yaw_ki: float = setting("pid.yaw.ki"),  # Eliminates steady-state heading drift
        yaw_kd: float = setting("pid.yaw.kd"),  # Strong derivative damping against heading wobble
        max_yaw_speed: float = setting("pid.yaw.max_speed"),  # Max vz deg/s
    ):
        self.nominal_side_dist_mm = nominal_side_dist_mm
        self.tolerance_mm = tolerance_mm
        self.front_target_mm = front_target_mm

        # Lateral (Y-axis) PID: error in mm -> vy in m/s
        self.pid_lateral = PIDController(
            PIDGains(
                kp=lateral_kp,
                ki=lateral_ki,
                kd=lateral_kd,
                max_output=max_lateral_speed,
                min_output=-max_lateral_speed,
                integral_limit=setting("pid.lateral.integral_limit"),
                deadband=tolerance_mm,  # |error| < 20mm (2cm) -> vy = 0
            )
        )

        # Yaw Heading PID: error in deg -> vz in deg/s (Locks heading rock-solid with 0 deadband)
        self.pid_yaw = PIDController(
            PIDGains(
                kp=yaw_kp,
                ki=yaw_ki,
                kd=yaw_kd,
                max_output=max_yaw_speed,
                min_output=-max_yaw_speed,
                integral_limit=setting("pid.yaw.integral_limit"),
                deadband=0.0,  # 0.0 deadband: instantly corrects even a fraction of a degree
            )
        )

    def reset(self):
        self.pid_lateral.reset()
        self.pid_yaw.reset()

    def classify_wall_state(self, state: RobotSensorSnapshot) -> Tuple[bool, bool, bool]:
        """Classifies (has_front_wall, has_left_wall, has_right_wall)."""
        has_left = (
            state.sharp_left_valid
            and state.sharp_left_mm is not None
            and state.sharp_left_mm < self.WALL_DETECT_THRESHOLD_MM
        )
        has_right = (
            state.sharp_right_valid
            and state.sharp_right_mm is not None
            and state.sharp_right_mm < self.WALL_DETECT_THRESHOLD_MM
        )
        has_front = (
            state.tof_valid
            and state.tof_filtered_mm is not None
            and state.tof_filtered_mm < setting("navigation.front_wall_threshold_mm")
        )
        return has_front, has_left, has_right

    def compute_lateral_error(self, state: RobotSensorSnapshot) -> Tuple[float, str, int]:
        """Calculates lateral error (mm), description, and case id (1.1 - 2.4)."""
        has_front, has_left, has_right = self.classify_wall_state(state)
        l_mm = state.sharp_left_mm
        r_mm = state.sharp_right_mm

        if has_front:
            # Case 1: Front Wall present
            if has_left and has_right and l_mm is not None and r_mm is not None:
                # Case 1.1: Walls on both sides -> error = R - L
                # If L < R (closer to left), error > 0 -> strafe right (vy > 0)
                # If L > R (closer to right), error < 0 -> strafe left (vy < 0)
                error_y = r_mm - l_mm
                case_name = "Case 1.1: Front Wall + Both Side Walls (|L-R| < 2cm)"
                case_id = 11
            elif has_left and l_mm is not None:
                # Case 1.2: Left wall only -> error = Nominal - L
                # If L < Nominal (too close to left), error > 0 -> strafe right (vy > 0)
                error_y = self.nominal_side_dist_mm - l_mm
                case_name = "Case 1.2: Front Wall + Left Wall Only (L +- 2cm)"
                case_id = 12
            elif has_right and r_mm is not None:
                # Case 1.3: Right wall only -> error = R - Nominal
                # If R < Nominal (too close to right), error < 0 -> strafe left (vy < 0)
                error_y = r_mm - self.nominal_side_dist_mm
                case_name = "Case 1.3: Front Wall + Right Wall Only (R +- 2cm)"
                case_id = 13
            else:
                # Case 1.4: Front wall only, no side walls
                error_y = 0.0
                case_name = "Case 1.4: Front Wall + No Side Walls"
                case_id = 14
        else:
            # Case 2: No Front Wall
            if has_left and has_right and l_mm is not None and r_mm is not None:
                # Case 2.1: Walls on both sides -> error = R - L
                error_y = r_mm - l_mm
                case_name = "Case 2.1: No Front Wall + Both Side Walls (|L-R| < 2cm)"
                case_id = 21
            elif has_left and l_mm is not None:
                # Case 2.2: Left wall only -> error = Nominal - L
                error_y = self.nominal_side_dist_mm - l_mm
                case_name = "Case 2.2: No Front Wall + Left Wall Only (L +- 2cm)"
                case_id = 22
            elif has_right and r_mm is not None:
                # Case 2.3: Right wall only -> error = R - Nominal
                error_y = r_mm - self.nominal_side_dist_mm
                case_name = "Case 2.3: No Front Wall + Right Wall Only (R +- 2cm)"
                case_id = 23
            else:
                # Case 2.4: Open space (no walls)
                error_y = 0.0
                case_name = "Case 2.4: Open Space (No Side Walls)"
                case_id = 24

        return error_y, case_name, case_id

    def compute_control_speeds(
        self,
        state: RobotSensorSnapshot,
        target_yaw_deg: float,
        base_vx: float = 0.0,
        dt: Optional[float] = None,
    ) -> Tuple[float, float, float, str, int, float]:
        """Calculates (vx, vy, vz, case_name, case_id, error_y).

        - vx: Forward speed (m/s)
        - vy: Lateral correction speed (m/s) from PID
        - vz: Angular yaw correction speed (deg/s) from Heading PID
        """
        error_y, case_name, case_id = self.compute_lateral_error(state)

        # Compute lateral correction speed vy
        vy = self.pid_lateral.compute(error_y, dt=dt)

        # Compute heading error (wrap to [-180, 180])
        raw_yaw_diff = target_yaw_deg - state.yaw
        yaw_error = (raw_yaw_diff + 180.0) % 360.0 - 180.0
        vz = self.pid_yaw.compute(yaw_error, dt=dt)

        # Longitudinal speed vx: if front wall detected and close, decelerate
        vx = base_vx
        has_front, _, _ = self.classify_wall_state(state)
        if has_front and state.tof_filtered_mm is not None:
            dist_to_stop = state.tof_filtered_mm - self.front_target_mm
            if dist_to_stop <= setting("navigation.front_stop_tolerance_mm"):
                vx = 0.0  # Reached front wall stop point
            elif dist_to_stop < setting("navigation.front_deceleration_mm"):
                # Proportional deceleration near front wall
                vx = min(vx, max(setting("navigation.minimum_front_speed_mps"), base_vx * (dist_to_stop / setting("navigation.front_deceleration_mm"))))

        return vx, vy, vz, case_name, case_id, error_y
