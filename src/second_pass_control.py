"""Gentle wall centering during navigation on a saved map."""

from .pid_controller import WallCenteringPID
from .settings import get as setting


class SmoothWallCenteringPID(WallCenteringPID):
    def __init__(self):
        config = setting("second_pass")
        self.lateral_slew_mps2 = config["lateral_slew_mps2"]
        self.last_lateral_speed = 0.0
        super().__init__(
            lateral_kp=config["lateral_kp"],
            lateral_ki=config["lateral_ki"],
            lateral_kd=config["lateral_kd"],
            max_lateral_speed=config["max_lateral_speed_mps"],
        )

    def reset(self):
        super().reset()
        self.last_lateral_speed = 0.0

    def compute_control_speeds(self, state, target_yaw_deg, base_vx=0.0, dt=None):
        vx, vy, vz, case_name, case_id, error_y = super().compute_control_speeds(
            state, target_yaw_deg, base_vx=base_vx, dt=dt)
        interval = dt if dt is not None and dt > 0 else 1.0 / setting("navigation.control_rate_hz")
        max_change = self.lateral_slew_mps2 * interval
        vy = max(self.last_lateral_speed - max_change,
                 min(self.last_lateral_speed + max_change, vy))
        self.last_lateral_speed = vy
        return vx, vy, vz, case_name, case_id, error_y
