"""Saved-map navigation through the shared SLAM hardware interface."""
import math
import time
from .slam_hardware import HardwareBackend
from .grid_slam import BlockedCellError, ScanHeadingDriftError, wrap
from .sdk_connection import load_robot_sdk
from .settings import get as setting


class NavigationBackend(HardwareBackend):
    def ensure_running(self):
        if not self.controller.is_running():
            raise RuntimeError('Motion was interrupted')
        waited = 0.0
        origin = getattr(self, 'scan_origin', None)
        if origin is not None:
            paused = False
            state = self.hub.get_latest_state()
            if not self.fresh(state, ['attitude_received_at', 'position_received_at']):
                started = time.monotonic()
                self.event_log.append({'timestamp': time.time(), 'type': 'scan_pose_wait'})
                if self.controller.pause_on_sensor_outage:
                    before = state
                    state, waited, paused = self.controller._wait_for_sensor_recovery(
                        state, ('attitude_received_at', 'position_received_at'),
                        'Stationary scan: stale chassis pose')
                    self.controller._validate_recovered_pose(before, state, origin[2])
                else:
                    deadline = started + setting('slam.scan_pose_recovery_timeout_sec')
                    while not self.fresh(state, ['attitude_received_at', 'position_received_at']):
                        if not self.controller.is_running():
                            raise RuntimeError('Motion was interrupted')
                        if time.monotonic() >= deadline:
                            raise RuntimeError('Stationary scan: stale chassis pose persisted for {:.2f}s'.format(
                                time.monotonic() - started))
                        # Chassis remains stopped. Do not issue speed packets or use old pose.
                        time.sleep(0.01)
                        state = self.hub.get_latest_state()
                    waited = time.monotonic() - started
                self.event_log.append({'timestamp': time.time(), 'type': 'scan_pose_recovered',
                                       'waited_sec': waited})
            if not all(math.isfinite(v) for v in (state.yaw, state.pos_x, state.pos_y)):
                raise RuntimeError('Stationary scan: invalid chassis pose')
            yaw_drift = abs(wrap(state.yaw - origin[2]))
            position_drift = math.hypot(state.pos_x - origin[0], state.pos_y - origin[1])
            if paused and position_drift > setting('slam.localization_gate_m'):
                raise RuntimeError('Stationary scan: position changed during sensor recovery ({:.3f} m)'.format(
                    position_drift))
            if (position_drift > setting('scan_guard.max_position_drift_m')
                    and not getattr(self, 'scan_position_warning_logged', False)):
                # Odometry can settle after wheel stop; warn once without aborting.
                self.event_log.append({'timestamp': time.time(), 'type': 'scan_position_drift',
                                       'position_drift_m': position_drift,
                                       'origin_m': list(origin[:2]),
                                       'position_m': [state.pos_x, state.pos_y]})
                self.scan_position_warning_logged = True
            if yaw_drift > setting('scan_guard.max_heading_drift_deg'):
                raise ScanHeadingDriftError('Stationary scan: chassis moved (yaw drift={:.2f} deg, position drift={:.3f} m)'.format(
                    yaw_drift, position_drift))
            self.controller._recovery_completed(paused)
        return waited

    def move(self, direction, align_if_straight=True, cells=1, on_cell=None):
        if cells < 1:
            raise ValueError('cells must be positive')
        self.hold_front_for_motion()
        self.select_robot_mode(load_robot_sdk().CHASSIS_LEAD, 'before_motion')
        turn = (direction - self.heading) % 4
        if turn:
            self.event_log.append({'timestamp': time.time(), 'type': 'chassis_turn',
                                   'degrees': {1: -90, 2: 180, 3: 90}[turn]})
            self.controller.turn_to_relative({1: -90, 2: 180, 3: 90}[turn])
        self.heading = direction
        if turn or align_if_straight:
            self.align_heading()
        else:
            state = self.hub.get_latest_state()
            if not self.fresh(state, ['attitude_received_at']):
                raise RuntimeError('Stale chassis attitude before straight motion')
            target = wrap(direction * 90)
            error = wrap(state.yaw - target)
            if abs(error) > setting('navigation.max_heading_error_deg'):
                raise RuntimeError('Cannot drive straight: heading differs by {:.1f} deg'.format(error))
            self.controller.target_heading_deg = target
        self.ensure_gimbal_front('after_chassis_turn' if turn else 'before_motion', restore_follow_mode=True)
        distance = self.sample(0, self.aim(0))
        stop_distance_mm = (self.controller.wall_pid.front_target_mm
                            + setting('navigation.front_stop_tolerance_mm'))
        if distance * 1000 <= stop_distance_mm:
            raise BlockedCellError(distance * 1000, stop_distance_mm)
        self.controller.front_ready = True
        before = self.hub.get_latest_state()
        if not self.fresh(before, ['position_received_at', 'attitude_received_at']):
            raise RuntimeError('Stale pose before moving')
        self.event_log.append({'timestamp': time.time(), 'type': 'walk_start',
                               'direction': direction, 'cell_size_m': setting('navigation.grid_size_m'),
                               'cells': cells})
        if cells == 1:
            result = self.controller.navigate_single_grid_step()
        else:
            result = self.controller.navigate_straight_cells(cells, on_cell=on_cell)
        self.event_log.append(dict(result, timestamp=time.time(), type='walk_end'))
        if not result['completed']:
            raise RuntimeError('Cell motion failed: {}'.format(result['reason']))
        after = self.prepare_stationary_scan()
        if not self.fresh(after, ['tof_received_at']):
            raise RuntimeError('Stale front ToF after stopping at cell')
        front_mm = self.system.calibration_mgr.raw_to_mm('tof', after.tof_raw)
        if front_mm is None or not math.isfinite(front_mm) or front_mm <= 0:
            raise RuntimeError('Invalid front ToF after stopping at cell')
        if after.tof_filtered_mm is not None and math.isfinite(after.tof_filtered_mm):
            front_mm = min(front_mm, after.tof_filtered_mm)
        if front_mm <= setting('navigation.emergency_front_mm'):
            self.event_log.append({'timestamp': time.time(), 'type': 'front_emergency_after_stop',
                                   'distance_mm': front_mm, 'direction': direction})
            raise RuntimeError('Front clearance below emergency limit after stopping: {:.0f} mm'.format(front_mm))
        if on_cell is not None:
            on_cell(cells)
        return (after.pos_x - before.pos_x, after.pos_y - before.pos_y), after.yaw
