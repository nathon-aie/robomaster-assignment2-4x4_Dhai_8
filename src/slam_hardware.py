"""Stationary Gimbal scans and guarded one-cell motion using the shared sensor hub."""
import math
import statistics
import time
from .grid_slam import wrap
from .sdk_connection import cancel_chassis_speed_timer, stop_chassis_wheels
from .settings import get as setting


class HardwareBackend:
    def __init__(self, system):
        self.system = system
        self.robot = system.robot
        self.controller = system.thread_2_controller
        self.collector = system.thread_1_sensor
        self.hub = system.sensor_hub
        self.heading = 0
        self.event_log = []
        self.scan_headings = None
        self.scan_origin = None
        self.scan_position_warning_logged = False
        self.initial_scan_completed = False
        self.gimbal_reference_ready = False
        self.commanded_gimbal_yaw = None
        self.controller.strict_sensors = True
        self.controller.calibration_manager = system.calibration_mgr
        self.controller.front_ready = False

    def ensure_running(self):
        if not self.controller.is_running():
            raise RuntimeError('Motion was interrupted')
        waited = 0.0
        origin = getattr(self, 'scan_origin', None)
        if origin is not None:
            state = self.hub.get_latest_state()
            if not self.fresh(state, ['attitude_received_at', 'position_received_at']):
                started = time.monotonic()
                deadline = started + setting('slam.scan_pose_recovery_timeout_sec')
                self.event_log.append({'timestamp': time.time(), 'type': 'scan_pose_wait'})
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
            if (position_drift > setting('scan_guard.max_position_drift_m')
                    and not getattr(self, 'scan_position_warning_logged', False)):
                # Odometry can settle after wheel stop; warn once without aborting.
                self.event_log.append({'timestamp': time.time(), 'type': 'scan_position_drift',
                                       'position_drift_m': position_drift,
                                       'origin_m': list(origin[:2]),
                                       'position_m': [state.pos_x, state.pos_y]})
                self.scan_position_warning_logged = True
            if yaw_drift > setting('scan_guard.max_heading_drift_deg'):
                raise RuntimeError('Stationary scan: chassis moved (yaw drift={:.2f} deg, position drift={:.3f} m)'.format(
                    yaw_drift, position_drift))
        return waited

    def fresh(self, state, fields):
        now = time.monotonic()
        age = setting('slam.max_sensor_age_sec')
        return all(0 < getattr(state, name) <= now and now - getattr(state, name) <= age for name in fields)

    def action_completed(self, action, name='Gimbal/chassis action'):
        started = time.monotonic()
        completed = action.wait_for_completed(timeout=setting('gimbal.action_timeout_sec'))
        succeeded = bool(action.has_succeeded)
        elapsed = time.monotonic() - started
        state = str(getattr(action, 'state', 'unknown'))
        self.event_log.append({'timestamp': time.time(), 'type': 'hardware_action',
                               'name': name, 'state': state, 'completed': bool(completed),
                               'succeeded': succeeded, 'elapsed_sec': elapsed})
        if not completed or not succeeded:
            raise RuntimeError('{} failed or timed out (state={}, elapsed={:.2f}s)'.format(name, state, elapsed))
        self.ensure_running()

    def gimbal_state(self, after=0):
        deadline = time.monotonic() + setting('slam.sensor_timeout_sec')
        while time.monotonic() < deadline:
            deadline += self.ensure_running() or 0.0
            state = self.hub.get_latest_state()
            if (self.fresh(state, ['gimbal_received_at'])
                    and state.gimbal_received_at > after
                    and math.isfinite(state.gimbal_yaw) and math.isfinite(state.gimbal_pitch)):
                return state
            time.sleep(0.01)
        raise RuntimeError('No fresh valid Gimbal feedback')

    def log_gimbal(self, phase, yaw, state, **details):
        event = {'timestamp': time.time(), 'type': 'gimbal', 'phase': phase,
                 'target_yaw': yaw, 'target_pitch': setting('gimbal.pitch_deg'),
                 'actual_yaw': state.gimbal_yaw, 'actual_pitch': state.gimbal_pitch,
                 'ground_yaw': getattr(state, 'gimbal_yaw_ground', None),
                 'ground_pitch': getattr(state, 'gimbal_pitch_ground', None)}
        event.update(details)
        self.event_log.append(event)

    def stop_scan_chassis(self):
        # Do not enter SDK speed-control mode between Gimbal position actions.
        if getattr(self, 'scan_origin', None) is None:
            self.controller.stop_chassis()

    def recenter_gimbal(self, name='Gimbal recenter'):
        self.controller.front_ready = False
        self.action_completed(self.robot.gimbal.recenter(
            yaw_speed=setting('gimbal.recenter_yaw_speed_dps'),
            pitch_speed=setting('gimbal.recenter_pitch_speed_dps')), name=name)
        self.commanded_gimbal_yaw = 0.0
        self.gimbal_reference_ready = True
        self.log_gimbal('recenter_completed', 0, self.hub.get_latest_state())

    def initialize_gimbal_reference(self):
        if self.gimbal_reference_ready:
            return
        self.recenter_gimbal(name='Initial Gimbal recenter')
        self.log_gimbal('reference_initialized', 0, self.hub.get_latest_state())

    def hold_front_for_motion(self):
        """Set a chassis-relative position once, before chassis starts moving."""
        self.ensure_running()
        self.controller.front_ready = False
        self.initialize_gimbal_reference()
        if setting('gimbal.hold_front_on_move'):
            self.action_completed(self.robot.gimbal.moveto(
                yaw=0, pitch=setting('gimbal.pitch_deg'),
                yaw_speed=setting('gimbal.yaw_speed_dps'),
                pitch_speed=setting('gimbal.pitch_speed_dps')), name='Gimbal front position hold')
            self.commanded_gimbal_yaw = 0.0
            self.log_gimbal('front_position_hold', 0, self.hub.get_latest_state())

    def aim(self, yaw, direct=False):
        """Execute planned relative moves only; never trim from angle feedback."""
        self.ensure_running()
        self.stop_scan_chassis()
        self.controller.front_ready = False
        if not getattr(self, 'gimbal_reference_ready', False):
            self.initialize_gimbal_reference()
        try:
            # Split the planned travel only, e.g. rear -> centre = two 90s.
            # Subsequent commands use the commanded reference, not measured error.
            remaining = yaw - self.commanded_gimbal_yaw
            while abs(remaining) > 0.01:
                step = remaining if direct else max(-setting('gimbal.move_step_deg'), min(setting('gimbal.move_step_deg'), remaining))
                state = self.gimbal_state()
                self.log_gimbal('move', yaw, state, delta_yaw=step, delta_pitch=0)
                self.action_completed(self.robot.gimbal.move(
                    yaw=step, pitch=0, yaw_speed=setting('gimbal.yaw_speed_dps'),
                    pitch_speed=setting('gimbal.pitch_speed_dps')), name='Gimbal move action')
                self.commanded_gimbal_yaw += step
                remaining = yaw - self.commanded_gimbal_yaw
            # Read angle telemetry for diagnostics only.
            deadline = time.monotonic() + setting('gimbal.settle_sec')
            while time.monotonic() < deadline:
                self.ensure_running()
                time.sleep(min(0.01, max(0, deadline - time.monotonic())))
            state = self.gimbal_state()
            self.log_gimbal('move_completed', yaw, state)
            self.collector.reset_tof_filter()
            return time.monotonic()
        except BaseException as exc:
            self.log_gimbal('failed', yaw, self.hub.get_latest_state(), error=str(exc))
            self.robot.gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
            raise

    def sample(self, yaw, after):
        values = []
        rejected_ranges = []
        capped_samples = 0
        fresh_packets = 0
        last = after
        deadline = time.monotonic() + setting('slam.sensor_timeout_sec')
        while time.monotonic() < deadline:
            deadline += self.ensure_running() or 0.0
            state = self.hub.get_latest_state()
            if (state.tof_received_at > last and self.fresh(state, ['tof_received_at'])
                    and (getattr(self, 'scan_origin', None) is None
                         or (self.fresh(state, ['attitude_received_at', 'position_received_at'])))):
                last = state.tof_received_at
                fresh_packets += 1
                raw = state.tof_raw
                bounds = setting('sensors.tof_filter')
                if raw is not None and math.isfinite(raw) and raw >= bounds['min_valid']:
                    # A range beyond the configured limit is still evidence of
                    # open space; cap it instead of aborting a distant scan.
                    capped_samples += raw > bounds['max_valid']
                    distance = self.system.calibration_mgr.raw_to_mm(
                        'tof', min(raw, bounds['max_valid']))
                    if distance is not None and math.isfinite(distance) and distance > 0:
                        values.append(distance / 1000)
                else:
                    rejected_ranges.append(raw)
                if len(values) >= setting('slam.scan_samples'):
                    result = statistics.median(values)
                    self.event_log.append({'timestamp': time.time(), 'type': 'tof_sample',
                                           'gimbal_yaw': yaw, 'range_m': result, 'accepted': len(values),
                                           'capped_far_samples': capped_samples})
                    return result
            time.sleep(0.01)
        bounds = setting('sensors.tof_filter')
        details = {'timestamp': time.time(), 'type': 'tof_sample_failed', 'gimbal_yaw': yaw,
                   'fresh_packets': fresh_packets, 'accepted': len(values),
                   'valid_raw_bounds': [bounds['min_valid'], bounds['max_valid']],
                   'rejected_raw_examples': rejected_ranges[-5:]}
        self.event_log.append(details)
        raise RuntimeError('No fresh valid ToF samples after Gimbal motion: yaw={}, '
                           'fresh={}, accepted={}, valid_raw={}..{}, rejected={}'.format(
                               yaw, fresh_packets, len(values), bounds['min_valid'],
                               bounds['max_valid'], rejected_ranges[-5:]))

    def align_heading(self):
        # Chassis remains at the cell centre; lateral PID is disabled during turns.
        deadline = time.monotonic() + setting('slam.heading_align_timeout_sec')
        target = wrap(self.heading * 90)
        try:
            while time.monotonic() < deadline:
                self.ensure_running()
                state = self.hub.get_latest_state()
                if not self.fresh(state, ['attitude_received_at']):
                    raise RuntimeError('Stale chassis attitude')
                if not math.isfinite(state.yaw):
                    raise RuntimeError('Invalid chassis yaw')
                error = wrap(target - state.yaw)
                if abs(error) <= setting('slam.heading_tolerance_deg'):
                    self.controller.target_heading_deg = target
                    return
                speed = max(-20.0, min(20.0, error * 1.8))
                self.controller.drive_speed(0, 0, speed)
                time.sleep(0.05)
            raise RuntimeError('Chassis heading alignment timed out')
        finally:
            self.controller.stop_chassis()

    def face(self, direction):
        turn = (direction - self.heading) % 4
        if turn:
            self.controller.turn_to_relative({1: -90, 2: 180, 3: 90}[turn])
        self.heading = direction
        self.align_heading()

    def scan(self, mode=None):
        mode = setting('gimbal.scan_mode') if mode is None else mode
        if mode == 'chassis':
            self.controller.stop_chassis()
            self.align_heading()
        else:
            cancel_chassis_speed_timer(self.robot.chassis)
            if not getattr(self, 'gimbal_reference_ready', False):
                state = self.wait_stationary_pose(settle_sec=setting('slam.startup_settle_sec'))
            else:
                state = self.hub.get_latest_state()
            self.scan_origin = (state.pos_x, state.pos_y, state.yaw)
            self.scan_position_warning_logged = False
            self.ensure_running()
        start_heading = self.heading
        scan_rear = not getattr(self, 'initial_scan_completed', False)
        ranges = {}
        self.scan_headings = {}
        completed = False
        self.event_log.append({'timestamp': time.time(), 'type': 'scan_mode', 'mode': mode})
        try:
            if mode == 'chassis':
                self.aim(0)
                for relative in ((0, 1, 2, 3) if scan_rear else (0, 3, 1)):
                    direction = (start_heading + relative) % 4
                    self.face(direction)
                    ranges[direction] = self.sample(0, self.aim(0))
                    self.scan_headings[direction] = self.heading
                self.face(start_heading)
            else:
                # Keep CHASSIS_LEAD, as in the reference repo. No chassis
                # rotation commands or mode switches are issued during this scan.
                if not getattr(self, 'gimbal_reference_ready', False):
                    self.initialize_gimbal_reference()
                else:
                    self.recenter_gimbal()
                # Initial sweep: left -> rear -> directly right, without revisiting left.
                directions = ((0, 0), (3, -90), (2, -180), (1, 90)) if scan_rear else ((0, 0), (3, -90), (1, 90))
                for relative, yaw in directions:
                    direction = (start_heading + relative) % 4
                    ranges[direction] = self.sample(yaw, self.aim(yaw, direct=(yaw == 90)))
                    self.scan_headings[direction] = start_heading
                self.recenter_gimbal()
            self.sample(0, self.aim(0))
            self.ensure_running()
            state = self.hub.get_latest_state()
            completed = True
            self.initial_scan_completed = True
            return ranges, self.heading, state.yaw
        finally:
            self.controller.front_ready = False
            if not completed and mode == 'gimbal' and self.controller.is_running():
                try:
                    self.aim(0)
                except Exception as exc:
                    self.event_log.append({'timestamp': time.time(), 'type': 'gimbal_cleanup', 'error': str(exc)})
            self.scan_origin = None
            if not completed or mode == 'chassis':
                self.robot.gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
                self.controller.stop_chassis()

    def prepare_stationary_scan(self):
        """Stop wheels with acknowledgement before handing control to Gimbal."""
        self.controller.front_ready = False
        stop_chassis_wheels(self.robot.chassis)
        self.event_log.append({'timestamp': time.time(), 'type': 'chassis_wheel_stop', 'acknowledged': True})
        return self.wait_stationary_pose()

    def wait_stationary_pose(self, settle_sec=None):
        """Fixed settling pause and fresh pose; movement guard remains during scanning."""
        if settle_sec is None:
            settle_sec = setting('slam.stationary_settle_sec')
        after = time.monotonic()
        deadline = after + setting('slam.stationary_timeout_sec')
        while time.monotonic() < deadline:
            self.ensure_running()
            state = self.hub.get_latest_state()
            if (time.monotonic() - after >= settle_sec
                    and self.fresh(state, ['position_received_at', 'attitude_received_at'])
                    and state.position_received_at > after and state.attitude_received_at > after):
                if not all(math.isfinite(v) for v in (state.pos_x, state.pos_y, state.yaw)):
                    raise RuntimeError('Invalid chassis pose')
                self.event_log.append({'timestamp': time.time(), 'type': 'chassis_stationary',
                                       'yaw': state.yaw, 'position_m': [state.pos_x, state.pos_y]})
                return state
            time.sleep(0.01)
        raise RuntimeError('No fresh chassis pose after stopping wheels')

    def move(self, direction):
        self.hold_front_for_motion()
        turn = (direction - self.heading) % 4
        if turn:
            self.event_log.append({'timestamp': time.time(), 'type': 'chassis_turn',
                                   'degrees': {1: -90, 2: 180, 3: 90}[turn]})
            self.controller.turn_to_relative({1: -90, 2: 180, 3: 90}[turn])
        self.heading = direction
        self.align_heading()
        distance = self.sample(0, self.aim(0))
        if distance * 1000 <= setting('navigation.front_target_mm'):
            raise RuntimeError('Front obstacle prevents entering the next cell')
        self.controller.front_ready = True
        before = self.hub.get_latest_state()
        if not self.fresh(before, ['position_received_at', 'attitude_received_at']):
            raise RuntimeError('Stale pose before moving')
        self.event_log.append({'timestamp': time.time(), 'type': 'walk_start',
                               'direction': direction, 'cell_size_m': setting('navigation.grid_size_m')})
        result = self.controller.navigate_single_grid_step()
        self.event_log.append(dict(result, timestamp=time.time(), type='walk_end'))
        if not result['completed']:
            raise RuntimeError('Cell motion failed: {}'.format(result['reason']))
        after = self.prepare_stationary_scan()
        return (after.pos_x - before.pos_x, after.pos_y - before.pos_y), after.yaw

    def stop(self):
        self.controller.front_ready = False
        self.controller.stop_running()
