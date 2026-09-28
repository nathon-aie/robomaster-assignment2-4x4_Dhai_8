#!/usr/bin/env python3
"""RoboMaster EP operations selected exclusively through the main menu."""
import json
import sys
import time
from pathlib import Path
from typing import List

from src.settings import get as setting, project_path
from src.robot_system import RobotSystem
from src.telemetry import TelemetryAnalyzer
from src.operation_menu import select_operation, calibration_action


def parse_custom_commands(cmd_input: str) -> List[str]:
    """Parses arbitrary command string or JSON list into robot controller commands."""
    if not cmd_input:
        return []
    s = cmd_input.strip()
    if s.startswith("["):
        try:
            return json.loads(s)
        except Exception:
            pass

    raw_items = [c.strip() for c in s.replace(";", ",").split(",") if c.strip()]
    parsed = []
    for item in raw_items:
        low = item.lower()
        if any(w in low for w in ("fwd", "forward", "move", "cell")):
            nums = [int(tok) for tok in item.split() if tok.isdigit()]
            cells = nums[0] if nums else 1
            parsed.append(f"Move Forward: {cells} cells")
        elif "left" in low:
            parsed.append("Turn Left (90 deg)")
        elif "right" in low:
            parsed.append("Turn Right (90 deg)")
        elif "around" in low or "180" in low:
            parsed.append("Turn Around (180 deg)")
        else:
            parsed.append(item)
    return parsed


def run_motion(commands, speed, timeout):
    """Execute menu-selected commands and shut down once on completion or interruption."""
    system = RobotSystem()
    try:
        if not system.connect_robot():
            return 1
        system.setup_threads()
        system.thread_2_controller.base_speed = speed
        system.thread_2_controller.set_commands(commands)
        system.start()
        completed = system.wait_for_completion(timeout=timeout)
        if not completed:
            print('[Motion] หมดเวลาทดสอบ')
        return 0 if completed else 1
    finally:
        system.shutdown()


def run_motion_test(commands):
    commands = parse_custom_commands(commands)
    if not commands:
        return 0
    duration = setting('navigation.duration_sec')
    return run_motion(commands, setting('navigation.base_speed_mps'),
                      duration if duration > 0 else None)


def test_step():
    print('[Motion] ทดสอบเดินหน้า 1 ช่อง พร้อม Sharp Centering และ ToF กันชน')
    return run_motion(['Move Forward: 1 cells'], setting('navigation.step_test_speed_mps'),
                      setting('system.step_test_timeout_sec'))


def test_turn(direction):
    commands = {'left': 'Turn Left (90 deg)', 'right': 'Turn Right (90 deg)',
                'around': 'Turn Around (180 deg)'}
    return run_motion([commands[direction]], setting('navigation.base_speed_mps'),
                      setting('system.turn_test_timeout_sec'))


def monitor_sensors():
    print("=" * 65)
    print("📡 LIVE SENSOR MONITOR (THREAD 1)")
    print("=" * 65)
    sys_runner = RobotSystem()
    if not sys_runner.connect_robot():
        sys_runner.shutdown(save_telemetry=False, run_analysis=False)
        return 1
    try:
        sys_runner.setup_threads()
        sys_runner.thread_1_sensor.start_collecting()
        print(f"{'Frame':<8} | {'Sharp L (mm)':<13} | {'Sharp R (mm)':<13} | {'ToF (mm)':<10} | {'Yaw (deg)':<10} | {'Walls (L/F/R)'}")
        print("-" * 75)
        while True:
            state = sys_runner.sensor_hub.wait_for_next_state(timeout=1.0)
            if state:
                sl = f"{state.sharp_left_mm:.1f}" if state.sharp_left_mm is not None else "N/A"
                sr = f"{state.sharp_right_mm:.1f}" if state.sharp_right_mm is not None else "N/A"
                tof = f"{state.tof_filtered_mm:.1f}" if state.tof_filtered_mm is not None else "N/A"
                walls = f"{'L' if state.wall_left_detected else '-'}/{'F' if state.wall_front_detected else '-'}/{'R' if state.wall_right_detected else '-'}"
                print(f"{state.frame_index:<8} | {sl:<13} | {sr:<13} | {tof:<10} | {state.yaw:<10.1f} | {walls}")
            time.sleep(0.1)
    finally:
        sys_runner.shutdown(save_telemetry=False)


def analyze_log(file):
    print('Analyzing telemetry log: {}'.format(file))
    TelemetryAnalyzer.analyze_file(file, save_plot=True)
    return 0


def calibrate_sensors(action):
    from src.calibrate import CalibrationSession, fit_command
    session = CalibrationSession(setting('robot.conn_type'))
    status = 0
    try:
        while action is not None:
            try:
                if action == 'fit':
                    fit_command(project_path('paths.measurements'), project_path('paths.calibration_output'))
                else:
                    session.collect(action, project_path('paths.measurements'), None, None,
                                    setting('sensors.tof_index'), setting('calibration.samples'))
            except (OSError, RuntimeError, ValueError) as exc:
                print('[calibration] {}'.format(exc), file=sys.stderr)
                status = 1
            action = calibration_action()
    except (EOFError, KeyboardInterrupt):
        print('\n[calibration] ออกจากเมนู Calibration')
    finally:
        session.close()
    return status


def test_gimbal():
    """Scan without starting the queued motion worker."""
    from src.slam_hardware import HardwareBackend
    from src.sdk_connection import cancel_chassis_speed_timer
    cycles = setting('gimbal.test_cycles')
    if cycles < 1:
        print('[Gimbal test] จำนวนรอบต้องมากกว่า 0')
        return 1
    system = RobotSystem()
    backend = None
    status, error = 'failed', ''
    try:
        if not system.connect_robot():
            return 1
        system.setup_threads()
        system.thread_1_sensor.start_collecting()
        system.thread_2_controller.enable_motion()
        backend = HardwareBackend(system)
        deadline = time.monotonic() + setting('slam.sensor_timeout_sec')
        while system.sensor_hub.get_latest_state().frame_index == 0:
            if time.monotonic() > deadline:
                raise RuntimeError('No initial position/attitude data')
            time.sleep(0.01)
        # This test never starts chassis motion. Send one stop request, but a
        # missing ACK must not prevent testing the Gimbal on an idle chassis.
        cancel_chassis_speed_timer(system.robot.chassis)
        stopped = system.robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        if stopped is not True:
            print('[Gimbal test] ส่งคำสั่งหยุดล้อแล้ว แต่ SDK ไม่ตอบรับ — เริ่มทดสอบ Gimbal')
        backend.wait_stationary_pose()
        for cycle in range(cycles):
            print('[Gimbal test] รอบ {}/{} — สแกน {} ทิศ (Ctrl+C เพื่อหยุด)'.format(
                cycle + 1, cycles, 4 if cycle == 0 else 3))
            # Always use Gimbal scans, regardless of the exploration scan mode.
            ranges, _, _ = backend.scan(mode='gimbal')
            print('[Gimbal test] ToF (m): {}'.format(ranges))
        status = 'completed'
    except (Exception, KeyboardInterrupt) as exc:
        status = 'interrupted' if isinstance(exc, KeyboardInterrupt) else 'failed'
        error = str(exc)
        print('[Gimbal test] {}: {}'.format(status, error or 'หยุดโดยผู้ใช้'))
    finally:
        system.shutdown(save_telemetry=False, run_analysis=False)
    return 0 if status == 'completed' else 1


def run_exploration():
    from src.grid_slam import DFSExplorer
    from src.slam_hardware import HardwareBackend
    system = RobotSystem()
    explorer = None
    recorder = system.telemetry
    map_file = Path(recorder.run_dir) / '{}_{}_map.json'.format(recorder.run_name, recorder.timestamp_str)
    try:
        if not system.connect_robot():
            return 1
        system.setup_threads()
        system.thread_1_sensor.start_collecting()
        system.thread_2_controller.enable_motion()
        explorer = DFSExplorer(HardwareBackend(system), map_file)
        deadline = time.monotonic() + setting("slam.sensor_timeout_sec")
        while system.sensor_hub.get_latest_state().frame_index == 0:
            if time.monotonic() > deadline:
                raise RuntimeError("No initial position/attitude data")
            time.sleep(0.01)
        success = explorer.run()
    except (Exception, KeyboardInterrupt) as exc:
        if explorer is None:
            print("[SLAM] Startup failed: {}".format(exc))
            return 1
        explorer.status = "interrupted" if isinstance(exc, KeyboardInterrupt) else "failed"
        explorer.error = str(exc)
        explorer.slam.export(explorer.output, explorer.status, explorer.error)
        success = False
    finally:
        system.shutdown()
    from src.slam_report import save_report
    plot, actions = save_report(explorer.output)
    print("[SLAM] map={} | actions={}".format(plot, actions))
    print("[SLAM] {} | visited={} | moves={} | data={}".format(
        explorer.status, len(explorer.slam.map.visited), explorer.moves, explorer.output))
    if explorer.error:
        print("[SLAM] " + explorer.error)
    return 0 if success else 1


def main():
    selection = select_operation()
    if selection is None:
        return 0
    task, parameters = selection
    handlers = {
        'explore': run_exploration,
        'step-test': test_step,
        'turn-test': test_turn,
        'monitor': monitor_sensors,
        'motion': run_motion_test,
        'calibrate': calibrate_sensors,
        'analysis': analyze_log,
        'gimbal-test': test_gimbal,
    }
    try:
        return handlers[task](**parameters) or 0
    except (EOFError, KeyboardInterrupt):
        print('\nหยุดการทำงาน')
        return 0
    except (OSError, RuntimeError, ValueError) as exc:
        print('[main] {}'.format(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
