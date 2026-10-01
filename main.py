#!/usr/bin/env python3
"""RoboMaster EP operations selected exclusively through the main menu."""
import json
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import List

from src.settings import get as setting, project_path
from src.robot_system import RobotSystem
from src.telemetry import TelemetryAnalyzer
from src.operation_menu import OperationGUI
from src.run_log import capture_console


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


def run_motion(commands, speed, timeout, control=None):
    """Execute menu-selected commands and shut down once on completion or interruption."""
    system = RobotSystem()
    if control:
        control.set_system(system)
    try:
        if not system.connect_robot():
            return 1
        if control and control.cancel.is_set():
            return 1
        system.setup_threads()
        system.thread_2_controller.base_speed = speed
        system.thread_2_controller.set_commands(commands)
        system.start()
        started = time.monotonic()
        while True:
            controller = system.thread_2_controller
            if control and control.cancel.is_set():
                completed = False
                break
            if controller.failure:
                raise RuntimeError(controller.failure)
            if controller.commands_completed:
                completed = True
                break
            if timeout and time.monotonic() - started > timeout:
                completed = False
                break
            time.sleep(0.1)
        if not completed:
            print('[Motion] หยุดงานหรือหมดเวลาทดสอบ')
        return 0 if completed else 1
    finally:
        system.shutdown()


def run_motion_test(commands, control=None):
    commands = parse_custom_commands(commands)
    if not commands:
        return 0
    duration = setting('navigation.duration_sec')
    return run_motion(commands, setting('navigation.base_speed_mps'),
                      duration if duration > 0 else None, control=control)


def test_step(control=None):
    print('[Motion] ทดสอบเดินหน้า 1 ช่อง พร้อม Sharp Centering และ ToF กันชน')
    return run_motion(['Move Forward: 1 cells'], setting('navigation.step_test_speed_mps'),
                      setting('system.step_test_timeout_sec'), control=control)


def test_turn(direction='right', control=None):
    commands = {'left': 'Turn Left (90 deg)', 'right': 'Turn Right (90 deg)',
                'around': 'Turn Around (180 deg)'}
    return run_motion([commands[direction]], setting('navigation.base_speed_mps'),
                      setting('system.turn_test_timeout_sec'), control=control)


def monitor_sensors(control=None):
    print("=" * 65)
    print("📡 LIVE SENSOR MONITOR (THREAD 1)")
    print("=" * 65)
    sys_runner = RobotSystem()
    if control:
        control.set_system(sys_runner)
    if not sys_runner.connect_robot():
        sys_runner.shutdown(save_telemetry=False, run_analysis=False)
        return 1
    try:
        if control and control.cancel.is_set():
            return 1
        sys_runner.setup_threads()
        sys_runner.thread_1_sensor.start_collecting()
        print(f"{'Frame':<8} | {'Sharp L (mm)':<13} | {'Sharp R (mm)':<13} | {'ToF (mm)':<10} | {'Yaw (deg)':<10} | {'Walls (L/F/R)'}")
        print("-" * 75)
        while not (control and control.cancel.is_set()):
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
    return 0


def analyze_log(file):
    print('Analyzing telemetry log: {}'.format(file))
    TelemetryAnalyzer.analyze_file(file, save_plot=True)
    return 0


def calibrate_sensors(action, reference_provider=None, control=None):
    from src.calibrate import CalibrationSession, fit_command
    if action == 'fit':
        fit_command(project_path('paths.measurements'), project_path('paths.calibration_output'))
        return 0
    session = CalibrationSession(setting('robot.conn_type'))
    try:
        session.collect(action, project_path('paths.measurements'), None, None,
                        setting('sensors.tof_index'), setting('calibration.samples'),
                        reference_provider=reference_provider)
        return 1 if control and control.cancel.is_set() else 0
    finally:
        session.close()


def test_gimbal(control=None):
    """Scan without starting the queued motion worker."""
    from src.slam_hardware import HardwareBackend
    from src.sdk_connection import cancel_chassis_speed_timer
    cycles = setting('gimbal.test_cycles')
    if cycles < 1:
        print('[Gimbal test] จำนวนรอบต้องมากกว่า 0')
        return 1
    system = RobotSystem()
    if control:
        control.set_system(system)
    backend = None
    status, error = 'failed', ''
    try:
        if not system.connect_robot():
            return 1
        if control and control.cancel.is_set():
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
        cancel_chassis_speed_timer(system.robot.chassis)
        stopped = system.robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        if stopped is not True:
            print('[Gimbal test] ส่งคำสั่งหยุดล้อแล้ว แต่ SDK ไม่ตอบรับ — เริ่มทดสอบ Gimbal')
        backend.wait_stationary_pose()
        for cycle in range(cycles):
            if control and control.cancel.is_set():
                status = 'interrupted'
                break
            print('[Gimbal test] รอบ {}/{} — สแกน {} ทิศ (Ctrl+C เพื่อหยุด)'.format(
                cycle + 1, cycles, 4 if cycle == 0 else 3))
            ranges, _, _ = backend.scan(mode='gimbal')
            print('[Gimbal test] ToF (m): {}'.format(ranges))
        if status != 'interrupted':
            status = 'completed'
    except (Exception, KeyboardInterrupt) as exc:
        status = 'interrupted' if isinstance(exc, KeyboardInterrupt) else 'failed'
        error = str(exc)
        print('[Gimbal test] {}: {}'.format(status, error or 'หยุดโดยผู้ใช้'))
    finally:
        system.shutdown(save_telemetry=False, run_analysis=False)
    return 0 if status == 'completed' else 1


def run_exploration(on_map_ready=None, control=None):
    from src.grid_slam import FrontierExplorer, GridSLAM
    from src.slam_hardware import HardwareBackend
    system = RobotSystem()
    explorer = None
    recorder = system.telemetry
    map_file = Path(recorder.run_dir) / '{}_{}_map.json'.format(recorder.run_name, recorder.timestamp_str)
    GridSLAM().export(map_file, 'not_started')
    status, error = 'not_started', ''
    with capture_console(map_file.parent / 'console.log'):
        if on_map_ready:
            on_map_ready(map_file)
        if control:
            control.set_system(system)
        try:
            if not system.connect_robot():
                raise RuntimeError('Could not connect to RoboMaster EP')
            if control and control.cancel.is_set():
                raise KeyboardInterrupt
            system.setup_threads()
            system.thread_1_sensor.start_collecting()
            system.thread_2_controller.enable_motion()
            explorer = FrontierExplorer(HardwareBackend(system), map_file)
            deadline = time.monotonic() + setting("slam.sensor_timeout_sec")
            while system.sensor_hub.get_latest_state().frame_index == 0:
                if time.monotonic() > deadline:
                    raise RuntimeError("No initial position/attitude data")
                time.sleep(0.01)
            success = explorer.run()
        except (Exception, KeyboardInterrupt) as exc:
            status = 'interrupted' if isinstance(exc, KeyboardInterrupt) else 'failed'
            error = str(exc)
            if explorer is None:
                print("[SLAM] Startup failed: {}".format(exc))
                GridSLAM().export(map_file, status, error)
            else:
                explorer.status, explorer.error = status, error
                explorer.slam.export(explorer.output, status, error)
            success = False
        finally:
            try:
                system.shutdown()
            except Exception as error:
                print('[SLAM] Telemetry shutdown warning: {}'.format(error))
                success = False
        from src.slam_report import save_report
        plot, actions = save_report(map_file)
        print("[SLAM] map={} | actions={}".format(plot, actions))
        print("[SLAM] {} | visited={} | moves={} | data={}".format(
            explorer.status if explorer else status,
            len(explorer.slam.map.visited) if explorer else 0,
            explorer.moves if explorer else 0, map_file))
        if explorer and explorer.error:
            print("[SLAM] " + explorer.error)
        return 0 if success else 1


def run_exploration_detection(conn_type=None, mock=False, on_map_ready=None,
                              control=None, fire_type='water_fire', on_frame=None,
                              target_color='Red', target_shape='All'):
    """Explore with SLAM, detect signs, and fire at matching confirmed targets."""
    from src import slam_detect_camera
    conn_type = conn_type or setting('robot.conn_type')
    calib = Path(project_path('paths.calibration'))
    if not calib.is_absolute():
        calib = Path(__file__).resolve().parent / calib
    mission_dir = Path(project_path('paths.telemetry'))
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output_path = mission_dir / f"run_detect_{timestamp}" / "explored_map.json"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    from src.grid_slam import GridSLAM
    GridSLAM().export(output_path, 'not_started')

    with capture_console(output_path.parent / 'console.log'):
        if on_map_ready:
            on_map_ready(output_path)

        success = False
        try:
            if mock:
                success = slam_detect_camera.run_simulation(
                    camera_index=0, output=output_path, control=control, on_frame=on_frame)
            else:
                success = slam_detect_camera.run_hardware(
                    conn_type, calib, output_path, control=control, fire_type=fire_type,
                    on_frame=on_frame, target_color=target_color, target_shape=target_shape,
                )
        except (Exception, KeyboardInterrupt) as exc:
            status = 'interrupted' if isinstance(exc, KeyboardInterrupt) else 'failed'
            detail = str(exc) or 'Interrupted by user'
            print(f"[Explore-Detect] {status}: {detail}")
            _mark_unfinished_map(output_path, status, detail)
        finally:
            if not success:
                status = 'interrupted' if control and control.cancel.is_set() else 'failed'
                _mark_unfinished_map(output_path, status, 'Mission stopped before exploration completed')
            if output_path.exists():
                try:
                    map_img = slam_detect_camera.render_map_image(output_path)
                    print(f"[Explore-Detect] 🗺️ บันทึกภาพแผนที่สำเร็จ: {map_img}")
                except Exception as e:
                    print(f"[Explore-Detect] Warning: ไม่สามารถเรนเดอร์ภาพแผนที่ได้: {e}")
        if output_path.exists():
            try:
                slam_detect_camera.save_mission_reports(output_path)
            except Exception as error:
                print('[Explore-Detect] Report warning: {}'.format(error))
        return 0 if success else 1


def _mark_unfinished_map(path, status, error):
    """Preserve a partial map while recording failures before the explorer starts."""
    if not path.exists():
        return
    data = json.loads(path.read_text(encoding='utf-8'))
    if data.get('status') not in ('not_started', 'running'):
        return
    data['status'], data['error'] = status, error
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    temp.replace(path)


def run_camera_detection(mode: str = 'robot-ap', control=None, on_frame=None):
    """Live target detection with 3 confirmation snapshots."""
    from src import detect_camera
    try:
        detect_camera.main(
            on_frame=on_frame,
            cancel=control.cancel if control is not None else None,
            webcam=mode == 'webcam',
            conn_type='sta' if mode == 'robot-sta' else 'ap',
        )
        return 0
    except Exception as exc:
        print(f"[Detect-Camera] Error: {exc}")
        return 1


def run_fire_test(conn_type=None, fire_type='water_fire', control=None, on_frame=None,
                  target_color='Red', target_shape='All'):
    """Use only the camera, Gimbal, and blaster while the chassis stays still."""
    from src.fire_test import run_stationary_fire

    cancel = control.cancel if control is not None else None
    run_stationary_fire(
        conn_type=conn_type or setting('robot.conn_type'),
        fire_type=fire_type,
        target_color=target_color,
        target_shape=target_shape,
        cancel=cancel,
        on_frame=on_frame,
    )
    return 1 if cancel is not None and cancel.is_set() else 0


def main():
    def interrupt_on_termination(_signum, _frame):
        raise KeyboardInterrupt

    termination_signals = [signal.SIGTERM]
    if hasattr(signal, 'SIGHUP'):
        termination_signals.append(signal.SIGHUP)
    previous_handlers = {signum: signal.getsignal(signum) for signum in termination_signals}
    for signum in termination_signals:
        signal.signal(signum, interrupt_on_termination)
    try:
        gui = OperationGUI(run_selected)
        gui.run()
        if gui.proceed_to_explore:
            return run_exploration_detection(
                conn_type=gui.selected_conn_type,
                fire_type=gui.selected_fire_type,
                target_color=gui.selected_target_color,
                target_shape=gui.selected_target_shape,
            )
        return 0
    except KeyboardInterrupt:
        print('[Main] หยุดงานตามคำสั่ง interrupt', file=sys.stderr)
        return 130
    except Exception as exc:
        print('[GUI] {}'.format(exc), file=sys.stderr)
        return 1
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


def run_selected(task, parameters, gui, control):
    from src.camera_check import run_camera_check

    handlers = {
        'camera-check': lambda: run_camera_check(
            conn_type=parameters.get('conn_type', setting('robot.conn_type')),
            cancel=control.cancel,
        ),
        'explore': lambda: run_exploration(on_map_ready=gui.show_map, control=control),
        'explore-detect': lambda: run_exploration_detection(
            conn_type=parameters.get('conn_type', setting('robot.conn_type')),
            mock=parameters.get('mock', False),
            on_map_ready=gui.show_map,
            control=control,
            fire_type=parameters.get('fire_type', 'water_fire'),
            target_color=parameters.get('target_color', 'Red'),
            target_shape=parameters.get('target_shape', 'All'),
            on_frame=gui.show_fire_frame,
        ),
        'detect-camera': lambda: run_camera_detection(
            parameters.get('mode', 'robot-ap'), control=control,
            on_frame=gui.show_fire_frame),
        'detect-webcam': lambda: run_camera_detection(
            'webcam', control=control, on_frame=gui.show_fire_frame),
        'fire-test': lambda: run_fire_test(
            conn_type=parameters.get('conn_type', setting('robot.conn_type')),
            fire_type=parameters.get('fire_type', 'water_fire'),
            target_color=parameters.get('target_color', 'Red'),
            target_shape=parameters.get('target_shape', 'All'),
            control=control,
            on_frame=gui.show_fire_frame,
        ),
        'step-test': lambda: test_step(control=control),
        'turn-test': lambda: test_turn(parameters.get('direction', 'right'), control=control),
        'monitor': lambda: monitor_sensors(control=control),
        'motion': lambda: run_motion_test(parameters.get('commands', ''), control=control),
        'calibrate': lambda: calibrate_sensors(parameters.get('action'),
                                               reference_provider=gui.ask_reference,
                                               control=control),
        'analysis': lambda: analyze_log(parameters.get('file')),
        'gimbal-test': lambda: test_gimbal(control=control),
    }
    return handlers[task]() or 0



if __name__ == '__main__':
    sys.exit(main())
