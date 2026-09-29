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
        completed = system.wait_for_completion(timeout=timeout)
        if not completed:
            print('[Motion] หมดเวลาทดสอบ')
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
    status = 0
    try:
        if reference_provider or control:
            session.collect(action, project_path('paths.measurements'), None, None,
                            setting('sensors.tof_index'), setting('calibration.samples'),
                            reference_provider=reference_provider)
            return 1 if control and control.cancel.is_set() else 0
        while action is not None:
            try:
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
    from src.grid_slam import FrontierExplorer
    from src.slam_hardware import HardwareBackend
    system = RobotSystem()
    explorer = None
    recorder = system.telemetry
    map_file = Path(recorder.run_dir) / '{}_{}_map.json'.format(recorder.run_name, recorder.timestamp_str)
    if on_map_ready:
        on_map_ready(map_file)
    if control:
        control.set_system(system)
    try:
        if not system.connect_robot():
            return 1
        if control and control.cancel.is_set():
            return 1
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


def run_exploration_detection(conn_type=None, mock=False, on_map_ready=None, control=None):
    """Explore with SLAM + camera sign detection (captures 3 confirmation images)."""
    import slam_detect_camera
    slam_detect_camera.clear_previous_captures()
    conn_type = conn_type or setting('robot.conn_type')
    calib = Path(project_path('paths.calibration'))
    if not calib.is_absolute():
        calib = Path(__file__).resolve().parent / calib
    mission_dir = Path(project_path('paths.telemetry'))
    timestamp = time.strftime('%Y%m%d_%H%M%S')
    output_path = mission_dir / f"run_detect_{timestamp}" / "explored_map.json"
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if on_map_ready:
        on_map_ready(output_path)

    success = False
    try:
        if mock:
            success = slam_detect_camera.run_simulation(camera_index=0, output=output_path, control=control)
        else:
            success = slam_detect_camera.run_hardware(conn_type, calib, output_path, control=control)
    except (Exception, KeyboardInterrupt) as exc:
        print(f"[Explore-Detect] Note: {exc}")
    finally:
        if output_path.exists():
            try:
                map_img = slam_detect_camera.render_map_image(output_path)
                print(f"[Explore-Detect] 🗺️ บันทึกภาพแผนที่สำเร็จ: {map_img}")
            except Exception as e:
                print(f"[Explore-Detect] Warning: ไม่สามารถเรนเดอร์ภาพแผนที่ได้: {e}")
    return 0 if success else 1


def run_camera_detection(mode: str = 'robot-ap'):
    """Live target detection with 3 confirmation snapshots."""
    import detect_camera
    orig_argv = list(sys.argv)
    if mode == 'webcam':
        sys.argv = ['detect_camera.py', '--webcam']
    elif mode == 'robot-sta':
        sys.argv = ['detect_camera.py', '--conn-type', 'sta']
    else:
        sys.argv = ['detect_camera.py', '--conn-type', 'ap']
    try:
        detect_camera.main()
        return 0
    except Exception as exc:
        print(f"[Detect-Camera] Error: {exc}")
        return 1
    finally:
        sys.argv = orig_argv


def run_menu():
    """Interactive menu fallback."""
    selection = select_operation()
    if selection is None:
        return 0
    task, parameters = selection
    handlers = {
        'explore': run_exploration,
        'explore-detect': run_exploration_detection,
        'detect-camera': run_camera_detection,
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


def run_selected(task, parameters, gui, control):
    handlers = {
        'explore': lambda: run_exploration(on_map_ready=gui.show_map, control=control),
        'explore-detect': lambda: run_exploration_detection(
            conn_type=parameters.get('conn_type', setting('robot.conn_type')),
            mock=parameters.get('mock', False),
            on_map_ready=gui.show_map,
            control=control
        ),
        'detect-camera': lambda: run_camera_detection(parameters.get('mode', 'robot-ap')),
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


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="RoboMaster EP - ระบบสำรวจ Grid SLAM และตรวจจับเป้าหมาย (Auto Capture 3 รูป)"
    )
    parser.add_argument(
        "--cli", action="store_true",
        help="รันตรงใน Command-line ทันที โดยไม่ต้องเปิดหน้าต่าง GUI ก่อน",
    )
    parser.add_argument(
        "--gui", action="store_true",
        help="เปิดหน้าต่าง GUI (ค่าเริ่มต้นเปิดให้อัตโนมัติอยู่แล้ว)",
    )
    parser.add_argument(
        "--no-camera", action="store_true",
        help="ให้หุ่นเคลื่อนที่สำรวจอย่างเดียว โดยไม่ต้องใช้กล้อง / ไม่ตรวจจับเป้าหมาย",
    )
    parser.add_argument(
        "--conn-type", choices=("ap", "sta"), default=setting("robot.conn_type"),
        help="โหมดการเชื่อมต่อหุ่นยนต์ (ap หรือ sta, ค่าเริ่มต้น: ap)",
    )
    parser.add_argument(
        "--mock", action="store_true",
        help="รันโหมดจำลอง (Simulator)",
    )
    parser.add_argument(
        "--step", action="store_true",
        help="ทดสอบเดินหน้า 1 ช่อง (เคลื่อนที่อย่างเดียว ไม่ใช้กล้อง)",
    )
    parser.add_argument(
        "--turn", choices=("left", "right", "around"), default=None,
        help="ทดสอบเลี้ยว (left=ซ้าย 90 องศา, right=ขวา 90 องศา, around=กลับหลัง 180 องศา)",
    )
    parser.add_argument(
        "--menu", action="store_true",
        help="เปิดเมนูเลือกคำสั่งแบบ Interactive Text Menu",
    )
    args, unknown = parser.parse_known_args()

    try:
        # 1. กรณีระบุคำสั่งเจาะจงผ่าน CLI
        if args.menu:
            return run_menu()
        if args.step:
            return test_step()
        if args.turn:
            return test_turn(args.turn)
        if args.no_camera:
            print("[SLAM] เริ่มการสำรวจและสร้างแผนที่ (เคลื่อนที่อย่างเดียว ไม่ใช้กล้อง)")
            return run_exploration()
        if args.cli:
            print("=" * 65)
            print("🚀 เริ่มภารกิจ (CLI Mode): สำรวจ Grid SLAM + ตรวจจับเป้าหมาย (Auto Capture 3 รูป)")
            print(f"📡 โหมดเชื่อมต่อ: {args.conn_type.upper()}")
            print("=" * 65)
            return run_exploration_detection(conn_type=args.conn_type, mock=args.mock)

        # 2. ค่าเริ่มต้น (Default): เปิดหน้าต่าง GUI ขึ้นมาก่อนเสมอ
        from src.operation_menu import OperationGUI
        gui = OperationGUI(run_selected)
        gui.run()

        # ถ้าผู้ใช้กดปุ่ม 'ปิด GUI แล้วเริ่มสำรวจทันที' จากในหน้าต่าง GUI
        if getattr(gui, 'proceed_to_explore', False):
            conn_type = getattr(gui, 'selected_conn_type', args.conn_type)
            print("=" * 65)
            print("🚀 เริ่มภารกิจ: สำรวจ Grid SLAM + ตรวจจับเป้าหมาย (Auto Capture 3 รูป)")
            print(f"📡 โหมดเชื่อมต่อ: {conn_type.upper()}")
            print("=" * 65)
            return run_exploration_detection(conn_type=conn_type, mock=args.mock)

        return 0

    except (EOFError, KeyboardInterrupt):
        print('\nหยุดการทำงานโดยผู้ใช้')
        return 0
    except Exception as exc:
        print(f'[main] Error: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
