"""Stationary camera, Gimbal, and blaster check without SLAM or chassis motion."""

import os
import json
import queue
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2

from detect_camera import build_side_by_side_view, detect_signs

from .sdk_connection import initialize_robot, load_robot_sdk, require_camera_codec
from .settings import get as setting, project_path
from .target_fire import SHOTS_PER_TARGET, TargetFireController


TEST_YAWS_DEG = (-60.0, -30.0, 0.0, 30.0, 60.0)
TEST_DWELL_SEC = 1.0
WINDOW_TITLE = "RoboMaster - Stationary Fire Test"


def run_stationary_fire(conn_type="ap", fire_type="water_fire", cancel=None,
                        show_camera=True, on_frame=None):
    """Aim and request three shots per visible target while the chassis stays put."""
    require_camera_codec()
    robot_sdk = load_robot_sdk()
    robot = robot_sdk.Robot()
    stopped = cancel if cancel is not None else threading.Event()
    reader_stop = threading.Event()
    inspection = {"active": False, "phase": "idle", "targets": [], "frame_seq": 0}
    lock = threading.Lock()
    events = []
    reader = None
    scan_worker = None
    stream_started = False
    connected = False
    preview = {"frame": None}
    preview_lock = threading.Lock()
    window_created = False
    report_time = datetime.now()
    if show_camera and on_frame is None and not (
            os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        print("[Fire test] No desktop display; camera preview is unavailable here.")
        show_camera = False

    try:
        initialize_robot(robot, conn_type)
        connected = True
        camera = robot.camera
        if camera.start_video_stream(display=False) is False:
            raise RuntimeError("RoboMaster camera stream could not start")
        stream_started = True
        shooter = TargetFireController(robot, inspection, lock, stopped, fire_type, events)

        def read_camera():
            while not reader_stop.is_set() and not stopped.is_set():
                try:
                    frame = camera.read_cv2_image(strategy="newest", timeout=0.5)
                except queue.Empty:
                    continue
                except Exception as error:
                    print("[Fire test] Camera read warning: {}".format(error))
                    reader_stop.wait(0.1)
                    continue
                if frame is not None:
                    try:
                        result, mask, detections = detect_signs(frame)
                        shooter.observe(detections, frame.shape)
                        if show_camera or on_frame is not None:
                            view = result.copy()
                            height, width = view.shape[:2]
                            cv2.drawMarker(view, (width // 2, height // 2),
                                           (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
                            frame_view = build_side_by_side_view(view, mask)
                            if on_frame is not None:
                                on_frame(frame_view)
                            else:
                                with preview_lock:
                                    preview["frame"] = frame_view
                    except Exception as error:
                        print("[Fire test] Detection warning: {}".format(error))

        reader = threading.Thread(target=read_camera, name="fire-test-camera", daemon=True)
        reader.start()
        print("[Fire test] Stationary test: Gimbal looks down, aims and fires; chassis stays put.")
        print("[Fire test] Mode: {}".format(fire_type))
        def scan_and_fire():
            pitch = setting("fire.scan_pitch_deg")
            for yaw in TEST_YAWS_DEG:
                if stopped.is_set():
                    break
                with lock:
                    inspection["active"] = False
                try:
                    action = robot.gimbal.moveto(
                        yaw=yaw, pitch=pitch,
                        yaw_speed=setting("gimbal.yaw_speed_dps"),
                        pitch_speed=setting("gimbal.pitch_speed_dps"),
                    )
                    if (action.wait_for_completed(timeout=setting("gimbal.action_timeout_sec")) is False
                            or getattr(action, "has_succeeded", True) is False):
                        print("[Fire test] Gimbal did not reach yaw {:.0f}; continuing.".format(yaw))
                        continue
                    stopped.wait(setting("gimbal.settle_sec"))
                    with lock:
                        inspection.update(active=True, phase="survey",
                                          camera_yaw=yaw, camera_pitch=pitch)
                    stopped.wait(TEST_DWELL_SEC)
                except Exception as error:
                    print("[Fire test] Gimbal warning at yaw {:.0f}: {}".format(yaw, error))

            if not stopped.is_set():
                shooter.fire_confirmed((0, 0), 0)

        if show_camera or on_frame is not None:
            scan_worker = threading.Thread(target=scan_and_fire, name="fire-test-gimbal")
            scan_worker.start()
            if on_frame is None:
                print("[Fire test] Live camera open; press q or close the window to stop.")
            while scan_worker.is_alive() and not stopped.is_set():
                if on_frame is not None:
                    scan_worker.join(timeout=0.05)
                    continue
                with preview_lock:
                    frame = preview["frame"]
                if frame is not None:
                    cv2.imshow(WINDOW_TITLE, frame)
                    window_created = True
                key = cv2.waitKey(20) & 0xFF
                if key == ord("q"):
                    stopped.set()
                    break
                if window_created:
                    try:
                        if cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_VISIBLE) < 1:
                            stopped.set()
                            break
                    except cv2.error:
                        stopped.set()
                        break
            scan_worker.join()
        else:
            scan_and_fire()

        fired = SHOTS_PER_TARGET * sum(event["fired"] for event in events)
        print("[Fire test] Confirmed targets: {} | shots: {}".format(
            sum(target["confirmed"] for target in inspection["targets"]), fired
        ))
        return fired
    finally:
        if scan_worker is not None and scan_worker.is_alive():
            stopped.set()
            scan_worker.join(timeout=setting("gimbal.action_timeout_sec") + 1)
        reader_stop.set()
        with lock:
            inspection["active"] = False
        if reader is not None:
            reader.join(timeout=1.5)
        if window_created:
            try:
                cv2.destroyWindow(WINDOW_TITLE)
            except cv2.error:
                pass
        if connected:
            try:
                robot.gimbal.moveto(
                    yaw=0, pitch=setting("gimbal.pitch_deg"),
                    yaw_speed=setting("gimbal.yaw_speed_dps"),
                    pitch_speed=setting("gimbal.pitch_speed_dps"),
                ).wait_for_completed(timeout=setting("gimbal.action_timeout_sec"))
            except Exception as error:
                print("[Fire test] Gimbal reset warning: {}".format(error))
        if stream_started:
            try:
                camera.stop_video_stream()
            except Exception as error:
                print("[Fire test] Camera stop warning: {}".format(error))
        try:
            robot.close()
        except Exception as error:
            print("[Fire test] Robot close warning: {}".format(error))
        try:
            report_dir = Path(project_path("paths.telemetry")) / "fire_tests"
            report_dir.mkdir(parents=True, exist_ok=True)
            report_path = report_dir / ("fire_test_{}.json".format(
                report_time.strftime("%Y%m%d_%H%M%S_%f")))
            report = {
                "started_at": report_time.isoformat(),
                "fire_mode": fire_type,
                "shots_requested": sum(event["shots_requested"] for event in events),
                "shots_fired": SHOTS_PER_TARGET * sum(event["fired"] for event in events),
                "targets": events,
            }
            report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2),
                                   encoding="utf-8")
            print("[Fire test] Saved target colors, shapes and shots: {}".format(report_path))
        except Exception as error:
            print("[Fire test] Could not save shot report: {}".format(error))
