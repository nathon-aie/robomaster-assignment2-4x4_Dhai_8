"""Stationary camera readiness check shared by the CLI and main GUI."""
import argparse
import queue
import sys
import time
from pathlib import Path

from .sdk_connection import initialize_robot, load_robot_sdk, require_camera_codec
from .settings import get as setting, PROJECT_ROOT


def run_camera_check(conn_type=None, timeout=10.0, cancel=None):
    """Receive five decoded frames without commanding chassis or blaster."""
    conn_type = conn_type or setting("robot.conn_type")
    if timeout <= 0:
        raise ValueError("Camera check timeout must be positive")
    if cancel is not None and cancel.is_set():
        return 1
    require_camera_codec()
    robot_module = load_robot_sdk()
    robot = robot_module.Robot()
    camera = None
    streaming = False
    try:
        print("Connecting via {}...".format(conn_type.upper()), flush=True)
        initialize_robot(robot, conn_type)
        if cancel is not None and cancel.is_set():
            return 1
        camera = robot.camera
        if camera.start_video_stream(display=False) is False:
            raise RuntimeError("Camera stream did not start")
        streaming = True

        deadline = time.monotonic() + timeout
        frames = 0
        while frames < 5 and time.monotonic() < deadline:
            if cancel is not None and cancel.is_set():
                print("Camera check stopped by operator.")
                return 1
            try:
                frame = camera.read_cv2_image(strategy="newest", timeout=0.5)
            except queue.Empty:
                continue
            if frame is None:
                continue
            frames += 1
            print("Frame {}: {} dtype={}".format(frames, frame.shape, frame.dtype),
                  flush=True)
        if cancel is not None and cancel.is_set():
            print("Camera check stopped by operator.")
            return 1
        if frames < 5:
            raise RuntimeError("Only {} decoded frames in {:.1f} seconds".format(
                frames, timeout))
        print("PASS: camera decoded 5 frames; chassis and blaster were not used.")
        return 0
    finally:
        try:
            if streaming:
                camera.stop_video_stream()
        finally:
            robot.close()


def main():
    project_python = PROJECT_ROOT / ".venv" / "Scripts" / "python.exe"
    if project_python.exists() and Path(sys.executable).resolve() != project_python.resolve():
        raise SystemExit(
            "Wrong Python environment: {}\n"
            "Run with the project's .venv instead:\n"
            "  {} -m src.check_robot_camera".format(sys.executable, project_python)
        )

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--conn-type", choices=("ap", "sta"),
                        default=setting("robot.conn_type"))
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    return run_camera_check(args.conn_type, args.timeout)
