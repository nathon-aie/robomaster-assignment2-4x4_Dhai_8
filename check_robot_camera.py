"""Stationary RoboMaster camera smoke test; never moves or fires the robot."""

import argparse
import queue
import sys
import time
from pathlib import Path

from src.sdk_connection import initialize_robot, load_robot_sdk, require_camera_codec
from src.settings import get as setting


def main():
    project_python = Path(__file__).resolve().parent / ".venv" / "Scripts" / "python.exe"
    if project_python.exists() and Path(sys.executable).resolve() != project_python.resolve():
        raise SystemExit(
            "Wrong Python environment: {}\n"
            "Run with the project's .venv instead:\n"
            "  {} {}".format(sys.executable, project_python, Path(__file__).resolve())
        )

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--conn-type", choices=("ap", "sta"),
                        default=setting("robot.conn_type"))
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    require_camera_codec()
    robot_module = load_robot_sdk()
    robot = robot_module.Robot()
    camera = None
    streaming = False
    try:
        print("Connecting via {}...".format(args.conn_type.upper()), flush=True)
        initialize_robot(robot, args.conn_type)
        camera = robot.camera
        if camera.start_video_stream(display=False) is False:
            raise RuntimeError("Camera stream did not start")
        streaming = True

        deadline = time.monotonic() + args.timeout
        frames = 0
        while frames < 5 and time.monotonic() < deadline:
            try:
                frame = camera.read_cv2_image(strategy="newest", timeout=0.5)
            except queue.Empty:
                continue
            if frame is None:
                continue
            frames += 1
            print("Frame {}: {} dtype={}".format(frames, frame.shape, frame.dtype),
                  flush=True)
        if frames < 5:
            raise RuntimeError("Only {} decoded frames in {:.1f} seconds".format(
                frames, args.timeout))
        print("PASS: camera decoded 5 frames; chassis and blaster were not used.")
    finally:
        if streaming:
            camera.stop_video_stream()
        robot.close()


if __name__ == "__main__":
    main()
