"""Aim the RoboMaster blaster from the cell where a camera target was found."""

import math
import time

from .settings import get as setting


FIRE_TYPES = {
    "water_fire": "water",
    "infared_fire": "ir",  # Keep the spelling used by the mission controls.
    "infrared_fire": "ir",
}
SHOTS_PER_TARGET = 3
TARGET_COLORS = ("Red", "Yellow", "Blue", "Green")
TARGET_SHAPES = ("Circle", "Square", "Vertical_Rect", "Horizontal_Rect", "All")


def fire_type_for_sdk(mode):
    return FIRE_TYPES[mode]


class TargetFireController:
    """The scan worker owns all Gimbal motion; the camera worker supplies sightings."""

    def __init__(self, robot, inspection, lock, stopped, mode, events,
                 pose_provider=None, target_color="Red", target_shape="All"):
        if target_color not in TARGET_COLORS:
            raise ValueError("Unsupported target color: {}".format(target_color))
        if target_shape not in TARGET_SHAPES:
            raise ValueError("Unsupported target shape: {}".format(target_shape))
        self.robot = robot
        self.inspection = inspection
        self.lock = lock
        self.stopped = stopped
        self.mode = mode
        self.fire_type = fire_type_for_sdk(mode)
        self.events = events
        self.pose_provider = pose_provider
        self.target_color = target_color
        self.target_shape = target_shape
        self.last_move_error = None
        self.fired = []
        # These are initial angle estimates. Fresh images correct the aim below.
        self.horizontal_fov_deg = setting("fire.horizontal_fov_deg")
        self.vertical_fov_deg = setting("fire.vertical_fov_deg")
        self.center_fraction = setting("fire.center_tolerance_fraction")
        self.yaw_bias_deg = setting("fire.yaw_bias_deg")
        self.pitch_bias_deg = setting("fire.pitch_bias_deg")
        self.camera_above_barrel_m = setting("fire.camera_above_barrel_m")
        self.default_target_distance_m = setting("fire.default_target_distance_m")

    def matches_target(self, target):
        return (target["color"] == self.target_color
                and (self.target_shape == "All" or target["shape"] == self.target_shape))

    def _matches_sighting(self, target, sighting):
        return (sighting["color"] == target["color"]
                and (self.target_shape == "All" or sighting["shape"] == target["shape"]))

    def _image_angles(self, center, width, height, yaw, pitch):
        x, y = center
        dx = (x - width / 2) / (width / 2)
        dy = (y - height / 2) / (height / 2)
        yaw += math.degrees(math.atan(dx * math.tan(math.radians(self.horizontal_fov_deg / 2))))
        pitch -= math.degrees(math.atan(dy * math.tan(math.radians(self.vertical_fov_deg / 2))))
        return yaw, pitch

    def observe(self, detections, frame_shape):
        height, width = frame_shape[:2]
        with self.lock:
            if not self.inspection.get("active"):
                return
            yaw = self.inspection["camera_yaw"]
            pitch = self.inspection["camera_pitch"]
            self.inspection["frame_seq"] = self.inspection.get("frame_seq", 0) + 1
            seq = self.inspection["frame_seq"]
            observations = []
            for detection in detections:
                aim_yaw, aim_pitch = self._image_angles(
                    detection["center"], width, height, yaw, pitch
                )
                observations.append({
                    "color": detection["color"], "shape": detection["shape"],
                    "center": detection["center"], "width": width, "height": height,
                    "yaw": aim_yaw, "pitch": aim_pitch,
                })
            self.inspection["observations"] = observations
            if self.inspection.get("phase") != "survey":
                return

            tracks = self.inspection.setdefault("targets", [])
            matched = set()
            for observation in observations:
                candidates = [
                    (abs(track["yaw"] - observation["yaw"])
                     + abs(track["pitch"] - observation["pitch"]), index)
                    for index, track in enumerate(tracks)
                    if index not in matched
                    and track["color"] == observation["color"]
                    and track["shape"] == observation["shape"]
                    and abs(track["yaw"] - observation["yaw"]) < 5
                    and abs(track["pitch"] - observation["pitch"]) < 6
                ]
                if candidates:
                    _, index = min(candidates)
                    track = tracks[index]
                    track["yaw"] = 0.6 * track["yaw"] + 0.4 * observation["yaw"]
                    track["pitch"] = 0.6 * track["pitch"] + 0.4 * observation["pitch"]
                    track["seen"] = track["seen"] + 1 if track["last_seq"] == seq - 1 else 1
                    track["last_seq"] = seq
                    track["confirmed"] = track["confirmed"] or track["seen"] >= 3
                else:
                    index = len(tracks)
                    tracks.append({
                        "color": observation["color"], "shape": observation["shape"],
                        "yaw": observation["yaw"], "pitch": observation["pitch"],
                        "seen": 1, "last_seq": seq, "confirmed": False,
                    })
                matched.add(index)

    def confirm_detection(self, detection):
        """Use the sign mapper's three-frame confirmation for this visible target."""
        with self.lock:
            if not self.inspection.get("active") or self.inspection.get("phase") != "survey":
                return
            observations = self.inspection.get("observations", [])
            matching = [item for item in observations
                        if item["color"] == detection["color"]
                        and item["shape"] == detection["shape"]
                        and item["center"] == detection["center"]]
            if not matching:
                return
            observation = matching[0]
            tracks = self.inspection.setdefault("targets", [])
            candidates = [
                (abs(track["yaw"] - observation["yaw"])
                 + abs(track["pitch"] - observation["pitch"]), track)
                for track in tracks
                if track["color"] == observation["color"]
                and track["shape"] == observation["shape"]
            ]
            if candidates:
                error, track = min(candidates, key=lambda item: item[0])
                if error < 10:
                    track["confirmed"] = True
                    return
            tracks.append({
                "color": observation["color"], "shape": observation["shape"],
                "yaw": observation["yaw"], "pitch": observation["pitch"],
                "seen": 3, "last_seq": self.inspection["frame_seq"],
                "confirmed": True,
            })

    def _already_fired(self, cell, direction, target):
        return any(
            item["cell"] == cell and item["direction"] == direction
            and item["color"] == target["color"] and item["shape"] == target["shape"]
            for item in self.fired
        )

    def _actual_pose(self):
        if self.pose_provider is None:
            return None
        try:
            state = self.pose_provider()
            return float(state.gimbal_yaw), float(state.gimbal_pitch)
        except Exception:
            return None

    def _command_pose(self, yaw, pitch, phase):
        try:
            action = self.robot.gimbal.moveto(
                yaw=yaw, pitch=pitch,
                yaw_speed=setting("gimbal.yaw_speed_dps"),
                pitch_speed=setting("gimbal.pitch_speed_dps"),
            )
            completed = action.wait_for_completed(
                timeout=setting("gimbal.action_timeout_sec"))
            succeeded = completed is not False and getattr(action, "has_succeeded", True)
            state = str(getattr(action, "state", "unknown"))
            reason = str(getattr(action, "failure_reason", ""))
        except Exception as error:
            succeeded, state, reason = False, "exception", str(error)

        actual = self._actual_pose()
        self.events.append({
            "timestamp": time.time(), "type": "target_gimbal_move",
            "phase": phase, "requested_yaw": round(yaw, 2),
            "requested_pitch": round(pitch, 2), "succeeded": bool(succeeded),
            "action_state": state, "reason": reason,
            "actual_yaw": round(actual[0], 2) if actual else None,
            "actual_pitch": round(actual[1], 2) if actual else None,
        })
        if not succeeded:
            self.last_move_error = "{}: {} {}".format(phase, state, reason).strip()
            print("[Target fire] Gimbal {} failed: {} {}".format(phase, state, reason))
        return bool(succeeded)

    def _move(self, yaw, pitch, travel=False):
        yaw = max(-250.0, min(250.0, yaw))
        pitch = max(-20.0, min(setting("fire.max_shot_pitch_deg"), pitch))
        with self.lock:
            self.inspection["active"] = False
            previous_yaw = self.inspection.get("camera_yaw", yaw)
        actual = self._actual_pose()
        if actual:
            previous_yaw = actual[0]
        if travel:
            level_pitch = setting("gimbal.pitch_deg")
            if not self._command_pose(previous_yaw, level_pitch, "lift"):
                return False, yaw, pitch, 0
            if not self._command_pose(yaw, level_pitch, "turn"):
                return False, yaw, pitch, 0
        if not self._command_pose(yaw, pitch, "aim"):
            return False, yaw, pitch, 0
        time.sleep(setting("gimbal.settle_sec"))
        with self.lock:
            self.inspection.update(camera_yaw=yaw, camera_pitch=pitch,
                                   phase="aim", active=True)
            seq = self.inspection.get("frame_seq", 0)
        return True, yaw, pitch, seq

    def _fresh_observation(self, after_seq, target):
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline and not self.stopped.is_set():
            with self.lock:
                seq = self.inspection.get("frame_seq", 0)
                observations = list(self.inspection.get("observations", []))
            if seq > after_seq:
                # The same colored sign can be classified as another shape
                # after it moves from the edge to the middle of the image.
                matches = [item for item in observations
                           if self._matches_sighting(target, item)]
                if matches:
                    return min(matches, key=lambda item:
                               abs(item["yaw"] - target["yaw"])
                               + 0.2 * abs(item["pitch"] - target["pitch"])
                               + (0 if item["shape"] == target["shape"] else 3))
                after_seq = seq
            time.sleep(0.02)
        return None

    def _centered(self, sighting):
        x = sighting["center"][0]
        return abs(x - sighting["width"] / 2) <= sighting["width"] * self.center_fraction

    def _yaw_correction(self, sighting):
        """Convert the horizontal gap between the two crosshairs to a yaw step."""
        x = sighting["center"][0]
        width = sighting["width"]
        offset = (x - width / 2) / (width / 2)
        angle = math.degrees(math.atan(
            offset * math.tan(math.radians(self.horizontal_fov_deg / 2))))
        return max(-10.0, min(10.0, angle * 0.65))

    def _barrel_pitch_offset(self, target):
        distance_mm = target.get("tof_distance_mm")
        distance_m = (distance_mm / 1000.0 if distance_mm is not None
                      else self.default_target_distance_m)
        if distance_m <= 0:
            distance_m = self.default_target_distance_m
        return math.degrees(math.atan2(self.camera_above_barrel_m, distance_m)), distance_m

    def _wait_for_lock(self, after_seq, target):
        """Confirm the same target in two fresh frames without moving the Gimbal."""
        deadline = time.monotonic() + 2.0
        centered_frames = 0
        while time.monotonic() < deadline and not self.stopped.is_set():
            with self.lock:
                seq = self.inspection.get("frame_seq", 0)
                observations = list(self.inspection.get("observations", []))
            if seq <= after_seq:
                time.sleep(0.02)
                continue
            after_seq = seq
            matches = [item for item in observations
                       if self._matches_sighting(target, item)]
            sighting = min(matches, key=lambda item:
                           abs(item["yaw"] - target["yaw"])
                           + 0.2 * abs(item["pitch"] - target["pitch"])
                           + (0 if item["shape"] == target["shape"] else 3)) if matches else None
            if sighting:
                centered_frames = centered_frames + 1 if self._centered(sighting) else 0
            if centered_frames >= 2:
                return True
        return False

    def fire_confirmed(self, cell, direction):
        with self.lock:
            targets = [dict(item) for item in self.inspection.get("targets", [])
                       if item["confirmed"] and self.matches_target(item)]
        for target in sorted(targets, key=lambda item: item["yaw"]):
            if self.stopped.is_set():
                break
            target_direction = target.get("direction", direction)
            if self._already_fired(cell, target_direction, target):
                continue
            yaw, pitch = target["yaw"], setting("fire.scan_pitch_deg")
            fired = False
            fire_command_sent = False
            locked = False
            aim_moves = 0
            aim_error_px = None
            aim_failure = None
            self.last_move_error = None
            fire_command_timestamp = None
            burst_hold_sec = 0.0
            barrel_pitch_offset_deg = 0.0
            target_distance_m = None
            try:
                # Use the camera crosshair and target centre to adjust yaw.
                # Pitch stays at the configured look-down angle.
                for attempt in range(6):
                    if self.stopped.is_set():
                        break
                    moved, yaw, pitch, seq = self._move(
                        yaw, pitch, travel=(attempt == 0))
                    aim_moves += 1
                    if not moved:
                        aim_failure = self.last_move_error or "gimbal_move_failed"
                        break
                    sighting = self._fresh_observation(seq, target)
                    if sighting is None:
                        aim_failure = "target_not_visible_after_move"
                        break
                    aim_error_px = round(sighting["center"][0] - sighting["width"] / 2, 1)
                    if self._centered(sighting):
                        locked = self._wait_for_lock(seq, target)
                        if not locked:
                            aim_failure = "target_not_stable_at_center"
                        break
                    if attempt < 5:
                        yaw += self._yaw_correction(sighting)

                if not locked and aim_failure is None:
                    aim_failure = "yaw_not_centered_after_corrections"

                if locked and not self.stopped.is_set():
                    barrel_pitch_offset_deg, target_distance_m = self._barrel_pitch_offset(target)
                    if (self.yaw_bias_deg or self.pitch_bias_deg
                            or barrel_pitch_offset_deg):
                        moved, yaw, pitch, _ = self._move(
                            yaw + self.yaw_bias_deg,
                            pitch + self.pitch_bias_deg + barrel_pitch_offset_deg
                        )
                    else:
                        moved = True
                    if moved:
                        with self.lock:
                            self.inspection["phase"] = "fire"
                            self.inspection["active"] = False
                        fire_command_sent = True
                        fire_command_timestamp = time.time()
                        fired = self.robot.blaster.fire(
                            fire_type=self.fire_type, times=SHOTS_PER_TARGET
                        ) is True
            except Exception as error:
                print("[Target fire] {} {}: {}".format(target["color"], target["shape"], error))
            finally:
                if fire_command_sent:
                    # SDK fire() only acknowledges the command. Keep the
                    # Gimbal at this pose while the three shots finish.
                    burst_hold_sec = setting("fire.burst_hold_sec")
                    print("[Target fire] Holding Gimbal on {} {} for {:.1f}s after burst"
                          .format(target["color"], target["shape"], burst_hold_sec))
                    time.sleep(burst_hold_sec)
                with self.lock:
                    visible_labels = sorted(set(
                        "{} {}".format(item["color"], item["shape"])
                        for item in self.inspection.get("observations", [])))
                actual = self._actual_pose()
                self.events.append({
                    "timestamp": time.time(), "type": "target_fire",
                    "cell": list(cell), "direction": target_direction,
                    "color": target["color"], "shape": target["shape"],
                    "fire_mode": self.mode, "yaw": round(yaw, 2),
                    "pitch": round(pitch, 2), "locked": locked, "fired": fired,
                    "aim_moves": aim_moves, "aim_error_px": aim_error_px,
                    "aim_failure": aim_failure if not locked else None,
                    "visible_labels": visible_labels,
                    "actual_yaw": round(actual[0], 2) if actual else None,
                    "actual_pitch": round(actual[1], 2) if actual else None,
                    "fire_commands_sent": 1 if fire_command_sent else 0,
                    "shots_requested": SHOTS_PER_TARGET if fire_command_sent else 0,
                    "fire_command_timestamp": fire_command_timestamp,
                    "burst_hold_sec": burst_hold_sec,
                    "target_distance_m": target_distance_m,
                    "barrel_pitch_offset_deg": round(barrel_pitch_offset_deg, 2),
                })
                if fire_command_sent:
                    # An attempted burst completes this target for the current
                    # wall, even when the SDK does not acknowledge the command.
                    self.fired.append(dict(target, cell=cell, direction=target_direction))
                if fired:
                    print("[Target fire] {} {} in cell {}: {} x{} at yaw {:.1f}, pitch {:.1f}"
                          .format(target["color"], target["shape"], cell,
                                  self.mode, SHOTS_PER_TARGET, yaw, pitch))
                elif not locked:
                    print("[Target fire] {} {} not centered in camera; skipping this shot"
                          .format(target["color"], target["shape"]))
                else:
                    print("[Target fire] Could not fire at {} {} in cell {}"
                          .format(target["color"], target["shape"], cell))
        with self.lock:
            self.inspection["active"] = False
            self.inspection["phase"] = "idle"
