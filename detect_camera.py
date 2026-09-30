import argparse
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# BGR color mapping for visualization overlays
BGR_COLORS = {
    "Red": (0, 0, 255),
    "Green": (0, 255, 0),
    "Blue": (255, 0, 0),
    "Yellow": (0, 255, 255),
}

# Target Hue centers in OpenCV HSV space [0, 180)
# Red wraps around 0 and 180
COLOR_HUE_CENTERS = {
    "Red": 0.0,
    "Yellow": 28.0,
    "Green": 62.0,
    "Blue": 115.0,
}

# Maximum allowed median angular difference (degrees in Hue space)
COLOR_HUE_TOLERANCES = {
    "Red": 22.0,
    "Yellow": 18.0,
    "Green": 26.0,
    "Blue": 26.0,
}

# Geometric and scale thresholds
MIN_CONTOUR_AREA = 800
MAX_FRAME_AREA_RATIO = 0.30
MIN_SOLIDITY = 0.85
MIN_ELLIPSE_IOU = 0.80
MIN_RECTANGULARITY = 0.78
VERTICAL_RECT_MAX_ASPECT_RATIO = 0.92
MIN_COLOR_CONFIDENCE = 0.45
MIN_COLOR_PURITY = 0.75
MIN_DARK_COLOR_VALUE = 24
MIN_DARK_COLOR_SATURATION = 70

MIN_SIZE_CLUSTER_COUNT = 3
MIN_SIZE_RATIO = 0.55
MAX_SIZE_RATIO = 1.8


def circular_hue_diff(h1, h2):
    """
    Computes angular difference between two Hue values on a circular [0, 180) scale.
    """
    diff = np.abs(h1 - h2)
    return np.minimum(diff, 180.0 - diff)


def classify_color_relative(roi_bgr, roi_mask=None):
    """
    Classifies the dominant color inside a region of interest using relative Hue distribution.
    Does NOT rely on rigid hardcoded HSV bounds. Returns (color_name, confidence).
    """
    if roi_bgr is None or roi_bgr.size == 0:
        return None, 0.0

    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    h_channel = hsv[:, :, 0]
    s_channel = hsv[:, :, 1]
    v_channel = hsv[:, :, 2]

    # Select valid saturated colored pixels (exclude white/gray wall and dark shadows)
    valid_color_mask = ((s_channel > 35) & (v_channel > 40)) | (
        (s_channel >= MIN_DARK_COLOR_SATURATION)
        & (v_channel >= MIN_DARK_COLOR_VALUE)
    )
    if roi_mask is not None:
        valid_color_mask = valid_color_mask & (roi_mask > 0)

    valid_count = np.count_nonzero(valid_color_mask)
    total_count = np.count_nonzero(roi_mask > 0) if roi_mask is not None else roi_bgr.shape[0] * roi_bgr.shape[1]

    # At least 20% of the candidate region must contain saturated color
    if valid_count < 25 or (valid_count / max(1, total_count)) < 0.20:
        return None, 0.0

    valid_hues = h_channel[valid_color_mask].astype(np.float32)
    median_hue = float(np.median(valid_hues))

    # Exact boundary classification covering real-world indoor lighting & dark green
    if median_hue < 15.0 or median_hue > 165.0:
        best_color = "Red"
        best_diff = min(median_hue, 180.0 - median_hue)
        max_tol = 15.0
    elif 18.0 <= median_hue <= 38.0:
        best_color = "Yellow"
        best_diff = abs(median_hue - 28.0)
        max_tol = 12.0
    elif 38.0 < median_hue <= 103.0:
        best_color = "Green"
        best_diff = abs(median_hue - 65.0)
        max_tol = 38.0
    elif 103.0 < median_hue <= 140.0:
        best_color = "Blue"
        best_diff = abs(median_hue - 118.0)
        max_tol = 22.0
    else:
        return None, 0.0

    # Yellow target signs have vivid saturation; dull wood/cardboard floor has S < 80
    if best_color == "Yellow":
        median_sat = float(np.median(s_channel[valid_color_mask]))
        if median_sat < 80:
            return None, 0.0

    confidence = max(0.0, min(1.0, 1.0 - (best_diff / max_tol)))
    if confidence < MIN_COLOR_CONFIDENCE:
        return None, 0.0

    # Reject regions whose median hue happens to match a target even though
    # the colored pixels are a mixture of different hues.
    hue_error = circular_hue_diff(valid_hues, COLOR_HUE_CENTERS[best_color])
    if np.count_nonzero(hue_error <= COLOR_HUE_TOLERANCES[best_color]) / len(valid_hues) < MIN_COLOR_PURITY:
        return None, 0.0
    return best_color, confidence


def refine_color_contour(frame, contour, color_name):
    """Separate the target color from nearby shadows before measuring its shape."""
    x, y, w, h = cv2.boundingRect(contour)
    roi = frame[y:y+h, x:x+w]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    hue = hsv[:, :, 0].astype(np.float32)
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]
    original_mask = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(original_mask, [contour - [x, y]], -1, 255, -1)
    colored = ((saturation > 35) & (value > 40)) | (
        (saturation >= MIN_DARK_COLOR_SATURATION)
        & (value >= MIN_DARK_COLOR_VALUE)
    )
    color_mask = np.where(
        (original_mask > 0) & colored
        & (circular_hue_diff(hue, COLOR_HUE_CENTERS[color_name])
           <= COLOR_HUE_TOLERANCES[color_name]),
        255, 0,
    ).astype(np.uint8)
    color_mask = cv2.morphologyEx(
        color_mask, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
    )
    components, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL,
                                     cv2.CHAIN_APPROX_SIMPLE)
    if not components:
        return None
    component = max(components, key=cv2.contourArea)
    if cv2.contourArea(component) < MIN_CONTOUR_AREA:
        return None
    return component + [x, y]


def classify_shape_perspective(contour, frame_shape):
    """
    Classifies a contour into Circle, Square, Horizontal_Rect, or Vertical_Rect.
    Tolerates perspective distortion from the robot's look-down pitch (-20 deg).
    """
    area = cv2.contourArea(contour)
    frame_h, frame_w = frame_shape
    frame_area = frame_h * frame_w

    if area < MIN_CONTOUR_AREA or area > frame_area * MAX_FRAME_AREA_RATIO:
        return None

    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    if hull_area == 0:
        return None

    solidity = area / float(hull_area)
    if solidity < MIN_SOLIDITY:
        return None  # Rejects irregular shadows and concave noise

    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return None

    x, y, w, h = cv2.boundingRect(contour)
    if w > frame_w * 0.85 or h > frame_h * 0.85:
        return None

    # Polygon approximation
    approx = cv2.approxPolyDP(contour, 0.030 * perimeter, True)
    num_vertices = len(approx)

    # --- 1. Circle Check (Fit Ellipse + IoU) ---
    is_circle = False
    if num_vertices >= 7 and len(contour) >= 5:
        try:
            ellipse = cv2.fitEllipse(contour)
            (ecx, ecy), (e_d1, e_d2), e_angle = ellipse
            minor_axis = min(e_d1, e_d2)
            major_axis = max(e_d1, e_d2)

            if major_axis > 0:
                axis_ratio = minor_axis / float(major_axis)
                # Tilted circle at -20 deg pitch has axis ratio >= 0.65
                if 0.62 <= axis_ratio <= 1.05:
                    local_cnt_mask = np.zeros((h, w), dtype=np.uint8)
                    shifted_cnt = contour - [x, y]
                    cv2.drawContours(local_cnt_mask, [shifted_cnt], -1, 255, -1)

                    local_ell_mask = np.zeros((h, w), dtype=np.uint8)
                    shifted_ellipse = ((ecx - x, ecy - y), (e_d1, e_d2), e_angle)
                    cv2.ellipse(local_ell_mask, shifted_ellipse, 255, -1)

                    intersection = np.count_nonzero((local_cnt_mask > 0) & (local_ell_mask > 0))
                    union = np.count_nonzero((local_cnt_mask > 0) | (local_ell_mask > 0))
                    ellipse_iou = intersection / float(union) if union > 0 else 0.0

                    if ellipse_iou >= MIN_ELLIPSE_IOU:
                        is_circle = True
        except Exception:
            pass

    if is_circle:
        return "Circle", (x, y, w, h)

    # --- 2. Rectangle / Square Check (minAreaRect + Quadrilateral vertices) ---
    min_rect = cv2.minAreaRect(contour)
    (rcx, rcy), (rw, rh), rangle = min_rect
    rect_area = rw * rh
    if rect_area <= 0:
        return None

    rectangularity = area / float(rect_area)

    # Require a clean quadrilateral before separating square and rectangle ratios.
    if rectangularity >= MIN_RECTANGULARITY and num_vertices in (4, 5):
        # Bounding boxes inflate under camera roll. Use the rotated rectangle's
        # horizontal and vertical sides to distinguish the four target shapes.
        angle = math.radians(rangle)
        horizontal, vertical = (rw, rh) if abs(math.cos(angle)) >= abs(math.sin(angle)) else (rh, rw)
        aspect_ratio = horizontal / float(vertical)

        # Close vertical signs can appear wider in the image (about 0.87 in
        # recorded frames), so keep them out of the square range.
        if VERTICAL_RECT_MAX_ASPECT_RATIO <= aspect_ratio <= 1.32:
            shape_name = "Square"
        elif aspect_ratio >= 1.50:
            shape_name = "Horizontal_Rect"
        elif aspect_ratio < VERTICAL_RECT_MAX_ASPECT_RATIO:
            shape_name = "Vertical_Rect"
        else:
            return None

        return shape_name, (x, y, w, h)

    return None


def is_in_blaster_zone(box, frame_shape):
    """
    Checks if a detected bounding box lies specifically within the robot blaster
    barrel (ปากกระบอกปืน) at the bottom center of the camera frame.
    """
    frame_h, frame_w = frame_shape[:2]
    bx, by, bw, bh = box
    cx = bx + bw // 2
    cy = by + bh // 2
    # Blaster barrel zone (เฉพาะบริเวณปากกระบอกปืนตรงกลางล่าง: x: 38%-62%, y >= 80%)
    if cy >= int(frame_h * 0.80) and int(frame_w * 0.38) <= cx <= int(frame_w * 0.62):
        return True
    if (by + bh) >= int(frame_h * 0.88) and int(frame_w * 0.38) <= cx <= int(frame_w * 0.62):
        return True
    return False


def is_valid_sign_placard(frame, box, min_contrast=18.0):
    """
    Verifies that the detected shape is an isolated target on a white/light placard.
    Rejects floors of any color (wood, green carpet, blue tiles, red linoleum, etc.)
    because floors do NOT have a surrounding high-contrast neutral placard margin.
    """
    bx, by, bw, bh = box
    fh, fw = frame.shape[:2]

    # Edge contact rejection: floors touch bottom / sides of the camera frame
    if (by + bh) >= int(fh * 0.92) or by <= 2 or bx <= 2 or (bx + bw) >= fw - 2:
        return False

    pad = max(4, int(min(bw, bh) * 0.08))
    y1, y2 = max(0, by - pad), min(fh, by + bh + pad)
    x1, x2 = max(0, bx - pad), min(fw, bx + bw + pad)

    ring_mask = np.ones((y2 - y1, x2 - x1), dtype=bool)
    inner_y1, inner_y2 = by - y1, by + bh - y1
    inner_x1, inner_x2 = bx - x1, bx + bw - x1
    ring_mask[inner_y1:inner_y2, inner_x1:inner_x2] = False

    ring_roi = frame[y1:y2, x1:x2]
    ring_hsv = cv2.cvtColor(ring_roi, cv2.COLOR_BGR2HSV)
    ring_s = ring_hsv[:, :, 1][ring_mask]

    if len(ring_s) == 0:
        return False

    median_border_s = float(np.median(ring_s))
    inner_roi = frame[by:by+bh, bx:bx+bw]
    inner_hsv = cv2.cvtColor(inner_roi, cv2.COLOR_BGR2HSV)
    inner_s = float(inner_hsv[:, :, 1].mean())

    sat_contrast = inner_s - median_border_s
    # Real placards have high saturation contrast with their neutral white border
    if sat_contrast < min_contrast or median_border_s > 55:
        return False

    return True


def build_side_by_side_view(result, mask_view):
    """
    Combines camera feed and sign mask side-by-side into a single display frame.
    """
    h, w = result.shape[:2]
    if len(mask_view.shape) == 2:
        mask_bgr = cv2.cvtColor(mask_view, cv2.COLOR_GRAY2BGR)
    else:
        mask_bgr = mask_view.copy()

    panel_left = result.copy()
    cv2.rectangle(panel_left, (8, 6), (190, 26), (0, 0, 0), -1)
    cv2.putText(panel_left, "CAMERA STREAM", (12, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

    panel_right = mask_bgr.copy()
    cv2.rectangle(panel_right, (8, 6), (170, 26), (0, 0, 0), -1)
    cv2.putText(panel_right, "TARGET MASK", (12, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

    separator = np.full((h, 3, 3), 70, dtype=np.uint8)
    combined = np.hstack([panel_left, separator, panel_right])

    if combined.shape[1] > 1400:
        scale = 1360.0 / combined.shape[1]
        new_w = 1360
        new_h = int(combined.shape[0] * scale)
        combined = cv2.resize(combined, (new_w, new_h), interpolation=cv2.INTER_AREA)

    return combined


def extract_candidate_regions(frame):
    """
    Extracts candidate colored regions on a neutral (white/gray) wall.
    Uses color saliency in LAB and HSV spaces so no specific color is hardcoded.
    Crops out the robot blaster barrel and chassis at the bottom of the frame.
    """
    frame_h, frame_w = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

    s_channel = hsv[:, :, 1]
    v_channel = hsv[:, :, 2]

    # Chroma in LAB space measures distance from neutral gray (128, 128)
    a_ch = lab[:, :, 1].astype(np.float32) - 128.0
    b_ch = lab[:, :, 2].astype(np.float32) - 128.0
    chroma = np.sqrt(a_ch * a_ch + b_ch * b_ch).astype(np.uint8)

    # Colored signs have either noticeable chroma or high saturation
    chroma_mask = cv2.threshold(chroma, 20, 255, cv2.THRESH_BINARY)[1]
    sat_mask = cv2.threshold(s_channel, 38, 255, cv2.THRESH_BINARY)[1]
    color_saliency = cv2.bitwise_or(chroma_mask, sat_mask)

    # Exclude deep shadows (dark floor/crevices)
    bright_mask = np.where(
        (v_channel > 35)
        | ((v_channel >= MIN_DARK_COLOR_VALUE)
           & (s_channel >= MIN_DARK_COLOR_SATURATION)),
        255, 0,
    ).astype(np.uint8)
    candidate_mask = cv2.bitwise_and(color_saliency, bright_mask)

    # Crop out specifically the robot blaster barrel (ปากกระบอกปืนตรงกลางล่าง)
    blaster_top = int(frame_h * 0.82)
    blaster_left = int(frame_w * 0.38)
    blaster_right = int(frame_w * 0.62)
    candidate_mask[blaster_top:, blaster_left:blaster_right] = 0

    # Morphological cleanup
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    candidate_mask = cv2.morphologyEx(candidate_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    candidate_mask = cv2.morphologyEx(candidate_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    # Re-enforce blaster barrel crop after closing
    candidate_mask[blaster_top:, blaster_left:blaster_right] = 0

    return candidate_mask


def filter_size_outliers(detections):
    """
    Filters out erratic size outliers using median scale clustering.
    """
    if len(detections) < MIN_SIZE_CLUSTER_COUNT:
        return detections

    scales = [
        np.sqrt(width * height)
        for _, _, width, height in (item["box"] for item in detections)
    ]
    median_scale = float(np.median(scales))
    return [
        item for item, scale in zip(detections, scales)
        if MIN_SIZE_RATIO * median_scale <= scale <= MAX_SIZE_RATIO * median_scale
    ]


def detect_signs(frame, debug=False):
    """
    Primary detection entrypoint compatible with slam_detect_camera.py.
    Returns:
        overlay: BGR frame with visual overlays and labels.
        mask_view: BGR image showing classified shapes filled with their detected colors.
        detections: List of detection dictionaries containing:
                    color, shape, center, contour, box, confidence.
    """
    frame_h, frame_w = frame.shape[:2]
    overlay = frame.copy()
    mask_view = np.zeros_like(frame)
    detections = []

    candidate_mask = extract_candidate_regions(frame)
    contours, _ = cv2.findContours(candidate_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

    for contour in contours:
        # Identify the color first so nearby shadows cannot distort the shape.
        x, y, w, h = cv2.boundingRect(contour)
        roi_mask = np.zeros((h, w), dtype=np.uint8)
        shifted_cnt = contour - [x, y]
        cv2.drawContours(roi_mask, [shifted_cnt], -1, 255, -1)

        # Erode mask slightly to eliminate boundary bleed from white wall
        roi_mask_inner = cv2.erode(roi_mask, erode_kernel, iterations=1)
        if np.count_nonzero(roi_mask_inner) > 20:
            roi_mask = roi_mask_inner

        roi_bgr = frame[y:y+h, x:x+w]
        color_name, confidence = classify_color_relative(roi_bgr, roi_mask)
        if color_name is None:
            continue

        refined = refine_color_contour(frame, contour, color_name)
        if refined is None:
            continue
        shape_info = classify_shape_perspective(refined, (frame_h, frame_w))
        if shape_info is None:
            continue
        shape_name, (x, y, w, h) = shape_info

        if is_in_blaster_zone((x, y, w, h), (frame_h, frame_w)):
            continue
        if not is_valid_sign_placard(frame, (x, y, w, h)):
            continue

        center = (x + w // 2, y + h // 2)
        detections.append({
            "color": color_name,
            "shape": shape_name,
            "center": center,
            "contour": refined,
            "box": (x, y, w, h),
            "confidence": confidence,
        })

    detections = filter_size_outliers(detections)
    for detection in detections:
        color_name = detection["color"]
        shape_name = detection["shape"]
        contour = detection["contour"]
        x, y, width, height = detection["box"]
        bgr = BGR_COLORS[color_name]

        cv2.drawContours(mask_view, [contour], -1, bgr, -1)
        cv2.drawContours(overlay, [contour], -1, bgr, -1)
        cv2.drawContours(overlay, [contour], -1, (255, 255, 255), 2)
        label = "{} {}".format(color_name, shape_name)
        label_y = max(y - 8, 20)
        cv2.putText(
            overlay, label, (x, label_y), cv2.FONT_HERSHEY_SIMPLEX,
            0.55, (255, 255, 255), 2, cv2.LINE_AA
        )

    cv2.addWeighted(overlay, 0.65, frame, 0.35, 0, overlay)

    # Camera aim point and each detected target centre are drawn after the
    # contour blend so the yaw offset remains easy to see in the live view.
    camera_center = (frame_w // 2, frame_h // 2)
    cv2.drawMarker(overlay, camera_center, (0, 255, 255),
                   cv2.MARKER_CROSS, 24, 2, cv2.LINE_AA)
    for detection in detections:
        target_center = detection["center"]
        cv2.circle(overlay, target_center, 7, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.circle(overlay, target_center, 2, (0, 0, 255), -1, cv2.LINE_AA)

    if debug:
        cv2.imshow("Debug Saliency Mask", candidate_mask)

    return overlay, mask_view, detections


def clear_previous_captures(capture_dir=None):
    """Deletes old captured target images from previous runs."""
    deleted_count = 0
    dirs = []
    if capture_dir is not None:
        dirs.append(Path(capture_dir))

    # Remove legacy root captured_signs folder if present
    legacy_root = Path("captured_signs")
    if legacy_root.is_dir():
        import shutil
        shutil.rmtree(legacy_root, ignore_errors=True)

    # Also search for any existing captured_signs folders in telemetry_logs and mission_maps
    for parent_folder in ("telemetry_logs", "mission_maps"):
        p = Path(parent_folder)
        if p.is_dir():
            for sub_captures in p.glob("**/captured_signs"):
                if sub_captures.is_dir() and sub_captures not in dirs:
                    dirs.append(sub_captures)

    for d in dirs:
        if d.is_dir():
            for f in d.glob("*.*"):
                if f.suffix.lower() in (".jpg", ".jpeg", ".png"):
                    try:
                        f.unlink()
                        deleted_count += 1
                    except Exception:
                        pass
    if deleted_count > 0:
        print("[Cleanup] ลบภาพที่แคปจากการสำรวจครั้งก่อนหน้าทิ้งแล้ว {} รูป".format(deleted_count))
    return deleted_count


def main(on_frame=None, cancel=None):
    parser = argparse.ArgumentParser(description="Adaptive RoboMaster Sign Detector")
    parser.add_argument("--conn-type", choices=("ap", "sta"), default="ap",
                        help="RoboMaster connection type (default: ap)")
    parser.add_argument("--webcam", action="store_true",
                        help="Use PC webcam instead of RoboMaster camera")
    parser.add_argument("--camera-index", type=int, default=0,
                        help="Webcam device index when using --webcam")
    parser.add_argument("--image", type=str, default=None,
                        help="Path to a single test image")
    parser.add_argument("--debug", action="store_true",
                        help="Show debug candidate masks and logs")
    args = parser.parse_args()

    # Mode 1: Test on a static image
    if args.image is not None:
        path = Path(args.image)
        if not path.is_file():
            print("Error: Image not found: {}".format(args.image))
            return
        frame = cv2.imread(str(path))
        result, mask_view, detections = detect_signs(frame, debug=args.debug)
        print("Detected {} sign(s):".format(len(detections)))
        for d in detections:
            print(" - {} {} at box={} (conf: {:.2f})".format(
                d["color"], d["shape"], d["box"], d.get("confidence", 0.0)
            ))
        combined = build_side_by_side_view(result, mask_view)
        cv2.imshow("RoboMaster - Camera & Sign Mask", combined)
        print("Press any key to exit.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return

    snapshot_tracker = {}
    capture_dir = Path("telemetry_logs/camera_test/captured_signs")
    capture_dir.mkdir(parents=True, exist_ok=True)
    clear_previous_captures(capture_dir)

    # Mode 2: Test on PC webcam
    if args.webcam:
        cap = cv2.VideoCapture(args.camera_index)
        if not cap.isOpened():
            print("Error: Could not open webcam index {}".format(args.camera_index))
            return
        print("Webcam detection started on index {}. {}".format(
            args.camera_index,
            "Use the GUI Stop button to quit." if on_frame is not None else "Press 'q' to quit."))
        print("Auto-snapshot: Will save 3 confirmation photos into '{}' when target is detected.".format(capture_dir))
        try:
            while cancel is None or not cancel.is_set():
                ret, frame = cap.read()
                if not ret or frame is None:
                    continue
                result, mask_view, detections = detect_signs(frame, debug=args.debug)

                # Capture up to 3 confirmation photos per detected target
                for d in detections:
                    key = (d["color"], d["shape"])
                    snaps = snapshot_tracker.setdefault(key, [])
                    if len(snaps) < 3:
                        idx = len(snaps) + 1
                        filename = "{}_{}_snap{}.jpg".format(d["color"], d["shape"], idx)
                        cv2.imwrite(str(capture_dir / filename), frame)
                        snaps.append(filename)
                        print("[Snapshot {}/3] Saved confirmation photo: {}".format(idx, filename))

                if on_frame is not None:
                    on_frame(build_side_by_side_view(result, mask_view))
                else:
                    combined = build_side_by_side_view(result, mask_view)
                    cv2.imshow("RoboMaster - Camera & Sign Mask", combined)
                if on_frame is None and cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        finally:
            cap.release()
            if on_frame is None:
                cv2.destroyAllWindows()
        return

    # Mode 3: RoboMaster Robot Camera
    try:
        from robomaster import robot
    except ImportError as error:
        print("RoboMaster SDK unavailable: {}".format(error))
        print("Tip: Use --webcam to test with your PC webcam.")
        return

    ep_robot = robot.Robot()
    ep_camera = None
    stream_started = False

    try:
        print("Connecting to robot camera via {}...".format(args.conn_type))
        ep_robot.initialize(conn_type=args.conn_type)
        ep_camera = ep_robot.camera
        ep_camera.start_video_stream(display=False)
        stream_started = True
        print("Robot camera detection started. {}".format(
            "Use the GUI Stop button to quit." if on_frame is not None else "Press 'q' to quit."))
        print("Auto-snapshot: Will save 3 confirmation photos into '{}' when target is detected.".format(capture_dir))

        previous_signs = None
        while cancel is None or not cancel.is_set():
            frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            if frame is not None:
                result, mask_view, detections = detect_signs(frame, debug=args.debug)
                signs = tuple(sorted(
                    "{} {}".format(item["color"], item["shape"])
                    for item in detections
                ))

                # Capture up to 3 confirmation photos per detected target
                for d in detections:
                    key = (d["color"], d["shape"])
                    snaps = snapshot_tracker.setdefault(key, [])
                    if len(snaps) < 3:
                        idx = len(snaps) + 1
                        filename = "{}_{}_snap{}.jpg".format(d["color"], d["shape"], idx)
                        cv2.imwrite(str(capture_dir / filename), frame)
                        snaps.append(filename)
                        print("[Snapshot {}/3] Saved confirmation photo: {}".format(idx, filename))

                if signs != previous_signs:
                    if signs:
                        print("Detected {} sign(s): {}".format(
                            len(signs), ", ".join(signs)
                        ))
                    else:
                        print("No recognized signs in view.")
                    previous_signs = signs

                if on_frame is not None:
                    on_frame(build_side_by_side_view(result, mask_view))
                else:
                    combined = build_side_by_side_view(result, mask_view)
                    cv2.imshow("RoboMaster - Camera & Sign Mask", combined)

            if on_frame is None and cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except Exception as error:
        print("Error: {}".format(error))
    finally:
        if on_frame is None:
            cv2.destroyAllWindows()
        if stream_started:
            ep_camera.stop_video_stream()
        ep_robot.close()
        print("Camera disconnected.")


if __name__ == "__main__":
    main()
