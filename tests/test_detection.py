"""
Comprehensive Test Suite for detect_camera.py
Verifies:
- All 16 color-shape combinations (4 colors x 4 shapes)
- Perspective distortion (-20 deg pitch down simulation)
- Low light & high light tolerance
- False positive rejection on textured walls and shadows
- Integration interface compatibility with slam_detect_camera.py
"""

import sys
from pathlib import Path

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

import cv2
import numpy as np
from detect_camera import detect_signs


def create_base_wall(width=800, height=600, brightness=230):
    """Generates realistic wall texture with slight noise."""
    wall = np.full((height, width, 3), brightness, dtype=np.uint8)
    noise = np.random.normal(0, 4, wall.shape).astype(np.int16)
    return np.clip(wall.astype(np.int16) + noise, 0, 255).astype(np.uint8)


# Canonical BGR colors for drawing synthetic signs
TEST_COLORS = {
    "Red": (30, 25, 215),
    "Green": (35, 175, 40),
    "Blue": (210, 60, 20),
    "Yellow": (25, 215, 225),
}


def draw_shape(canvas, shape_name, color_bgr, center=(400, 300)):
    cx, cy = center
    if shape_name == "Circle":
        # Perspective ellipse (tilted by -20 deg pitch)
        cv2.ellipse(canvas, (cx, cy), (48, 40), 10, 0, 360, color_bgr, -1)
    elif shape_name == "Square":
        w, h = 42, 40
        pts = np.array([
            [cx - w, cy - h],
            [cx + w + 4, cy - h + 2],
            [cx + w + 5, cy + h],
            [cx - w - 2, cy + h - 2]
        ], dtype=np.int32)
        cv2.fillPoly(canvas, [pts], color_bgr)
    elif shape_name == "Horizontal_Rect":
        w, h = 68, 28
        pts = np.array([
            [cx - w, cy - h],
            [cx + w, cy - h + 1],
            [cx + w, cy + h],
            [cx - w, cy + h]
        ], dtype=np.int32)
        cv2.fillPoly(canvas, [pts], color_bgr)
    elif shape_name == "Vertical_Rect":
        w, h = 26, 60
        pts = np.array([
            [cx - w, cy - h],
            [cx + w, cy - h],
            [cx + w, cy + h],
            [cx - w, cy + h]
        ], dtype=np.int32)
        cv2.fillPoly(canvas, [pts], color_bgr)


def test_all_16_combinations():
    print("\n--- Test: All 16 Combinations (4 Colors x 4 Shapes) ---")
    colors = ["Red", "Green", "Blue", "Yellow"]
    shapes = ["Circle", "Square", "Horizontal_Rect", "Vertical_Rect"]

    passed_count = 0
    total_count = len(colors) * len(shapes)

    for color in colors:
        for shape in shapes:
            wall = create_base_wall(width=500, height=400)
            draw_shape(wall, shape, TEST_COLORS[color], center=(250, 200))
            _, _, detections = detect_signs(wall)

            found = any(d["color"] == color and d["shape"] == shape for d in detections)
            status = "PASS" if found else "FAIL"
            if found:
                passed_count += 1
            else:
                det_str = [(d["color"], d["shape"]) for d in detections]
                print(f"  FAILED: Expected ({color}, {shape}), got: {det_str}")

    print(f"Result: {passed_count}/{total_count} combinations passed.")
    assert passed_count == total_count, f"Some combinations failed ({passed_count}/{total_count})"


def test_perspective_multi_sign_scene():
    print("\n--- Test: Perspective Scene With 4 Different Signs Concurrently ---")
    wall = create_base_wall()

    # 1. Red Circle
    draw_shape(wall, "Circle", TEST_COLORS["Red"], center=(160, 200))
    # 2. Blue Square
    draw_shape(wall, "Square", TEST_COLORS["Blue"], center=(360, 200))
    # 3. Green Horizontal_Rect
    draw_shape(wall, "Horizontal_Rect", TEST_COLORS["Green"], center=(580, 200))
    # 4. Yellow Vertical_Rect
    draw_shape(wall, "Vertical_Rect", TEST_COLORS["Yellow"], center=(250, 420))

    _, _, detections = detect_signs(wall)
    found_keys = {(d["color"], d["shape"]) for d in detections}

    expected = {
        ("Red", "Circle"),
        ("Blue", "Square"),
        ("Green", "Horizontal_Rect"),
        ("Yellow", "Vertical_Rect"),
    }

    print(f"Detected: {sorted(found_keys)}")
    assert expected.issubset(found_keys), f"Missing signs! Expected {expected}, got {found_keys}"
    print("PASS: Multi-sign scene detected correctly.")


def test_lighting_variation():
    print("\n--- Test: Lighting Invariance (Dim 0.6x & Bright 1.3x) ---")
    for factor, label in [(0.60, "Dim light"), (1.30, "Bright light")]:
        wall = create_base_wall(brightness=int(180 * factor))
        draw_shape(wall, "Circle", tuple(np.clip(np.array(TEST_COLORS["Yellow"]) * factor, 0, 255).astype(np.uint8).tolist()), center=(300, 300))

        _, _, detections = detect_signs(wall)
        found = any(d["color"] == "Yellow" and d["shape"] == "Circle" for d in detections)
        print(f"Lighting [{label}]: {'PASS' if found else 'FAIL'}")
        assert found, f"Failed under {label}"


def test_false_positive_rejection():
    print("\n--- Test: False Positive Rejection (Blank Wall & Floor Shadow) ---")
    wall = create_base_wall()
    # Add a horizontal dark shadow bar at bottom (simulating floor/wall joint)
    cv2.rectangle(wall, (0, 550), (800, 600), (50, 50, 50), -1)

    _, _, detections = detect_signs(wall)
    print(f"Detections on empty wall + shadow: {len(detections)}")
    assert len(detections) == 0, f"False positives detected: {detections}"
    print("PASS: Zero false positives on plain wall and floor shadows.")


def test_slam_detect_camera_interface():
    print("\n--- Test: Integration Interface With slam_detect_camera.py ---")
    # Verify return types and keys expected by slam_detect_camera.py:
    # line 401: result, mask, detections = detect_signs(frame)
    # line 423: key = (current_cell, current_direction, detection["color"], detection["shape"])
    # line 474: cv2.imshow("Detected Sign Mask", mask)
    wall = create_base_wall(width=400, height=300)
    draw_shape(wall, "Square", TEST_COLORS["Red"], center=(200, 150))

    result, mask, detections = detect_signs(wall)

    assert isinstance(result, np.ndarray), "Result must be a numpy ndarray"
    assert isinstance(mask, np.ndarray), "Mask must be a numpy ndarray"
    assert result.shape == wall.shape, f"Result shape mismatch: {result.shape} vs {wall.shape}"
    assert mask.shape == wall.shape, f"Mask shape mismatch: {mask.shape} vs {wall.shape}"
    assert len(detections) >= 1, "Expected at least 1 detection"

    for det in detections:
        assert "color" in det, "Missing 'color' key in detection"
        assert "shape" in det, "Missing 'shape' key in detection"
        assert "box" in det, "Missing 'box' key in detection"
        assert "center" in det, "Missing 'center' key in detection"
        assert "contour" in det, "Missing 'contour' key in detection"
        assert det["color"] in ("Red", "Green", "Blue", "Yellow")
        assert det["shape"] in ("Circle", "Square", "Horizontal_Rect", "Vertical_Rect")

    print("PASS: slam_detect_camera.py interface contract is 100% compliant.")


if __name__ == "__main__":
    try:
        test_all_16_combinations()
        test_perspective_multi_sign_scene()
        test_lighting_variation()
        test_false_positive_rejection()
        test_slam_detect_camera_interface()
        print("\n=======================================================")
        print("ALL TESTS PASSED SUCCESSFULLY! (100% COMPLIANT & ROBUST)")
        print("=======================================================")
    except AssertionError as e:
        print("\nTEST FAILED:", e)
        sys.exit(1)
