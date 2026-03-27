#!/usr/bin/env python3
"""
cv_move_to_spot.py — Vision-guided arm movement to detected incision spots
===========================================================================

PURPOSE:
  Loads the eye-to-hand calibration, detects S1–S4 incision spots via
  RealSense + ArUco, transforms them to xArm base frame, and moves the arm
  to a selected spot on keypress.

PREREQUISITES:
  1. Run cv_aruco_test.py (Phase 1) to verify detection works.
  2. Run cv_calibration.py (Phase 2) to generate T_cam_to_base.npy.
  3. Connect xArm over the network.

USAGE:
  python surgical_test/cv/cv_move_to_spot.py [ARM_IP]

  ARM_IP defaults to 192.168.1.113.

CONTROLS:
  1–4  — move arm to spot S1–S4
  H    — move arm to home position
  ESC/q — quit
"""

import sys
import os
import time

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
    _RS_AVAILABLE = True
except ImportError:
    print("ERROR: pyrealsense2 not installed. Run: pip install pyrealsense2")
    sys.exit(1)

# Add repo root to path for xarm imports
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

try:
    from xarm.wrapper import XArmAPI
except ImportError:
    print("ERROR: xarm package not found. Run from the repo root or install the package.")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

STREAM_WIDTH  = 640
STREAM_HEIGHT = 480
STREAM_FPS    = 30

ARUCO_DICT_ID    = cv2.aruco.DICT_4X4_50
MARKER_START_ID  = 0
MARKER_END_ID    = 1
NUM_SPOTS        = 4

# Arm movement
SPEED            = 50    # mm/s
APPROACH_HEIGHT  = 30    # mm above the detected spot Z (safety clearance)
END_EFFECTOR_PITCH = -90 # degrees — keep end-effector pointing straight down

# Home joint angles (degrees) — same as in voice-movement.py
HOME_JOINTS = [0, -45, 0, 45, 0, 90, 0]

DEFAULT_ARM_IP = "192.168.1.113"

CALIB_PATH = os.path.join(os.path.dirname(__file__), "T_cam_to_base.npy")

# Drawing colours (BGR)
COLOR_LINE   = (0,   0,   255)
COLOR_MARKER = (0,   255, 0  )
COLOR_SPOT   = (255, 100, 0  )
COLOR_LABEL  = (255, 255, 255)


# ---------------------------------------------------------------------------
# Arm helpers
# ---------------------------------------------------------------------------

def connect_arm(ip: str) -> XArmAPI:
    arm = XArmAPI(ip, do_not_open=True)
    arm.connect()
    if arm.error_code != 0:
        arm.clean_error()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.5)
    print(f"[xArm] Connected to {ip}")
    return arm


def go_home(arm: XArmAPI) -> None:
    print("[xArm] Moving to home …")
    arm.set_servo_angle(angle=HOME_JOINTS, speed=30, wait=True)


def move_to_spot(arm: XArmAPI, x_mm: float, y_mm: float, z_mm: float) -> None:
    """
    Move the TCP to (x, y, z) in xArm base frame (mm), approaching from
    APPROACH_HEIGHT mm above.  End-effector points straight down (pitch=-90).
    """
    # Approach above the target first
    approach_z = z_mm + APPROACH_HEIGHT
    print(f"[xArm] Approach ({x_mm:.1f}, {y_mm:.1f}, {approach_z:.1f}) mm …")
    code = arm.set_position(
        x=x_mm, y=y_mm, z=approach_z,
        roll=0, pitch=END_EFFECTOR_PITCH, yaw=0,
        speed=SPEED, wait=True,
    )
    if code not in (0, None):
        print(f"[xArm] WARNING: set_position returned code={code}")
        return

    # Descend to the actual target
    print(f"[xArm] Descend  ({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}) mm …")
    arm.set_position(
        x=x_mm, y=y_mm, z=z_mm,
        roll=0, pitch=END_EFFECTOR_PITCH, yaw=0,
        speed=SPEED // 2, wait=True,
    )


# ---------------------------------------------------------------------------
# RealSense helpers
# ---------------------------------------------------------------------------

def open_realsense():
    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.color, STREAM_WIDTH, STREAM_HEIGHT, rs.format.bgr8, STREAM_FPS)
    config.enable_stream(rs.stream.depth, STREAM_WIDTH, STREAM_HEIGHT, rs.format.z16,  STREAM_FPS)
    profile  = pipeline.start(config)
    align    = rs.align(rs.stream.color)
    intr     = (
        profile.get_stream(rs.stream.color)
               .as_video_stream_profile()
               .get_intrinsics()
    )
    print(f"[RealSense] Started {STREAM_WIDTH}×{STREAM_HEIGHT} @ {STREAM_FPS} fps")
    return pipeline, align, intr


def get_frames(pipeline, align):
    frames      = align.process(pipeline.wait_for_frames())
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    if not color_frame:
        return None, None
    return np.asanyarray(color_frame.get_data()), depth_frame if depth_frame else None


def get_depth_at_pixel(depth_frame, x: int, y: int, kernel: int = 5) -> float:
    if depth_frame is None:
        return 0.0
    half = kernel // 2
    w, h = depth_frame.get_width(), depth_frame.get_height()
    depths = [
        depth_frame.get_distance(x + dx, y + dy)
        for dy in range(-half, half + 1)
        for dx in range(-half, half + 1)
        if 0 <= x + dx < w and 0 <= y + dy < h
    ]
    depths = [d for d in depths if d > 0]
    return float(np.median(depths)) if depths else 0.0


def pixel_to_3d_cam(depth_frame, intr, x: float, y: float) -> np.ndarray | None:
    """Return (X, Y, Z) in metres in camera frame, or None if no depth."""
    d = get_depth_at_pixel(depth_frame, int(x), int(y))
    if d == 0.0:
        return None
    pt = rs.rs2_deproject_pixel_to_point(intr, [x, y], d)
    return np.array(pt)   # metres


# ---------------------------------------------------------------------------
# ArUco helpers
# ---------------------------------------------------------------------------

def build_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    if hasattr(cv2.aruco, "ArucoDetector"):
        params   = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        return detector, dictionary, "new"
    else:
        params = cv2.aruco.DetectorParameters_create()
        return None, dictionary, params


def detect_markers(frame: np.ndarray, detector_bundle) -> dict[int, tuple[float, float]]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detector, dictionary, extra = detector_bundle
    if extra == "new":
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=extra)

    centers = {}
    if ids is None:
        return centers
    for corner_set, mid in zip(corners, ids.flatten()):
        pts = corner_set.reshape(4, 2)
        centers[int(mid)] = (float(pts[:, 0].mean()), float(pts[:, 1].mean()))
    return centers


def compute_spots(pt_a, pt_b, n=NUM_SPOTS):
    ax, ay = pt_a
    bx, by = pt_b
    return [
        (ax + (2*k - 1) / (2*n) * (bx - ax),
         ay + (2*k - 1) / (2*n) * (by - ay))
        for k in range(1, n + 1)
    ]


# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------

def draw_overlay(frame, centers, spots, spots_3d_base, selected_idx=None):
    for mid, (cx, cy) in centers.items():
        pt = (int(cx), int(cy))
        cv2.circle(frame, pt, 8, COLOR_MARKER, -1)
        cv2.putText(frame, f"M{mid}", (pt[0]+10, pt[1]-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_MARKER, 2)

    if MARKER_START_ID in centers and MARKER_END_ID in centers:
        pa = tuple(int(v) for v in centers[MARKER_START_ID])
        pb = tuple(int(v) for v in centers[MARKER_END_ID])
        cv2.line(frame, pa, pb, COLOR_LINE, 2)

        for i, (sx, sy) in enumerate(spots):
            pt    = (int(sx), int(sy))
            label = f"S{i+1}"
            color = (0, 255, 255) if i == selected_idx else COLOR_SPOT
            cv2.circle(frame, pt, 12, color, -1)
            cv2.circle(frame, pt, 12, (255, 255, 255), 2)
            cv2.putText(frame, label, (pt[0]+14, pt[1]+5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, COLOR_LABEL, 2)

            # Show base-frame Z if available
            if spots_3d_base and i < len(spots_3d_base) and spots_3d_base[i] is not None:
                p = spots_3d_base[i]
                info = f"({p[0]:.0f},{p[1]:.0f},{p[2]:.0f})mm"
                cv2.putText(frame, info, (pt[0]+14, pt[1]+22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    n_det = sum(1 for m in (MARKER_START_ID, MARKER_END_ID) if m in centers)
    cv2.putText(frame, f"Markers: {n_det}/2 | 1-4=move H=home ESC=quit",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 2)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def run():
    # --- Load calibration ---
    if not os.path.exists(CALIB_PATH):
        print(f"ERROR: Calibration file not found: {CALIB_PATH}")
        print("Run cv_calibration.py first to generate it.")
        sys.exit(1)

    T_cam_to_base = np.load(CALIB_PATH)
    print(f"[move_to_spot] Loaded T_cam_to_base from {CALIB_PATH}")
    print(np.array2string(T_cam_to_base, precision=3, suppress_small=True))

    # --- Connect arm ---
    ip = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_ARM_IP
    arm = connect_arm(ip)

    # --- Open camera ---
    pipeline, rs_align, intr = open_realsense()
    detector_bundle = build_detector()

    spots_3d_base: list[np.ndarray | None] = []
    spots_pixels:  list[tuple[float, float]] = []
    last_selected: int | None = None

    print("\n[move_to_spot] Live feed started.")
    print("  Press 1–4 to move to that spot, H for home, ESC/q to quit.\n")

    try:
        while True:
            color, depth = get_frames(pipeline, rs_align)
            if color is None:
                continue

            centers = detect_markers(color, detector_bundle)

            spots_pixels = []
            spots_3d_base = []

            if MARKER_START_ID in centers and MARKER_END_ID in centers:
                spots_pixels = compute_spots(
                    centers[MARKER_START_ID], centers[MARKER_END_ID], NUM_SPOTS
                )

                for sx, sy in spots_pixels:
                    pt_cam = pixel_to_3d_cam(depth, intr, sx, sy)
                    if pt_cam is not None:
                        # Homogeneous transform to base frame
                        p_cam_h  = np.append(pt_cam, 1.0)
                        p_base   = (T_cam_to_base @ p_cam_h)[:3] * 1000  # m → mm
                        spots_3d_base.append(p_base)
                    else:
                        spots_3d_base.append(None)

            draw_overlay(color, centers, spots_pixels, spots_3d_base, last_selected)
            cv2.imshow("Move to Spot", color)

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break

            elif key == ord("h"):
                last_selected = None
                go_home(arm)

            elif ord("1") <= key <= ord("4"):
                idx = key - ord("1")   # 0-based
                if idx >= len(spots_3d_base):
                    print(f"[move_to_spot] Spot S{idx+1} not detected yet.")
                    continue
                if spots_3d_base[idx] is None:
                    print(f"[move_to_spot] Spot S{idx+1}: no depth reading, cannot move.")
                    continue

                last_selected = idx
                p = spots_3d_base[idx]
                print(f"[move_to_spot] Moving to S{idx+1}: base=({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) mm")
                move_to_spot(arm, float(p[0]), float(p[1]), float(p[2]))

    finally:
        pipeline.stop()
        arm.disconnect()
        cv2.destroyAllWindows()
        print("[move_to_spot] Done.")


if __name__ == "__main__":
    run()
