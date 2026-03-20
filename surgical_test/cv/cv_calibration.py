#!/usr/bin/env python3
"""
cv_calibration.py — Eye-to-hand calibration for the xArm7 + RealSense D435i
=============================================================================

PURPOSE:
  Finds T_cam_to_base — the 4×4 rigid transform from camera frame to xArm
  base frame — using OpenCV's calibrateHandEye (Tsai method).

SETUP:
  1. Print an ArUco marker (ID 5, DICT_4X4_50) at ~8 cm physical size.
  2. Attach the marker flat and firmly to the xArm end-effector (gripper face).
  3. Keep the RealSense camera fixed on its tripod, pointed at the workspace.
  4. Connect the xArm over the network.

USAGE:
  python surgical_test/cv/cv_calibration.py [ARM_IP]

  ARM_IP defaults to 192.168.1.113 (or set in example/wrapper/robot.conf).

CONTROLS (during collection):
  SPACE — capture current pose (move arm first, then press SPACE)
  C     — compute calibration once ≥10 poses are collected
  ESC/q — quit without saving

OUTPUT:
  surgical_test/cv/T_cam_to_base.npy  — 4×4 float64 transform matrix
"""

import sys
import os
import time
import math

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

# ArUco marker attached to end-effector
CALIBRATION_MARKER_ID   = 5
CALIBRATION_MARKER_SIZE = 0.08   # physical side length in metres (~8 cm)
ARUCO_DICT_ID           = cv2.aruco.DICT_4X4_50

MIN_POSES = 10   # minimum captures before calibration is allowed

OUTPUT_PATH = os.path.join(os.path.dirname(__file__), "T_cam_to_base.npy")

DEFAULT_ARM_IP = "192.168.1.113"


# ---------------------------------------------------------------------------
# xArm helpers
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


def get_gripper_to_base(arm: XArmAPI) -> np.ndarray | None:
    """
    Read current TCP pose (x, y, z, roll, pitch, yaw) from the arm and
    return the 4×4 homogeneous transform T_gripper_to_base.

    xArm returns angles in degrees by default.
    """
    code, pos = arm.get_position(is_radian=False)
    if code != 0 or pos is None:
        print(f"[xArm] get_position failed (code={code})")
        return None

    x, y, z, roll_deg, pitch_deg, yaw_deg = pos[:6]

    # Convert to radians for matrix construction
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    yw = math.radians(yaw_deg)

    # Rotation: ZYX intrinsic (yaw → pitch → roll)
    Rz = np.array([
        [math.cos(yw), -math.sin(yw), 0],
        [math.sin(yw),  math.cos(yw), 0],
        [0,             0,            1],
    ])
    Ry = np.array([
        [ math.cos(p), 0, math.sin(p)],
        [ 0,           1, 0          ],
        [-math.sin(p), 0, math.cos(p)],
    ])
    Rx = np.array([
        [1, 0,           0          ],
        [0, math.cos(r), -math.sin(r)],
        [0, math.sin(r),  math.cos(r)],
    ])
    R = Rz @ Ry @ Rx

    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = [x / 1000, y / 1000, z / 1000]  # mm → m
    return T


# ---------------------------------------------------------------------------
# RealSense helpers
# ---------------------------------------------------------------------------

def open_realsense():
    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.color, STREAM_WIDTH, STREAM_HEIGHT, rs.format.bgr8, STREAM_FPS)
    profile  = pipeline.start(config)
    intr     = (
        profile.get_stream(rs.stream.color)
               .as_video_stream_profile()
               .get_intrinsics()
    )
    print(f"[RealSense] Started {STREAM_WIDTH}×{STREAM_HEIGHT} @ {STREAM_FPS} fps")
    return pipeline, intr


def get_color_frame(pipeline) -> np.ndarray | None:
    frames = pipeline.wait_for_frames()
    cf = frames.get_color_frame()
    return np.asanyarray(cf.get_data()) if cf else None


# ---------------------------------------------------------------------------
# ArUco pose estimation
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


def detect_marker_pose(frame: np.ndarray, detector_bundle, intrinsics):
    """
    Detect the calibration marker (ID=CALIBRATION_MARKER_ID) and estimate
    its pose via solvePnP.

    Returns (rvec, tvec) in camera frame, or (None, None) if not found.
    rvec: rotation vector (Rodrigues), tvec: translation in metres.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detector, dictionary, extra = detector_bundle

    if extra == "new":
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=extra)

    if ids is None:
        return None, None

    for corner_set, mid in zip(corners, ids.flatten()):
        if int(mid) != CALIBRATION_MARKER_ID:
            continue

        # Build camera matrix from RealSense intrinsics
        cam_matrix = np.array([
            [intrinsics.fx,  0,             intrinsics.ppx],
            [0,              intrinsics.fy, intrinsics.ppy],
            [0,              0,             1             ],
        ], dtype=np.float64)
        dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float64)

        # 3D object points for a square marker of known size (centred at origin)
        half = CALIBRATION_MARKER_SIZE / 2
        obj_pts = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float64)

        img_pts = corner_set.reshape(4, 2).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, cam_matrix, dist_coeffs)
        if ok:
            return rvec, tvec

    return None, None


def rvec_tvec_to_matrix(rvec, tvec) -> np.ndarray:
    """Convert Rodrigues rvec + tvec to a 4×4 transform matrix."""
    R, _ = cv2.Rodrigues(rvec)
    T    = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = tvec.flatten()
    return T


# ---------------------------------------------------------------------------
# Calibration computation
# ---------------------------------------------------------------------------

def compute_calibration(
    T_gripper_to_base_list: list[np.ndarray],
    T_marker_to_cam_list:   list[np.ndarray],
) -> np.ndarray:
    """
    Run cv2.calibrateHandEye (Tsai) and return T_cam_to_base as a 4×4 matrix.

    Hand-eye convention used here:
      A = T_gripper_to_base  (robot kinematics)
      B = T_marker_to_cam    (vision, marker on gripper)

    calibrateHandEye solves: A @ X = X @ B  →  X = T_gripper_to_cam
    Then T_cam_to_base = T_gripper_to_base @ inv(T_gripper_to_cam).

    Actually the OpenCV convention for eye-to-hand is:
      R_gripper2base, t_gripper2base  — from robot FK
      R_target2cam,   t_target2cam    — from vision

    Result R_cam2base, t_cam2base gives T_cam_to_base directly.
    """
    R_g2b = [T[:3, :3] for T in T_gripper_to_base_list]
    t_g2b = [T[:3, [3]] for T in T_gripper_to_base_list]
    R_t2c = [T[:3, :3] for T in T_marker_to_cam_list]
    t_t2c = [T[:3, [3]] for T in T_marker_to_cam_list]

    R_cam2base, t_cam2base = cv2.calibrateHandEye(
        R_g2b, t_g2b, R_t2c, t_t2c,
        method=cv2.CALIB_HAND_EYE_TSAI,
    )

    T_cam_to_base = np.eye(4)
    T_cam_to_base[:3, :3] = R_cam2base
    T_cam_to_base[:3,  3] = t_cam2base.flatten()
    return T_cam_to_base


def estimate_reprojection_error(
    T_cam_to_base: np.ndarray,
    T_gripper_to_base_list: list[np.ndarray],
    T_marker_to_cam_list:   list[np.ndarray],
) -> float:
    """
    Simple consistency check: for each pair, compute where the marker should
    be in base frame via both paths and report the mean translation error (mm).
    """
    errors = []
    T_base_to_cam = np.linalg.inv(T_cam_to_base)

    for T_g2b, T_m2c in zip(T_gripper_to_base_list, T_marker_to_cam_list):
        # Marker position in base frame via robot FK
        p_marker_base_fk = T_g2b[:3, 3]   # gripper ≈ marker (mounted at TCP)

        # Marker position in base frame via camera
        p_marker_cam_h   = np.append(T_m2c[:3, 3], 1.0)
        p_marker_base_cv = (T_cam_to_base @ p_marker_cam_h)[:3]

        err_mm = np.linalg.norm(p_marker_base_fk - p_marker_base_cv) * 1000
        errors.append(err_mm)

    return float(np.mean(errors))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def run():
    ip = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_ARM_IP

    print(f"[calibration] Connecting to xArm at {ip} …")
    arm = connect_arm(ip)

    pipeline, intr = open_realsense()
    detector_bundle = build_detector()

    T_gripper_to_base_list: list[np.ndarray] = []
    T_marker_to_cam_list:   list[np.ndarray] = []

    print("\n[calibration] Controls:")
    print("  Move the arm to a new pose, then press SPACE to capture.")
    print("  Press C to compute calibration (need ≥10 poses).")
    print("  Press ESC or q to quit.\n")

    try:
        while True:
            frame = get_color_frame(pipeline)
            if frame is None:
                continue

            rvec, tvec = detect_marker_pose(frame, detector_bundle, intr)

            # Draw detection state on frame
            overlay = frame.copy()
            n = len(T_gripper_to_base_list)
            status = f"Poses captured: {n}  |  {'Marker DETECTED' if rvec is not None else 'No marker'}"
            cv2.putText(overlay, status, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if rvec is not None else (0, 0, 255), 2)
            cv2.putText(overlay, "SPACE=capture  C=compute  ESC=quit", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

            if rvec is not None:
                # Draw marker axes for visual confirmation
                cam_matrix = np.array([
                    [intr.fx, 0, intr.ppx],
                    [0, intr.fy, intr.ppy],
                    [0, 0, 1],
                ], dtype=np.float64)
                dist_coeffs = np.array(intr.coeffs, dtype=np.float64)
                cv2.drawFrameAxes(overlay, cam_matrix, dist_coeffs, rvec, tvec,
                                  CALIBRATION_MARKER_SIZE / 2)

            cv2.imshow("Eye-to-Hand Calibration", overlay)

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                print("[calibration] Quit without saving.")
                break

            elif key == ord(" "):
                if rvec is None:
                    print("[calibration] SPACE pressed but marker not detected — try again.")
                    continue

                T_g2b = get_gripper_to_base(arm)
                if T_g2b is None:
                    print("[calibration] Could not read arm pose — try again.")
                    continue

                T_m2c = rvec_tvec_to_matrix(rvec, tvec)
                T_gripper_to_base_list.append(T_g2b)
                T_marker_to_cam_list.append(T_m2c)

                dist_m = np.linalg.norm(tvec) * 1000
                print(f"[calibration] Captured pose {len(T_gripper_to_base_list):2d}  "
                      f"| marker dist={dist_m:.0f} mm  "
                      f"| arm xyz=({T_g2b[0,3]*1000:.0f}, {T_g2b[1,3]*1000:.0f}, {T_g2b[2,3]*1000:.0f}) mm")

            elif key == ord("c"):
                if len(T_gripper_to_base_list) < MIN_POSES:
                    print(f"[calibration] Need at least {MIN_POSES} poses (have {len(T_gripper_to_base_list)}).")
                    continue

                print(f"\n[calibration] Computing with {len(T_gripper_to_base_list)} poses …")
                T_cam_to_base = compute_calibration(T_gripper_to_base_list, T_marker_to_cam_list)

                err_mm = estimate_reprojection_error(
                    T_cam_to_base, T_gripper_to_base_list, T_marker_to_cam_list
                )
                print(f"[calibration] Mean consistency error: {err_mm:.1f} mm")
                if err_mm > 10:
                    print("[calibration] WARNING: error > 10 mm — consider recapturing poses.")

                np.save(OUTPUT_PATH, T_cam_to_base)
                print(f"[calibration] Saved T_cam_to_base → {OUTPUT_PATH}")
                print("\nT_cam_to_base =")
                print(np.array2string(T_cam_to_base, precision=4, suppress_small=True))
                break

    finally:
        pipeline.stop()
        arm.disconnect()
        cv2.destroyAllWindows()
        print("[calibration] Done.")


if __name__ == "__main__":
    run()
