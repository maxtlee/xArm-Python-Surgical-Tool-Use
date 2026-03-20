#!/usr/bin/env python3
"""
cv_aruco_test.py — 2D ArUco marker detection scaffold for surgical retractor demo
==================================================================================

PURPOSE:
  Uses an Intel RealSense D435i color stream and OpenCV ArUco detection to:
    1. Detect marker ID 0 and ID 1 placed at the two ends of a fake incision
    2. Draw a red line connecting their centers
    3. Compute 4 evenly spaced target spots along that line (at 12.5%, 37.5%, 62.5%, 87.5%)
    4. Draw and label the spots S1–S4 in blue
    5. Print marker centers and spot pixel coordinates to the terminal each frame

WHY ArUco MARKERS:
  ArUco markers provide a simple, deterministic way to define the two ends of the
  incision without needing a trained vision model. Each marker has a unique integer
  ID, so we always know which end is "start" (ID 0) vs "end" (ID 1). This gives a
  clean 2D detection scaffold we can build depth sensing and robot control on top of.

DEPENDENCIES:
  pip install opencv-contrib-python pyrealsense2 numpy

RUN:
  python surgical_test/cv_aruco_test.py            # auto-detects RealSense or falls back to webcam
  python surgical_test/cv_aruco_test.py --webcam   # force webcam (no RealSense needed)

CONTROLS:
  q or ESC — quit

LATER INTEGRATION POINTS (search for TODO tags in this file):
  TODO[DEPTH]       — convert spot pixels → 3D camera-frame coords using depth frame
  TODO[CALIBRATION] — transform camera-frame 3D points → xArm base-frame via T_cam_to_base
  TODO[ROBOT]       — send arm.set_position() to move the xArm to a target spot
  TODO[VOICE]       — hook "move to spot N" voice command into the existing voice GUI

RECOMMENDED PROJECT STRUCTURE FOR FUTURE FILES:
  surgical_test/
    cv_aruco_test.py        ← this file: 2D detection scaffold
    cv_depth_test.py        ← TODO: adds depth-to-3D conversion on top of this
    cv_calibration.py       ← TODO: hand-eye / extrinsic calibration routine
    voice-movement.py       ← existing voice GUI (do not modify yet)
    gripper_poses.yaml      ← existing gripper config

  vision/                   ← helper modules (extract from this file once stable)
    __init__.py
    realsense_stream.py     ← open/close pipeline, grab aligned color+depth frames
    aruco_detector.py       ← detect markers, compute centers, draw overlays
    spot_generator.py       ← compute N spots along a line segment
    depth_utils.py          ← TODO: pixel+depth → 3D using camera intrinsics
    calibration.py          ← TODO: load T_cam_to_base, apply rigid transform
"""

import sys

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
    _REALSENSE_AVAILABLE = True
except ImportError:
    _REALSENSE_AVAILABLE = False

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

STREAM_WIDTH  = 640
STREAM_HEIGHT = 480
STREAM_FPS    = 30

# ArUco dictionary — 4x4 grid, 50 unique IDs. Must match the printed markers.
ARUCO_DICT_ID = cv2.aruco.DICT_4X4_50

# The two marker IDs that define the incision endpoints.
MARKER_START_ID = 0   # placed at one end of the incision
MARKER_END_ID   = 1   # placed at the other end

# Number of target spots evenly distributed between the two markers.
# Spots are placed at positions 1/(2n), 3/(2n), … along the line so they are
# centred in each equal segment rather than sitting on the endpoints.
NUM_SPOTS = 4

# Drawing colours (BGR)
COLOR_LINE   = (0,   0,   255)  # red  — line between markers
COLOR_MARKER = (0,   255, 0  )  # green — marker centre dot
COLOR_SPOT   = (255, 100, 0  )  # blue  — target spots
COLOR_LABEL  = (255, 255, 255)  # white — text labels


# ---------------------------------------------------------------------------
# RealSense helpers
# ---------------------------------------------------------------------------

def _realsense_device_connected() -> bool:
    """Return True if at least one RealSense device is plugged in."""
    if not _REALSENSE_AVAILABLE:
        return False
    ctx = rs.context()
    return len(ctx.query_devices()) > 0


def open_realsense(width: int, height: int, fps: int):
    """
    Start a RealSense pipeline with color + depth.
    On macOS, depth requires running with sudo for USB device access.

    Returns (pipeline, intr_dict) where intr_dict has fx/fy/ppx/ppy.
    """
    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16,  fps)
    pipeline.start(config)
    print(f"[RealSense] Pipeline started (color + depth) — {width}×{height} @ {fps} fps")

    frames      = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    intr        = color_frame.profile.as_video_stream_profile().get_intrinsics()
    intr_dict   = {"fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx, "ppy": intr.ppy}
    print(f"[RealSense] Intrinsics: fx={intr.fx:.1f} fy={intr.fy:.1f} "
          f"cx={intr.ppx:.1f} cy={intr.ppy:.1f}")
    return pipeline, intr_dict


def get_realsense_frames(pipeline):
    """
    Grab one frameset. Returns (color_array, depth_array_m).
    depth_array_m is (H,W) float32 metres, 0 where invalid.
    All data copied to numpy — no rs2 objects escape this function.
    """
    frames      = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    if not color_frame:
        return None, None
    color_array = np.asanyarray(color_frame.get_data()).copy()
    depth_array = None
    if depth_frame:
        scale       = depth_frame.get_units()
        raw         = np.asanyarray(depth_frame.get_data()).copy().astype(np.float32)
        depth_array = raw * scale
    return color_array, depth_array


# ---------------------------------------------------------------------------
# Depth helpers  (pure numpy — no rs2 frame lifetime issues)
# ---------------------------------------------------------------------------

def get_depth_at_pixel(depth_array: np.ndarray | None, x: int, y: int, kernel: int = 5) -> float:
    """
    Return the median depth (metres) over a kernel×kernel patch centred on (x, y).
    Returns 0.0 if no valid (nonzero) readings exist in the patch.
    """
    if depth_array is None:
        return 0.0
    h, w = depth_array.shape
    half = kernel // 2
    patch = depth_array[max(0, y-half):min(h, y+half+1),
                        max(0, x-half):min(w, x+half+1)]
    valid = patch[patch > 0]
    return float(np.median(valid)) if len(valid) > 0 else 0.0


def pixel_to_3d(depth_array: np.ndarray | None, intr: dict, x: float, y: float) -> tuple[float, float, float]:
    """
    Deproject pixel (x, y) to a 3D point in camera frame (mm).
    Uses the pinhole model directly — no rs2 API calls.
    Returns (0, 0, 0) when depth is unavailable.
    """
    z_m = get_depth_at_pixel(depth_array, int(x), int(y))
    if z_m == 0.0:
        return (0.0, 0.0, 0.0)
    X = (x - intr["ppx"]) / intr["fx"] * z_m * 1000
    Y = (y - intr["ppy"]) / intr["fy"] * z_m * 1000
    Z = z_m * 1000
    return (X, Y, Z)


def open_webcam(index: int = 0) -> cv2.VideoCapture:
    """Open the default webcam as a fallback when no RealSense is connected."""
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        print(f"ERROR: Could not open webcam (index {index}).")
        sys.exit(1)
    print(f"[Webcam] Opened camera index {index} — RealSense not available or not connected.")
    return cap


def get_color_frame_webcam(cap: cv2.VideoCapture) -> np.ndarray | None:
    """Grab one frame from the webcam, returning a BGR numpy array or None."""
    ret, frame = cap.read()
    return frame if ret else None


# ---------------------------------------------------------------------------
# ArUco helpers
# ---------------------------------------------------------------------------

def build_aruco_detector(dict_id: int):
    """
    Build and return an ArUco detector for the given dictionary ID.

    Handles both the new API (OpenCV ≥ 4.7) and the legacy API gracefully.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)

    # OpenCV 4.7+ uses ArucoDetector; older versions use detectMarkers directly.
    if hasattr(cv2.aruco, "ArucoDetector"):
        params   = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        return detector, dictionary, "new"
    else:
        params = cv2.aruco.DetectorParameters_create()
        return None, dictionary, params  # caller handles the old-style call


def detect_markers(frame: np.ndarray, detector_bundle) -> dict[int, tuple[float, float]]:
    """
    Run ArUco detection on a BGR frame.

    Returns a dict mapping detected marker ID → (cx, cy) pixel center.
    Only returns entries for markers that were actually detected.

    detector_bundle is the tuple returned by build_aruco_detector().
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    detector, dictionary, extra = detector_bundle

    if extra == "new":
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        # Legacy OpenCV < 4.7 path
        params = extra
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)

    centers = {}
    if ids is None:
        return centers

    for corner_set, marker_id in zip(corners, ids.flatten()):
        # corner_set shape: (1, 4, 2) — four corners in pixel coordinates
        pts = corner_set.reshape(4, 2)
        cx  = float(pts[:, 0].mean())
        cy  = float(pts[:, 1].mean())
        centers[int(marker_id)] = (cx, cy)

    return centers


# ---------------------------------------------------------------------------
# Spot computation
# ---------------------------------------------------------------------------

def compute_spots(
    pt_a: tuple[float, float],
    pt_b: tuple[float, float],
    n: int = NUM_SPOTS,
) -> list[tuple[float, float]]:
    """
    Return n evenly spaced points along the line segment from pt_a to pt_b.

    Points are placed at fractional positions t = (2k-1)/(2n) for k = 1..n,
    centring each spot within its equal-length segment of the line.

    For n=4: t = [0.125, 0.375, 0.625, 0.875]

    TODO[DEPTH]: Once depth is available, extend this to return 3D spots
    instead of 2D pixels by calling depth_utils.pixel_to_3d() on each result.

    TODO[CALIBRATION]: After converting to camera-frame 3D points, apply
    T_cam_to_base (a 4×4 rigid transform) to get xArm base-frame coordinates
    that can be passed directly to arm.set_position().
    """
    ax, ay = pt_a
    bx, by = pt_b
    spots  = []
    for k in range(1, n + 1):
        t = (2 * k - 1) / (2 * n)
        spots.append((ax + t * (bx - ax), ay + t * (by - ay)))
    return spots


# ---------------------------------------------------------------------------
# Drawing helpers
# ---------------------------------------------------------------------------

def draw_overlay(
    frame:   np.ndarray,
    centers: dict[int, tuple[float, float]],
    spots:   list[tuple[float, float]],
) -> None:
    """
    Draw all visual annotations onto frame in-place:
      - Green dot at each detected marker centre with its ID label
      - Red line connecting the two incision-end markers (if both visible)
      - Blue circles and S1–S4 labels at each target spot
    """
    # Draw each detected marker centre
    for marker_id, (cx, cy) in centers.items():
        pt = (int(cx), int(cy))
        cv2.circle(frame, pt, 8, COLOR_MARKER, -1)
        cv2.putText(
            frame, f"M{marker_id}", (pt[0] + 10, pt[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_MARKER, 2,
        )

    # Draw connecting line and spots only when both endpoint markers are present
    if MARKER_START_ID in centers and MARKER_END_ID in centers:
        pa = tuple(int(v) for v in centers[MARKER_START_ID])
        pb = tuple(int(v) for v in centers[MARKER_END_ID])
        cv2.line(frame, pa, pb, COLOR_LINE, 2)

        for i, (sx, sy) in enumerate(spots):
            pt    = (int(sx), int(sy))
            label = f"S{i + 1}"
            cv2.circle(frame, pt, 10, COLOR_SPOT, -1)
            cv2.circle(frame, pt, 10, (255, 255, 255), 2)   # white ring
            cv2.putText(
                frame, label, (pt[0] + 13, pt[1] + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, COLOR_LABEL, 2,
            )

    # Status overlay: how many target markers are visible
    n_detected = sum(1 for m in (MARKER_START_ID, MARKER_END_ID) if m in centers)
    status     = f"Markers: {n_detected}/2  |  Spots: {len(spots)}"
    cv2.putText(
        frame, status, (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 2,
    )


# ---------------------------------------------------------------------------
# Terminal output
# ---------------------------------------------------------------------------

def print_coordinates(
    centers:   dict[int, tuple[float, float]],
    spots:     list[tuple[float, float]],
    spots_3d:  list[tuple[float, float, float]] | None = None,
) -> None:
    """
    Print a compact summary of marker centres and spot pixel + 3D coordinates.
    Only prints when both endpoint markers are visible to reduce noise.

    spots_3d: list of (X, Y, Z) mm tuples in camera frame, one per spot.
              Pass None (or omit) when depth is not available.
    """
    if MARKER_START_ID not in centers or MARKER_END_ID not in centers:
        return

    cx0, cy0 = centers[MARKER_START_ID]
    cx1, cy1 = centers[MARKER_END_ID]
    print(f"  M0 center : ({cx0:.1f}, {cy0:.1f})   M1 center : ({cx1:.1f}, {cy1:.1f})")
    for i, (sx, sy) in enumerate(spots):
        if spots_3d and i < len(spots_3d):
            X, Y, Z = spots_3d[i]
            if Z > 0:
                print(f"  S{i + 1}  pixel=({sx:.0f}, {sy:.0f})  3D=({X:+.1f}, {Y:+.1f}, {Z:.1f}) mm")
            else:
                print(f"  S{i + 1}  pixel=({sx:.0f}, {sy:.0f})  3D=no depth")
        else:
            print(f"  S{i + 1}         : ({sx:.1f}, {sy:.1f})")
    print()


# ---------------------------------------------------------------------------
# Main capture loop
# ---------------------------------------------------------------------------

def run():
    """
    Open either a RealSense pipeline or the system webcam (auto-detected),
    detect markers every frame, draw overlays, and print coordinates when
    both endpoint markers are visible. Press q or ESC to quit.

    Pass --webcam on the command line to force webcam mode.
    """
    force_webcam = "--webcam" in sys.argv

    use_realsense = (not force_webcam) and _realsense_device_connected()

    if use_realsense:
        pipeline, rs_intrinsics = open_realsense(STREAM_WIDTH, STREAM_HEIGHT, STREAM_FPS)
        depth_state   = {"frame": None}

        def _get_frame_rs():
            color, depth = get_realsense_frames(pipeline)
            depth_state["frame"] = depth
            return color

        get_frame = _get_frame_rs
        cleanup   = lambda: (pipeline.stop(), print("[RealSense] Pipeline stopped."))
    else:
        if not force_webcam and not _REALSENSE_AVAILABLE:
            print("[cv_aruco_test] pyrealsense2 not installed — falling back to webcam.")
        elif not force_webcam:
            print("[cv_aruco_test] No RealSense device detected — falling back to webcam.")
        cap           = open_webcam()
        depth_state   = {"frame": None}
        rs_intrinsics = None
        get_frame     = lambda: get_color_frame_webcam(cap)
        cleanup       = lambda: (cap.release(), print("[Webcam] Released."))

    detector_bundle = build_aruco_detector(ARUCO_DICT_ID)

    print("[cv_aruco_test] Running — press q or ESC to quit\n")

    prev_spot_hash = None   # used to avoid printing identical coordinates repeatedly

    try:
        while True:
            frame = get_frame()
            if frame is None:
                continue

            # Detect all visible ArUco markers in this frame
            centers = detect_markers(frame, detector_bundle)

            # Compute spots only when both incision-end markers are present
            spots = []
            if MARKER_START_ID in centers and MARKER_END_ID in centers:
                spots = compute_spots(
                    centers[MARKER_START_ID],
                    centers[MARKER_END_ID],
                    NUM_SPOTS,
                )

            # Compute 3D camera-frame coords for each spot (RealSense only)
            depth_array = depth_state["frame"]   # numpy (H,W) float32 metres, or None
            spots_3d = None
            if spots and rs_intrinsics is not None and depth_array is not None:
                spots_3d = [pixel_to_3d(depth_array, rs_intrinsics, sx, sy)
                            for sx, sy in spots]

            # Draw overlays onto the frame
            draw_overlay(frame, centers, spots)

            # Print coordinates to terminal (only when something changed)
            if spots:
                spot_hash = tuple((round(x), round(y)) for x, y in spots)
                if spot_hash != prev_spot_hash:
                    print_coordinates(centers, spots, spots_3d)
                    prev_spot_hash = spot_hash
            else:
                if prev_spot_hash is not None:
                    print("[cv_aruco_test] Waiting for both markers…")
                prev_spot_hash = None

            cv2.imshow("ArUco Incision Tracker", frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):  # 27 = ESC
                break

    finally:
        cleanup()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    run()
