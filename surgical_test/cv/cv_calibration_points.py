#!/usr/bin/env python3
"""
cv_calibration_points.py — ArUco-assisted point-correspondence calibration
===========================================================================

No marker on the end-effector needed.

WORKFLOW:
  For ArUco marker points (M0, M1):
    1. Physically touch the arm TCP to the marker center.
    2. Wait for green banner ("Marker M0 detected — stable").
    3. Press SPACE — camera 3D is read automatically from ArUco detection.

  For non-marker points (elevated point, extra surface):
    1. Physically touch (or hover) the arm TCP to the target.
    2. LEFT-CLICK on where the TCP appears in the camera image.
    3. Press SPACE to record the pair.

  After 4+ non-coplanar points: press C to compute and save T_cam_to_base.npy.

  Suggested points:
    1. TCP touching M0 center        (auto ArUco 3D)
    2. TCP touching M1 center        (auto ArUco 3D)
    3. TCP touching a surface corner  (click fallback)
    4. TCP ~150mm above M0           (click fallback — breaks coplanar degeneracy)

USAGE:
  sudo python surgical_test/cv/cv_calibration_points.py 192.168.1.195
"""

import sys, os, time
import cv2
import numpy as np
from collections import deque

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

try:
    import pyrealsense2 as rs
except ImportError:
    print("ERROR: pyrealsense2 not installed"); sys.exit(1)

try:
    from xarm.wrapper import XArmAPI
except ImportError:
    print("ERROR: xarm package not found"); sys.exit(1)

# ---------------------------------------------------------------------------
STREAM_W, STREAM_H, STREAM_FPS = 640, 480, 30
ARUCO_DICT_ID   = cv2.aruco.DICT_4X4_50
MARKER_START_ID = 0
MARKER_END_ID   = 1
NUM_SPOTS       = 4
DEFAULT_ARM_IP  = "192.168.1.195"
OUTPUT_PATH     = os.path.join(os.path.dirname(__file__), "T_cam_to_base.npy")

# Arm goes here before you start jogging manually
HOME_JOINTS = [-156, 93, 97, 26, -93, 94, -95]

# Stability: marker center must stay within STABLE_DIST_PX pixels for STABLE_FRAMES frames
STABLE_DIST_PX  = 5
STABLE_FRAMES   = 10
# How close a click must be to a marker center to trigger auto-ArUco mode (pixels)
CLICK_MARKER_RADIUS = 40


# ---------------------------------------------------------------------------
def connect_arm(ip):
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


def open_realsense():
    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.color, STREAM_W, STREAM_H, rs.format.bgr8, STREAM_FPS)
    config.enable_stream(rs.stream.depth, STREAM_W, STREAM_H, rs.format.z16,  STREAM_FPS)
    pipeline.start(config)
    frames      = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    intr        = color_frame.profile.as_video_stream_profile().get_intrinsics()
    intr_dict   = {"fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx, "ppy": intr.ppy}
    print(f"[RealSense] Started. fx={intr.fx:.1f} ppx={intr.ppx:.1f}")
    return pipeline, intr_dict


def get_frames(pipeline):
    frames      = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    if not color_frame:
        return None, None
    color = np.asanyarray(color_frame.get_data()).copy()
    depth = None
    if depth_frame:
        scale = depth_frame.get_units()
        depth = np.asanyarray(depth_frame.get_data()).copy().astype(np.float32) * scale
    return color, depth


def get_depth_at_pixel(depth, x, y, k=5):
    if depth is None: return 0.0
    h, w = depth.shape
    patch = depth[max(0,y-k):min(h,y+k+1), max(0,x-k):min(w,x+k+1)]
    valid = patch[patch > 0]
    return float(np.median(valid)) if len(valid) else 0.0


def pixel_to_3d(depth, intr, x, y):
    """Returns point in metres (camera frame), or None if no depth."""
    z = get_depth_at_pixel(depth, int(x), int(y))
    if z == 0: return None
    X = (x - intr["ppx"]) / intr["fx"] * z
    Y = (y - intr["ppy"]) / intr["fy"] * z
    return np.array([X, Y, z])   # metres, camera frame


def build_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        return detector, dictionary, "new"
    return None, dictionary, cv2.aruco.DetectorParameters_create()


def detect_markers(frame, bundle):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detector, dictionary, extra = bundle
    if extra == "new":
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=extra)
    centers = {}
    if ids is not None:
        for cs, mid in zip(corners, ids.flatten()):
            pts = cs.reshape(4, 2)
            centers[int(mid)] = (float(pts[:,0].mean()), float(pts[:,1].mean()))
    return centers


def compute_spots(pa, pb, n=NUM_SPOTS):
    ax, ay = pa; bx, by = pb
    return [(ax + (2*k-1)/(2*n)*(bx-ax), ay + (2*k-1)/(2*n)*(by-ay)) for k in range(1, n+1)]


# ---------------------------------------------------------------------------
def solve_transform(pts_cam, pts_arm):
    """
    Solve for T_cam_to_base (4x4) given corresponding 3D points.
    pts_cam: list of np.array([x,y,z]) in metres, camera frame
    pts_arm: list of np.array([x,y,z]) in metres, arm base frame

    Uses Kabsch algorithm (SVD-based rigid transform).
    """
    A = np.array(pts_cam)   # (N, 3)
    B = np.array(pts_arm)   # (N, 3)

    centroid_A = A.mean(axis=0)
    centroid_B = B.mean(axis=0)

    AA = A - centroid_A
    BB = B - centroid_B

    H  = AA.T @ BB
    U, S, Vt = np.linalg.svd(H)
    R  = Vt.T @ U.T

    # Handle reflection
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = centroid_B - R @ centroid_A

    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = t
    return T


def reprojection_error(T, pts_cam, pts_arm):
    errors = []
    for pc, pa in zip(pts_cam, pts_arm):
        pc_h     = np.append(pc, 1.0)
        pa_pred  = (T @ pc_h)[:3]
        errors.append(np.linalg.norm(pa_pred - pa) * 1000)
    return float(np.mean(errors))


# ---------------------------------------------------------------------------
class MarkerStabilityTracker:
    """Tracks whether a marker center has been stable for STABLE_FRAMES frames."""

    def __init__(self, dist_threshold=STABLE_DIST_PX, required_frames=STABLE_FRAMES):
        self.dist_threshold  = dist_threshold
        self.required_frames = required_frames
        # marker_id -> deque of (cx, cy)
        self._history = {}

    def update(self, centers: dict):
        """Call once per frame with the current detected centers dict."""
        # Remove markers not currently visible
        for mid in list(self._history.keys()):
            if mid not in centers:
                self._history.pop(mid)
        # Add/update visible markers
        for mid, (cx, cy) in centers.items():
            if mid not in self._history:
                self._history[mid] = deque(maxlen=self.required_frames)
            self._history[mid].append((cx, cy))

    def is_stable(self, marker_id: int) -> bool:
        """True if marker has been visible and within dist_threshold for required_frames."""
        hist = self._history.get(marker_id)
        if not hist or len(hist) < self.required_frames:
            return False
        xs = [p[0] for p in hist]
        ys = [p[1] for p in hist]
        spread = max(max(xs)-min(xs), max(ys)-min(ys))
        return spread <= self.dist_threshold

    def stable_center(self, marker_id: int):
        """Return mean (cx, cy) over history if stable, else None."""
        if not self.is_stable(marker_id):
            return None
        hist = self._history[marker_id]
        cx = float(np.mean([p[0] for p in hist]))
        cy = float(np.mean([p[1] for p in hist]))
        return (cx, cy)

    def stable_markers(self) -> list:
        """Return list of marker IDs that are currently stable."""
        return [mid for mid in self._history if self.is_stable(mid)]


# ---------------------------------------------------------------------------
def draw_banner(frame, text, color_bgr):
    """Draw a filled banner at top of frame."""
    h, w = frame.shape[:2]
    cv2.rectangle(frame, (0, 0), (w, 36), color_bgr, -1)
    cv2.putText(frame, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


def run():
    ip  = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_ARM_IP
    arm = connect_arm(ip)
    pipeline, intr = open_realsense()
    bundle  = build_detector()
    tracker = MarkerStabilityTracker()

    pts_cam  = []   # metres, camera frame
    pts_arm  = []   # metres, arm base frame
    click_pt = {"x": None, "y": None}
    last_depth = {"arr": None}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            click_pt["x"], click_pt["y"] = x, y

    cv2.namedWindow("Point Calibration")
    cv2.setMouseCallback("Point Calibration", on_mouse)

    print("\n[calibration] Controls:")
    print("  For M0/M1:   touch TCP to marker center — green banner shows when stable.")
    print("               Press SPACE to auto-record (no click needed).")
    print("  For others:  LEFT-CLICK on TCP location in image, then press SPACE.")
    print("  C  = compute transform (need ≥4 non-coplanar points)")
    print("  ESC/q = quit")
    print(f"\n  Suggested sequence:")
    print(f"    Point 1 — TCP to M0 center  (auto)")
    print(f"    Point 2 — TCP to M1 center  (auto)")
    print(f"    Point 3 — TCP to a surface corner  (click)")
    print(f"    Point 4 — TCP ~150mm above M0  (click — breaks coplanar degeneracy)\n")

    try:
        while True:
            color, depth = get_frames(pipeline)
            if color is None:
                continue
            last_depth["arr"] = depth

            centers = detect_markers(color, bundle)
            tracker.update(centers)

            spots = []
            if MARKER_START_ID in centers and MARKER_END_ID in centers:
                spots = compute_spots(centers[MARKER_START_ID], centers[MARKER_END_ID])

            # ---- Draw marker/spot overlay ----
            for mid, (cx, cy) in centers.items():
                stable = tracker.is_stable(mid)
                dot_color = (0, 255, 0) if stable else (0, 180, 0)
                cv2.circle(color, (int(cx), int(cy)), 8, dot_color, -1)
                label = f"M{mid} {'STABLE' if stable else ''}"
                cv2.putText(color, label, (int(cx)+10, int(cy)-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, dot_color, 2)

            if MARKER_START_ID in centers and MARKER_END_ID in centers:
                pa = tuple(int(v) for v in centers[MARKER_START_ID])
                pb = tuple(int(v) for v in centers[MARKER_END_ID])
                cv2.line(color, pa, pb, (0,0,255), 2)
                for i, (sx, sy) in enumerate(spots):
                    cv2.circle(color, (int(sx), int(sy)), 10, (255,100,0), -1)
                    cv2.putText(color, f"S{i+1}", (int(sx)+13, int(sy)+5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)

            # ---- Draw click crosshair ----
            if click_pt["x"] is not None:
                cx, cy = click_pt["x"], click_pt["y"]
                cv2.drawMarker(color, (cx, cy), (0,255,255),
                               cv2.MARKER_CROSS, 20, 2)
                d = get_depth_at_pixel(depth, cx, cy)
                cv2.putText(color, f"depth={d*1000:.0f}mm", (cx+12, cy-8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)

            # ---- Draw previously recorded points ----
            for i, pc in enumerate(pts_cam):
                px = int(pc[0]/pc[2]*intr["fx"] + intr["ppx"])
                py = int(pc[1]/pc[2]*intr["fy"] + intr["ppy"])
                cv2.circle(color, (px, py), 6, (255,0,255), -1)
                cv2.putText(color, str(i+1), (px+8, py),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,255), 1)

            # ---- Banner: auto-mode vs click-mode ----
            stable_ids = tracker.stable_markers()
            # Determine if current click is near a stable marker
            auto_marker_id = None
            if click_pt["x"] is not None and stable_ids:
                for mid in stable_ids:
                    sc = tracker.stable_center(mid)
                    if sc:
                        dist = np.hypot(click_pt["x"] - sc[0], click_pt["y"] - sc[1])
                        if dist < CLICK_MARKER_RADIUS:
                            auto_marker_id = mid
                            break
            # If no click yet but exactly one stable marker → auto mode
            if click_pt["x"] is None and len(stable_ids) == 1:
                auto_marker_id = stable_ids[0]

            if stable_ids:
                stable_str = ", ".join(f"M{m}" for m in stable_ids)
                if auto_marker_id is not None:
                    banner_text  = f"AUTO: {stable_str} stable — touch TCP to marker, press SPACE"
                    banner_color = (30, 140, 30)  # green
                else:
                    banner_text  = f"Marker {stable_str} stable — click non-marker TCP location, press SPACE"
                    banner_color = (30, 110, 140)  # teal
            else:
                banner_text  = "No stable marker — click TCP location in image, then press SPACE"
                banner_color = (30, 100, 180)  # yellow-ish

            draw_banner(color, banner_text, banner_color)

            n = len(pts_cam)
            cv2.putText(color,
                        f"Points recorded: {n} | C=compute  ESC=quit",
                        (10, STREAM_H - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (200,200,200), 1)
            cv2.imshow("Point Calibration", color)

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break

            elif key == ord(" "):
                # ---- Determine camera 3D source ----
                # 1. Auto: click near stable marker OR single stable marker with no click
                use_marker_id = None

                if click_pt["x"] is not None:
                    # Check if click is near a stable marker
                    for mid in tracker.stable_markers():
                        sc = tracker.stable_center(mid)
                        if sc:
                            dist = np.hypot(click_pt["x"] - sc[0], click_pt["y"] - sc[1])
                            if dist < CLICK_MARKER_RADIUS:
                                use_marker_id = mid
                                break
                elif len(tracker.stable_markers()) == 1:
                    # No click, single stable marker → auto
                    use_marker_id = tracker.stable_markers()[0]

                # ---- Get arm position ----
                code, pos = arm.get_position(is_radian=False)
                if code != 0 or pos is None:
                    print("[calibration] Failed to read arm position, try again.")
                    continue

                arm_mm = np.array(pos[:3])
                arm_m  = arm_mm / 1000.0

                # ---- Get camera 3D ----
                if use_marker_id is not None:
                    sc = tracker.stable_center(use_marker_id)
                    pt_cam = pixel_to_3d(last_depth["arr"], intr, sc[0], sc[1])
                    src_desc = f"ArUco M{use_marker_id} center (auto)"
                    click_pt["x"] = click_pt["y"] = None
                elif click_pt["x"] is not None:
                    pt_cam = pixel_to_3d(last_depth["arr"], intr, click_pt["x"], click_pt["y"])
                    src_desc = f"click ({click_pt['x']}, {click_pt['y']})"
                    click_pt["x"] = click_pt["y"] = None
                else:
                    print("[calibration] No click and multiple (or zero) stable markers visible.")
                    print("             Click the TCP location in the image, or ensure only")
                    print("             one marker is stable.")
                    continue

                if pt_cam is None:
                    print("[calibration] No depth at that location — try a different spot.")
                    continue

                pts_cam.append(pt_cam)
                pts_arm.append(arm_m)

                print(f"[calibration] Point {n+1} recorded ({src_desc}):")
                print(f"  arm (mm): ({arm_mm[0]:.1f}, {arm_mm[1]:.1f}, {arm_mm[2]:.1f})")
                print(f"  cam  (m): ({pt_cam[0]:.4f}, {pt_cam[1]:.4f}, {pt_cam[2]:.4f})")

            elif key == ord("c"):
                if len(pts_cam) < 4:
                    print(f"[calibration] Need ≥4 points (have {len(pts_cam)}).")
                    continue

                T = solve_transform(pts_cam, pts_arm)
                err = reprojection_error(T, pts_cam, pts_arm)
                print(f"\n[calibration] Solved with {len(pts_cam)} points.")
                print(f"[calibration] Mean error: {err:.1f} mm")
                if err > 20:
                    print("[calibration] WARNING: error >20mm — consider adding more points.")
                np.save(OUTPUT_PATH, T)
                print(f"[calibration] Saved → {OUTPUT_PATH}")
                print("\nT_cam_to_base =")
                print(np.array2string(T, precision=4, suppress_small=True))
                break

    finally:
        pipeline.stop()
        arm.disconnect()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    run()
