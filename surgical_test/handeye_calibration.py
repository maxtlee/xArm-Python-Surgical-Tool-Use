#!/usr/bin/env python3
"""
Eye-to-hand calibration (fixed camera, ArUco GridBoard on the robot).

Computes the rigid transform from the camera frame to the robot base frame
so you can map a 3-D point in the camera (e.g. from ArUco pose estimation)
into the base frame:

    p_base_hom = T_base_cam @ p_cam_hom

OpenCV's calibrateHandEye for this setup returns (R_cam2base, t_cam2base) with
p_base = R @ p_cam + t, i.e. T_base_cam in homogeneous form.

Inputs must use the SAME printed board geometry as in ``generate-board``.

Pose convention
-----------------
Each JSON entry is ``T_base_tcp``: 4x4 homogeneous transform mapping points
from the TCP (gripper) frame to the base frame:

    p_base = T_base_tcp @ p_tcp

For calibrateHandEye (eye-to-hand) we pass the inverse:

    T_tcp_base = inv(T_base_tcp)   # OpenCV: base-to-gripper mapping

Units: store ``T_base_tcp`` translations in millimetres (matches the rest of
this project). Internally, hand-eye uses metres for OpenCV; results are saved
in millimetres.

Example poses file (``poses.json``)::

    {
      "T_base_tcp": [
        [[r11, r12, r13, tx], [r21, r22, r23, ty], [r31, r32, r33, tz], [0, 0, 0, 1]],
        ...
      ]
    }
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Defaults — must match the printed board from ``generate-board``
# ---------------------------------------------------------------------------

DEFAULT_ARUCO_DICT = cv2.aruco.DICT_4X4_50
DEFAULT_GRID = (3, 3)  # markers in x, markers in y
DEFAULT_MARKER_LENGTH_MM = 50.0
DEFAULT_MARKER_SEPARATION_MM = 10.0
DEFAULT_CALIBRATION_JSON = Path(__file__).resolve().parent / "calibration.json"


# ---------------------------------------------------------------------------
# I/O helpers
# ---------------------------------------------------------------------------

def load_intrinsics(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load camera matrix and distortion coeffs from calibration.json."""
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    mtx = np.asarray(data["mtx"], dtype=np.float64)
    dist = np.asarray(data["dist"], dtype=np.float64)
    if dist.ndim == 1:
        dist = dist.reshape(-1, 1)
    return mtx, dist


def load_poses_json(path: Path) -> list[np.ndarray]:
    """Load a list of 4x4 ``T_base_tcp`` matrices (mm)."""
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    if "T_base_tcp" not in data:
        raise KeyError('JSON must contain a "T_base_tcp" array of 4x4 matrices.')
    poses = []
    for i, block in enumerate(data["T_base_tcp"]):
        T = np.asarray(block, dtype=np.float64)
        if T.shape != (4, 4):
            raise ValueError(f"T_base_tcp[{i}] must be 4x4, got {T.shape}.")
        poses.append(T)
    return poses


def _homogeneous_inverse(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = (-R.T @ t).flatten()
    return Ti


def _T_from_Rt(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


# ---------------------------------------------------------------------------
# Board
# ---------------------------------------------------------------------------

def make_grid_board(
    grid: tuple[int, int],
    marker_length_mm: float,
    marker_separation_mm: float,
    aruco_dict_type: int = DEFAULT_ARUCO_DICT,
) -> cv2.aruco.GridBoard:
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    m_len_m = marker_length_mm / 1000.0
    sep_m = marker_separation_mm / 1000.0
    return cv2.aruco.GridBoard(grid, m_len_m, sep_m, aruco_dict)


def generate_board_image(
    out_path: Path,
    grid: tuple[int, int],
    marker_length_mm: float,
    marker_separation_mm: float,
    out_size: tuple[int, int] = (4000, 4000),
    margin_px: int = 20,
) -> None:
    """Write a printable PNG of the ArUco grid board."""
    board = make_grid_board(grid, marker_length_mm, marker_separation_mm)
    img = board.generateImage(out_size, marginSize=margin_px, borderBits=1)
    if len(img.shape) == 2:
        img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    else:
        img_bgr = img
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), img_bgr)
    print(f"Wrote board image to {out_path.resolve()}")
    print(
        f"  Grid: {grid[0]} x {grid[1]} markers, "
        f"marker side = {marker_length_mm} mm, gap = {marker_separation_mm} mm"
    )


def estimate_board_pose(
    bgr: np.ndarray,
    board: cv2.aruco.GridBoard,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    aruco_dict_type: int = DEFAULT_ARUCO_DICT,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Return (R_target2cam, t_target2cam) with translation in **metres**.

    OpenCV convention: p_cam = R @ p_target + t.
    """
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is None or len(ids) == 0:
        raise RuntimeError(
            "No ArUco markers detected — check lighting, focus, and dictionary (DICT_4X4_50)."
        )

    obj_pts, img_pts = board.matchImagePoints(corners, ids)
    if obj_pts is None or len(obj_pts) < 4:
        raise RuntimeError(
            "Not enough board correspondences — ensure several grid markers are visible "
            "and geometry matches --marker-length / --separation."
        )

    obj_xyz = np.ascontiguousarray(obj_pts.reshape(-1, 3), dtype=np.float64)
    img_uv = np.ascontiguousarray(img_pts.reshape(-1, 2), dtype=np.float64)

    ok, rvec, tvec = cv2.solvePnP(
        obj_xyz,
        img_uv,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        raise RuntimeError("solvePnP failed for the detected board.")

    R_tc, _ = cv2.Rodrigues(rvec)
    t_tc = tvec.reshape(3, 1)
    return R_tc, t_tc


# ---------------------------------------------------------------------------
# Hand–eye (eye-to-hand)
# ---------------------------------------------------------------------------

def calibrate_eye_to_hand(
    images: list[np.ndarray],
    T_base_tcp_list: list[np.ndarray],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    board: cv2.aruco.GridBoard,
    method: int = cv2.CALIB_HAND_EYE_PARK,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Returns ``T_base_cam`` (4x4, translation in mm): p_base = T_base_cam @ p_cam.

    ``T_base_tcp`` must use mm; board pose uses metres internally (OpenCV).
    """
    if len(images) != len(T_base_tcp_list):
        raise ValueError("Number of images must match number of poses.")

    R_base2gripper: list[np.ndarray] = []
    t_base2gripper: list[np.ndarray] = []
    R_target2cam: list[np.ndarray] = []
    t_target2cam: list[np.ndarray] = []

    for img, T_base_tcp in zip(images, T_base_tcp_list):
        T_tcp_base = _homogeneous_inverse(T_base_tcp)
        R_b2g = T_tcp_base[:3, :3]
        t_b2g = T_tcp_base[:3, 3:4]

        R_tc, t_tc = estimate_board_pose(img, board, camera_matrix, dist_coeffs)

        R_base2gripper.append(R_b2g)
        t_base2gripper.append(t_b2g)
        R_target2cam.append(R_tc)
        t_target2cam.append(t_tc)

    # OpenCV expects gripper translations in metres for this dataset.
    t_base2gripper_m = [t / 1000.0 for t in t_base2gripper]

    R_cb, t_cb = cv2.calibrateHandEye(
        R_base2gripper,
        t_base2gripper_m,
        R_target2cam,
        t_target2cam,
        method=method,
    )
    t_cb_mm = t_cb.reshape(3) * 1000.0
    T_base_cam = _T_from_Rt(R_cb, t_cb_mm)
    return T_base_cam, (R_cb, t_cb_mm.reshape(3, 1))


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_method(name: str) -> int:
    mapping = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    key = name.upper().strip()
    if key not in mapping:
        raise argparse.ArgumentTypeError(f"Unknown method {name!r}; choose from {list(mapping)}")
    return mapping[key]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    p_gen = sub.add_parser("generate-board", help="Write a printable ArUco GridBoard PNG.")
    p_gen.add_argument("-o", "--output", type=Path, default=Path("handeye_aruco_board.png"))
    p_gen.add_argument("--grid-x", type=int, default=DEFAULT_GRID[0])
    p_gen.add_argument("--grid-y", type=int, default=DEFAULT_GRID[1])
    p_gen.add_argument("--marker-length", type=float, default=DEFAULT_MARKER_LENGTH_MM, help="Marker side length (mm).")
    p_gen.add_argument("--separation", type=float, default=DEFAULT_MARKER_SEPARATION_MM, help="Gap between markers (mm).")
    p_gen.add_argument("--width", type=int, default=4000)
    p_gen.add_argument("--height", type=int, default=4000)
    p_gen.add_argument("--margin", type=int, default=20)

    p_cal = sub.add_parser("calibrate", help="Run eye-to-hand calibration from images + poses.")
    p_cal.add_argument(
        "--calibration",
        type=Path,
        default=DEFAULT_CALIBRATION_JSON,
        help="Path to calibration.json (camera matrix + dist).",
    )
    p_cal.add_argument("--images", nargs="+", type=Path, required=True, help="Calibration images (same order as poses).")
    p_cal.add_argument("--poses", type=Path, required=True, help="JSON file with T_base_tcp list.")
    p_cal.add_argument("-o", "--output", type=Path, default=Path("handeye_result.json"))
    p_cal.add_argument("--grid-x", type=int, default=DEFAULT_GRID[0])
    p_cal.add_argument("--grid-y", type=int, default=DEFAULT_GRID[1])
    p_cal.add_argument("--marker-length", type=float, default=DEFAULT_MARKER_LENGTH_MM)
    p_cal.add_argument("--separation", type=float, default=DEFAULT_MARKER_SEPARATION_MM)
    p_cal.add_argument("--method", type=_parse_method, default="PARK")
    p_cal.add_argument("--preview-dir", type=Path, default=None, help="If set, save detection overlays here.")

    args = parser.parse_args(argv)

    if args.command == "generate-board":
        generate_board_image(
            args.output,
            (args.grid_x, args.grid_y),
            args.marker_length,
            args.separation,
            out_size=(args.width, args.height),
            margin_px=args.margin,
        )
        return 0

    # calibrate
    mtx, dist = load_intrinsics(args.calibration)
    poses = load_poses_json(args.poses)
    if len(poses) != len(args.images):
        print(
            f"Error: got {len(args.images)} images but {len(poses)} poses.",
            file=sys.stderr,
        )
        return 1

    board = make_grid_board(
        (args.grid_x, args.grid_y),
        args.marker_length,
        args.separation,
    )

    images_bgr: list[np.ndarray] = []
    for p in args.images:
        im = cv2.imread(str(p))
        if im is None:
            print(f"Error: could not read image {p}", file=sys.stderr)
            return 1
        images_bgr.append(im)

    if args.preview_dir is not None:
        args.preview_dir.mkdir(parents=True, exist_ok=True)
        aruco_dict = cv2.aruco.getPredefinedDictionary(DEFAULT_ARUCO_DICT)
        det = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
        for i, im in enumerate(images_bgr):
            gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = det.detectMarkers(gray)
            vis = im.copy()
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(vis, corners, ids)
            cv2.imwrite(str(args.preview_dir / f"det_{i:02d}.png"), vis)

    try:
        T_base_cam, _ = calibrate_eye_to_hand(
            images_bgr,
            poses,
            mtx,
            dist,
            board,
            method=args.method,
        )
    except Exception as e:
        print(f"Calibration failed: {e}", file=sys.stderr)
        return 1

    T_cam_base = _homogeneous_inverse(T_base_cam)

    out = {
        "description": (
            "Eye-to-hand result. T_base_cam maps homogeneous points from camera frame to base "
            "(p_base = T_base_cam @ p_cam). Translations in millimetres."
        ),
        "T_base_cam": T_base_cam.tolist(),
        "T_cam_base": T_cam_base.tolist(),
        "units": "mm for translation components in 4x4 matrices",
        "calibration_intrinsics_file": str(args.calibration.resolve()),
        "method": args.method,
        "board": {
            "grid": [args.grid_x, args.grid_y],
            "marker_length_mm": args.marker_length,
            "marker_separation_mm": args.separation,
            "aruco_dict": "DICT_4X4_50",
        },
    }

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)

    np.set_printoptions(precision=4, suppress=True)
    print("Hand–eye calibration OK.")
    print("T_base_cam (p_base = T_base_cam @ p_cam), translation mm:\n", T_base_cam)
    print("\nSaved:", out_path.resolve())
    print(
        "\nTo map an ArUco point in camera frame (mm) to base: "
        "p_base_hom = T_base_cam @ np.append(p_cam_mm, 1.0)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
