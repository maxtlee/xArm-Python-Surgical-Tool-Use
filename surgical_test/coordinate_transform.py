"""
Camera-to-robot coordinate transformation.

Uses the ArUco marker on the robot base as a bridge between the camera
coordinate frame and the robot base frame.

Convention
----------
All positions are in mm.  Rotations use Rodrigues vectors (OpenCV convention)
or 3x3 rotation matrices internally.  The public API accepts/returns mm arrays.
"""

import numpy as np
import cv2

from surgical_test.config import (
    BASE_MARKER_OFFSET_MM,
    BASE_MARKER_RVEC_IN_ROBOT,
    WORKSPACE_MIN,
    WORKSPACE_MAX,
    DEFAULT_EE_ORIENTATION,
)


# ------------------------------------------------------------------
# Homogeneous-matrix helpers
# ------------------------------------------------------------------

def _rvec_tvec_to_mat(rvec, tvec):
    """Build a 4x4 homogeneous transform from a Rodrigues rvec and tvec (mm)."""
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).flatten())
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).flatten()
    return T


def _invert_transform(T):
    """Invert a 4x4 rigid-body transform efficiently."""
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


# ------------------------------------------------------------------
# Core transform computation
# ------------------------------------------------------------------

def compute_camera_to_robot_transform(base_rvec, base_tvec):
    """
    Compute the 4x4 homogeneous matrix that maps a point in the
    **camera frame** to the **robot base frame**.

    Parameters
    ----------
    base_rvec : array-like, shape (3,)
        Rodrigues rotation of the base marker as seen by the camera.
    base_tvec : array-like, shape (3,)
        Translation of the base marker in the camera frame (mm).

    Returns
    -------
    np.ndarray, shape (4, 4)
        T_robot_camera — multiply a homogeneous point in camera coords
        to obtain the same point in robot coords.

    Derivation
    ----------
    T_cam_marker  : camera  ← marker  (from ArUco pose estimation)
    T_robot_marker: robot   ← marker  (known calibration constant)
    T_robot_camera = T_robot_marker @ inv(T_cam_marker)
    """
    T_cam_marker = _rvec_tvec_to_mat(base_rvec, base_tvec)
    T_robot_marker = _rvec_tvec_to_mat(BASE_MARKER_RVEC_IN_ROBOT, BASE_MARKER_OFFSET_MM)
    T_robot_camera = T_robot_marker @ _invert_transform(T_cam_marker)
    return T_robot_camera


def transform_point(T_robot_camera, point_camera_mm):
    """
    Apply the camera-to-robot transform to a 3-D point.

    Parameters
    ----------
    T_robot_camera : np.ndarray, shape (4, 4)
    point_camera_mm : array-like, shape (3,)

    Returns
    -------
    np.ndarray, shape (3,)
        The point expressed in the robot base frame (mm).
    """
    p_h = np.ones(4, dtype=np.float64)
    p_h[:3] = np.asarray(point_camera_mm, dtype=np.float64).flatten()
    return (T_robot_camera @ p_h)[:3]


# ------------------------------------------------------------------
# Incision target computation
# ------------------------------------------------------------------

def compute_incision_target(marker_a_tvec, marker_b_tvec, T_robot_camera):
    """
    Compute the target pose for the end-effector at the incision midpoint.

    Parameters
    ----------
    marker_a_tvec, marker_b_tvec : array-like, shape (3,)
        Positions of the two incision-end markers in the **camera** frame (mm).
    T_robot_camera : np.ndarray, shape (4, 4)
        Camera-to-robot transform.

    Returns
    -------
    target : np.ndarray, shape (6,)
        [x, y, z, roll, pitch, yaw] in mm and degrees, suitable for
        ``arm.set_position()``.
    """
    pt_a = transform_point(T_robot_camera, marker_a_tvec)
    pt_b = transform_point(T_robot_camera, marker_b_tvec)
    midpoint = (pt_a + pt_b) / 2.0

    roll, pitch, yaw = DEFAULT_EE_ORIENTATION
    return np.array([midpoint[0], midpoint[1], midpoint[2],
                     roll, pitch, yaw], dtype=np.float64)


# ------------------------------------------------------------------
# Safety check
# ------------------------------------------------------------------

def is_within_workspace(target_xyz):
    """Return True if *target_xyz* (mm) lies inside the configured bounding box."""
    pt = np.asarray(target_xyz[:3], dtype=np.float64)
    return bool(np.all(pt >= WORKSPACE_MIN) and np.all(pt <= WORKSPACE_MAX))
