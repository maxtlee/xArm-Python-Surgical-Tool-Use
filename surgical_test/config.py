"""
Surgical retractor configuration.

All spatial values are in millimetres and degrees unless noted otherwise.
"""

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# ArUco marker settings
# ---------------------------------------------------------------------------

ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50
MARKER_SIZE_MM = 50.0  # physical printed marker side length

MARKER_ID_BASE = 0          # marker fixed to the xArm base
MARKER_ID_INCISION_A = 1    # one end of incision
MARKER_ID_INCISION_B = 2    # other end of incision

# Offset of the base marker centre relative to the robot base origin,
# expressed in the robot base frame [x, y, z] (mm).
# Measure this once when you attach the marker and update here.
BASE_MARKER_OFFSET_MM = np.array([0.0, 0.0, 0.0], dtype=np.float64)

# Orientation of the base marker in the robot base frame (Rodrigues vector).
# Identity (no rotation) means the marker Z-axis is aligned with the robot Z-axis.
BASE_MARKER_RVEC_IN_ROBOT = np.array([0.0, 0.0, 0.0], dtype=np.float64)

# ---------------------------------------------------------------------------
# Camera settings
# ---------------------------------------------------------------------------

USE_REALSENSE = False  # set True if pyrealsense2 is installed and a RealSense camera is connected

# If not using RealSense, provide camera intrinsics manually.
# These are overridden at runtime when a RealSense camera is detected.
CAMERA_MATRIX = np.array([
    [615.0,   0.0, 320.0],
    [  0.0, 615.0, 240.0],
    [  0.0,   0.0,   1.0],
], dtype=np.float64)

DIST_COEFFS = np.zeros((5, 1), dtype=np.float64)

# ---------------------------------------------------------------------------
# Motion / safety settings
# ---------------------------------------------------------------------------

APPROACH_HEIGHT_MM = 50.0   # hover this far above the target before descending
MOTION_SPEED = 50           # mm/s during incision approach
MOTION_SPEED_DESCEND = 20   # mm/s when descending to the target

# Default end-effector orientation at the incision (roll, pitch, yaw in degrees).
# Adjust to match your retractor tool orientation.
DEFAULT_EE_ORIENTATION = (-180.0, 0.0, 0.0)

# Workspace bounding box in robot base frame (mm) — crude safety check.
WORKSPACE_MIN = np.array([-600.0, -600.0, -100.0])
WORKSPACE_MAX = np.array([ 600.0,  600.0,  600.0])

# Number of frames to average during marker capture (reduces noise).
CAPTURE_AVG_FRAMES = 10
