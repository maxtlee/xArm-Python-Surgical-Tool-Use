"""
ArUco marker detection with depth camera support.

Supports Intel RealSense (preferred) with automatic intrinsics,
or a generic USB webcam with manually configured intrinsics.
"""

import time
import numpy as np
import cv2

from surgical_test.config import (
    ARUCO_DICT_TYPE,
    MARKER_SIZE_MM,
    CAPTURE_AVG_FRAMES,
    USE_REALSENSE,
    CAMERA_MATRIX,
    DIST_COEFFS,
)

# Try importing RealSense — graceful fallback if unavailable.
try:
    import pyrealsense2 as rs
    _HAS_REALSENSE = True
except ImportError:
    _HAS_REALSENSE = False


class ArucoDetector:
    """Detects ArUco markers and estimates their 6-DOF poses."""

    def __init__(self, use_realsense=USE_REALSENSE, marker_size_mm=MARKER_SIZE_MM):
        self.marker_size_mm = marker_size_mm
        self.use_realsense = use_realsense and _HAS_REALSENSE

        self._pipeline = None
        self._cap = None
        self._camera_matrix = CAMERA_MATRIX.copy()
        self._dist_coeffs = DIST_COEFFS.copy()

        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
        self._detector_params = cv2.aruco.DetectorParameters()
        self._aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, self._detector_params)

    # ------------------------------------------------------------------
    # Camera lifecycle
    # ------------------------------------------------------------------

    def start(self):
        """Open the camera stream and retrieve intrinsics."""
        if self.use_realsense:
            self._start_realsense()
        else:
            self._start_webcam()

    def stop(self):
        """Release camera resources."""
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def _start_realsense(self):
        self._pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = self._pipeline.start(config)

        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intrinsics = color_stream.get_intrinsics()
        self._camera_matrix = np.array([
            [intrinsics.fx, 0.0, intrinsics.ppx],
            [0.0, intrinsics.fy, intrinsics.ppy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        self._dist_coeffs = np.array(intrinsics.coeffs[:5], dtype=np.float64).reshape(5, 1)

    def _start_webcam(self):
        self._cap = cv2.VideoCapture(0)
        if not self._cap.isOpened():
            raise RuntimeError("Cannot open webcam")

    # ------------------------------------------------------------------
    # Frame capture
    # ------------------------------------------------------------------

    def _grab_color_frame(self):
        """Return a BGR numpy array from the active camera."""
        if self._pipeline is not None:
            frames = self._pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                return None
            return np.asanyarray(color_frame.get_data())

        if self._cap is not None:
            ret, frame = self._cap.read()
            return frame if ret else None

        raise RuntimeError("Camera not started — call start() first")

    # ------------------------------------------------------------------
    # Detection
    # ------------------------------------------------------------------

    def detect_markers(self, frame):
        """
        Detect ArUco markers in a BGR frame.

        Returns
        -------
        dict[int, tuple[np.ndarray, np.ndarray]]
            Mapping of marker_id -> (rvec, tvec).
            rvec is a (3,) Rodrigues rotation vector.
            tvec is a (3,) translation in mm (same scale as marker_size_mm).
        np.ndarray
            The annotated frame with drawn marker outlines and axes.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._aruco_detector.detectMarkers(gray)

        result = {}
        annotated = frame.copy()

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

            marker_len = self.marker_size_mm / 1000.0  # estimatePoseSingleMarkers uses metres
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, marker_len, self._camera_matrix, self._dist_coeffs
            )

            for i, marker_id in enumerate(ids.flatten()):
                rvec = rvecs[i].flatten()
                tvec = tvecs[i].flatten() * 1000.0  # convert back to mm
                result[int(marker_id)] = (rvec, tvec)
                cv2.drawFrameAxes(
                    annotated, self._camera_matrix, self._dist_coeffs,
                    rvecs[i], tvecs[i], marker_len * 0.5,
                )

        return result, annotated

    def capture_single(self):
        """Grab one frame, detect markers, return (detections_dict, annotated_frame)."""
        frame = self._grab_color_frame()
        if frame is None:
            return {}, None
        return self.detect_markers(frame)

    def capture_averaged(self, n_frames=CAPTURE_AVG_FRAMES, delay_ms=50):
        """
        Capture *n_frames* detections and average rvec/tvec per marker to reduce noise.

        Only markers that appear in **every** frame are included.

        Returns
        -------
        dict[int, tuple[np.ndarray, np.ndarray]]
            Averaged {marker_id: (rvec_mean, tvec_mean)}.
        np.ndarray or None
            The last annotated frame (for display purposes).
        """
        accumulator: dict[int, list[tuple[np.ndarray, np.ndarray]]] = {}
        last_annotated = None

        for _ in range(n_frames):
            detections, annotated = self.capture_single()
            if annotated is not None:
                last_annotated = annotated
            for mid, (rv, tv) in detections.items():
                accumulator.setdefault(mid, []).append((rv, tv))
            if delay_ms > 0:
                time.sleep(delay_ms / 1000.0)

        averaged = {}
        for mid, samples in accumulator.items():
            if len(samples) == n_frames:
                rvecs = np.array([s[0] for s in samples])
                tvecs = np.array([s[1] for s in samples])
                averaged[mid] = (rvecs.mean(axis=0), tvecs.mean(axis=0))

        return averaged, last_annotated

    # ------------------------------------------------------------------
    # Live preview (blocking — press 'q' to quit)
    # ------------------------------------------------------------------

    def live_preview(self, window_name="ArUco Preview"):
        """
        Show a live camera feed with detected markers overlaid.
        Press **q** to close the window and return.
        """
        while True:
            frame = self._grab_color_frame()
            if frame is None:
                continue
            _, annotated = self.detect_markers(frame)
            cv2.imshow(window_name, annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyWindow(window_name)
