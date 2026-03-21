"""
Calibration workflow for the surgical retractor system.

Captures ArUco marker poses, computes the camera-to-robot transform, and
stores the incision target so that voice commands can use it later.

Usage from the GUI
------------------
1. Instantiate ``CalibrationManager(on_log=callback)``.
2. Call ``start_camera()``  — opens camera, begins live preview thread.
3. Call ``capture()``       — averages N frames, validates all markers found.
4. Call ``stop_camera()``   — shuts down the camera (markers may now be removed).
5. Read ``incision_target`` — the [x, y, z, roll, pitch, yaw] for the arm.
"""

import threading
import numpy as np
import cv2

from surgical_test.config import (
    MARKER_ID_BASE,
    MARKER_ID_INCISION_A,
    MARKER_ID_INCISION_B,
    CAPTURE_AVG_FRAMES,
)
from surgical_test.aruco_detector import ArucoDetector
from surgical_test.coordinate_transform import (
    compute_camera_to_robot_transform,
    compute_incision_target,
    is_within_workspace,
)

REQUIRED_IDS = {MARKER_ID_BASE, MARKER_ID_INCISION_A, MARKER_ID_INCISION_B}


class CalibrationManager:
    """Manages the full calibration lifecycle."""

    def __init__(self, on_log=None):
        """
        Parameters
        ----------
        on_log : callable(str) or None
            Function called with status/error messages (thread-safe expected).
        """
        self._log = on_log or (lambda msg: None)
        self._detector = ArucoDetector()

        self.cam_to_robot = None          # 4x4 transform, set after capture
        self.incision_target = None       # [x, y, z, roll, pitch, yaw]
        self.is_calibrated = False

        self._preview_thread = None
        self._preview_stop = threading.Event()
        self._last_frame = None
        self._last_frame_lock = threading.Lock()

    # ------------------------------------------------------------------
    # Camera lifecycle
    # ------------------------------------------------------------------

    def start_camera(self):
        """Open the camera and begin a background preview loop."""
        try:
            self._detector.start()
        except Exception as e:
            self._log(f"Camera start failed: {e}")
            raise

        self._preview_stop.clear()
        self._preview_thread = threading.Thread(target=self._preview_loop, daemon=True)
        self._preview_thread.start()
        self._log("Camera started — position markers and verify in preview.")

    def stop_camera(self):
        """Stop the preview and release camera resources."""
        self._preview_stop.set()
        if self._preview_thread is not None:
            self._preview_thread.join(timeout=3)
            self._preview_thread = None
        self._detector.stop()
        self._log("Camera stopped.")

    def _preview_loop(self):
        """Continuously grab frames and update ``_last_frame`` for the GUI.

        Does NOT call cv2.imshow — the Tkinter main thread polls
        ``get_last_frame()`` instead (required on macOS).
        """
        while not self._preview_stop.is_set():
            detections, annotated = self._detector.capture_single()
            if annotated is None:
                continue
            self._draw_status_overlay(annotated, detections)
            with self._last_frame_lock:
                self._last_frame = annotated

    def _draw_status_overlay(self, frame, detections):
        """Draw which required markers are currently visible."""
        y = 30
        for mid in sorted(REQUIRED_IDS):
            found = mid in detections
            colour = (0, 255, 0) if found else (0, 0, 255)
            label = f"Marker {mid}: {'OK' if found else 'NOT FOUND'}"
            cv2.putText(frame, label, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, colour, 2)
            y += 25

    def get_last_frame(self):
        """Return the most recent annotated preview frame (or None)."""
        with self._last_frame_lock:
            return self._last_frame.copy() if self._last_frame is not None else None

    # ------------------------------------------------------------------
    # Capture & calibration
    # ------------------------------------------------------------------

    def capture(self):
        """
        Average multiple frames, compute the transform, and store the target.

        Returns True on success, False if any required marker is missing or
        the computed target is outside the workspace.
        """
        self._log(f"Capturing {CAPTURE_AVG_FRAMES} frames …")
        detections, _ = self._detector.capture_averaged(n_frames=CAPTURE_AVG_FRAMES)

        missing = REQUIRED_IDS - set(detections.keys())
        if missing:
            self._log(f"Calibration FAILED — missing markers: {sorted(missing)}")
            return False

        base_rvec, base_tvec = detections[MARKER_ID_BASE]
        inc_a_rvec, inc_a_tvec = detections[MARKER_ID_INCISION_A]
        inc_b_rvec, inc_b_tvec = detections[MARKER_ID_INCISION_B]

        self.cam_to_robot = compute_camera_to_robot_transform(base_rvec, base_tvec)
        self.incision_target = compute_incision_target(inc_a_tvec, inc_b_tvec,
                                                       self.cam_to_robot)

        if not is_within_workspace(self.incision_target):
            self._log(
                f"Calibration FAILED — target {np.round(self.incision_target[:3], 1)} "
                "is outside the workspace bounds."
            )
            self.incision_target = None
            return False

        self.is_calibrated = True
        self._log(
            f"Calibration OK — incision target: "
            f"x={self.incision_target[0]:.1f}  "
            f"y={self.incision_target[1]:.1f}  "
            f"z={self.incision_target[2]:.1f} mm"
        )
        return True

    def reset(self):
        """Clear stored calibration data."""
        self.cam_to_robot = None
        self.incision_target = None
        self.is_calibrated = False
        self._log("Calibration reset.")
