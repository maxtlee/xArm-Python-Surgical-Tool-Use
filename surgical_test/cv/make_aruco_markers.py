#!/usr/bin/env python3
"""
make_aruco_markers.py — Generate printable ArUco marker PNG files
==================================================================

Generates marker ID 0 and ID 1 from the DICT_4X4_50 dictionary.
These are the two markers used in cv_aruco_test.py to define the
two endpoints of the fake incision.

HOW TO USE:
  1. Run this script once to generate the PNG files:
       python surgical_test/make_aruco_markers.py

  2. Print aruco_marker_0.png and aruco_marker_1.png at the size you need.
     A physical size of 4–6 cm per side works well at typical camera distances.
     Print on plain white paper; ensure no scaling is applied ("actual size").

  3. Place marker ID 0 at one end of the fake incision and ID 1 at the other.

NAMING CONVENTION:
  Marker ID 0 → aruco_marker_0.png  (MARKER_START_ID in cv_aruco_test.py)
  Marker ID 1 → aruco_marker_1.png  (MARKER_END_ID   in cv_aruco_test.py)

DEPENDENCIES:
  pip install opencv-contrib-python

OUTPUT:
  PNG files are written to the same directory as this script.
"""

import os
import sys

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Configuration — must match ARUCO_DICT_ID in cv_aruco_test.py
# ---------------------------------------------------------------------------

ARUCO_DICT_ID  = cv2.aruco.DICT_4X4_50
MARKER_IDS     = [0, 1, 2]

# Output image size in pixels. 400px gives a crisp image at any print size.
MARKER_PX      = 400

# White border (in pixels) added around the marker so it is easier for the
# detector to find at the edge of the printed sheet.
BORDER_PX      = 40

OUTPUT_DIR     = os.path.dirname(os.path.abspath(__file__))


# ---------------------------------------------------------------------------
# Generation
# ---------------------------------------------------------------------------

def generate_marker(dictionary, marker_id: int, size_px: int) -> np.ndarray:
    """
    Draw a single ArUco marker and return it as a grayscale numpy array.

    Uses the new API (cv2.aruco.generateImageMarker) if available,
    falling back to the legacy drawMarker for older OpenCV versions.
    """
    img = np.zeros((size_px, size_px), dtype=np.uint8)

    if hasattr(cv2.aruco, "generateImageMarker"):
        # OpenCV >= 4.7
        cv2.aruco.generateImageMarker(dictionary, marker_id, size_px, img, 1)
    else:
        # Legacy path
        img = cv2.aruco.drawMarker(dictionary, marker_id, size_px)

    return img


def add_border(img: np.ndarray, border_px: int) -> np.ndarray:
    """Add a solid white border around the marker image."""
    return cv2.copyMakeBorder(
        img,
        border_px, border_px, border_px, border_px,
        cv2.BORDER_CONSTANT,
        value=255,
    )


def add_label(img: np.ndarray, marker_id: int) -> np.ndarray:
    """
    Add a human-readable label below the marker indicating its ID and role.
    This makes it easy to tell which printout to place at which end.
    """
    if marker_id == 0:
        role = "START (end 0)"
    elif marker_id == 1:
        role = "END   (end 1)"
    else:
        role = f"AUX   (id  {marker_id})"
    label = f"ID {marker_id}  —  {role}  —  DICT_4X4_50"

    # Convert to BGR so we can draw coloured text
    img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # Append a white strip at the bottom for the label
    strip_h = 48
    strip   = np.full((strip_h, img.shape[1], 3), 255, dtype=np.uint8)
    img_bgr = np.vstack([img_bgr, strip])

    cv2.putText(
        img_bgr, label,
        (10, img_bgr.shape[0] - 14),
        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 40, 40), 1, cv2.LINE_AA,
    )
    return img_bgr


def main():
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)

    for marker_id in MARKER_IDS:
        # 1. Generate the raw marker image
        img = generate_marker(dictionary, marker_id, MARKER_PX)

        # 2. Add a white border so the detector can find edges near sheet boundaries
        img = add_border(img, BORDER_PX)

        # 3. Add a printed label so it is obvious which physical marker is which
        img = add_label(img, marker_id)

        # 4. Save
        filename = os.path.join(OUTPUT_DIR, f"aruco_marker_{marker_id}.png")
        cv2.imwrite(filename, img)
        h, w = img.shape[:2]
        print(f"Saved: {filename}  ({w}×{h} px)")

    print(
        "\nPrint both files at actual size (no scaling)."
        "\nRecommended physical size: 4–6 cm per marker side for ~50–80 cm camera distance."
        "\nPlace marker ID 0 at one end of the fake incision, ID 1 at the other."
    )


if __name__ == "__main__":
    main()
