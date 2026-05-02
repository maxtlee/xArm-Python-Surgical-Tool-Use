# Dexkit-Surgical-Assistance

Aims to use the xArm and DexKit hand as a tool-using and voice-commanded assistant for hand surgeons. Uses OpenAI Whisper for speech-to-text and performs basic keyword detection to set incremental or absolute setpoints to the xArm and DexKit hand. 

Project run by undergraduates in the Carnegie Mellon Foam Robotics Lab and advised by Dr. Nancy Pollard.

Forked from https://github.com/xArm-Developer/xArm-Python-SDK.git

---

## Project Structure

```
surgical_test/
  voice-movement.py       # voice-controlled xArm GUI and demo script (main app)
  cv_aruco_test.py        # standalone CV test: ArUco marker detection + incision spot generation
  make_aruco_markers.py   # generate printable ArUco marker PNGs (run once)
  gripper_poses.yaml      # DexHand open/close pose definitions
```

## Branches

```
main                      # Working demo code
vision                    # Experimental code testing localization using an Intel RealSense depth/RGB camera for ArUco marker detection
deepgram                  # Experimental code testing API use for faster speech-to-text
sim                       # Experimental code implementing a simulator in MuJoCo
```

---

## Setup

### 1. Clone this repository

```bash
git clone https://github.com/maxtlee/xArm-Python-Surgical-Tool-Use.git
cd xArm-Python-Surgical-Tool-Use
```

### 2. Install packages

**Linux / Mac (voice control + xArm SDK)**

```bash
sudo apt install portaudio19-dev python3-tk   # Linux only
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

**Windows (CMD)**

Tkinter is bundled with the standard Python installer (ensure "tcl/tk and IDLE" is checked). Python 3.11 recommended.

```bash
python3 -m venv .venv
.venv\Scripts\activate.bat
pip install -r requirements.txt
```

**Windows (PowerShell)**

```bash
python3 -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

**Computer vision pipeline (macOS — all platforms)**

`pyrealsense2` is not available via pip on macOS. Use a dedicated conda environment:

```bash
conda create -n surgical-cv python=3.13 -y
conda activate surgical-cv
conda install -c conda-forge pyrealsense2 -y
pip install opencv-contrib-python numpy
```

On Linux, `pip install pyrealsense2` works without conda.

### 3. Connect to the xArm

Plug in via ethernet and power on the xArm. Set your host machine's wired network adapter to a **static IPv4** address on the same subnet as the robot:

| Field   | Value         |
| ------- | ------------- |
| IP      | 192.168.1.50  |
| Netmask | 255.255.255.0 |

The robot's default IP is **192.168.1.195**. Verify connectivity with `ping 192.168.1.195`.

Set the IP in `example/wrapper/robot.conf`:

```ini
[xArm]
ip = 192.168.1.195
```

---

## Running the Apps

### Voice-controlled xArm GUI

```bash
source .venv/bin/activate
python3 surgical_test/voice-movement.py
```

Click **Connect** (IP pre-filled from `robot.conf`), then **Start Listening**.

| Voice command          | Action                    |
| ---------------------- | ------------------------- |
| `"forward [qualifier]"` | Move end-effector +X     |
| `"backward [qualifier]"` | Move end-effector −X    |
| `"left [qualifier]"`   | Move end-effector +Y      |
| `"right [qualifier]"`  | Move end-effector −Y      |
| `"up [qualifier]"`     | Move end-effector +Z      |
| `"down [qualifier]"`   | Move end-effector −Z      |
| `"go home"`            | Return to home position   |
| `"pickup"`             | Move to pickup position   |
| `"engage"`             | Move to engage position   |
| `"retract"`            | Move to retract position  |
| `"open"` / `"close"`  | Open / close DexHand      |
| `"stop"`               | Emergency stop            |

Magnitude qualifiers: `"a little"` = 5 mm · *(none)* = 20 mm · `"more"` = 40 mm · `"a lot"` = 80 mm

### Computer Vision Pipeline (ArUco incision tracking)

```bash
conda activate surgical-cv

# Step 1: generate printable marker PNGs (run once)
python3 surgical_test/make_aruco_markers.py
# → prints aruco_marker_0.png and aruco_marker_1.png
# Print at actual size (4–6 cm per side). Place ID 0 at one end of the
# fake incision, ID 1 at the other.

# Step 2: run the live detection test
python3 surgical_test/cv_aruco_test.py
```

The live window shows:
- Green dots at each detected marker centre
- Red line connecting the two incision endpoints
- Blue circles labelled S1–S4 at 4 evenly spaced spots along the incision
- Pixel coordinates printed to the terminal each frame

Press **q** or **ESC** to quit.

---

## DexKit Hand Usage

To run the DexKit hand control script (`dexKit_control/NIBIB1.py`), change the port name to match your machine. The port is hardcoded at line 206 of `NIBIB1.py`.

# xArm-Python-SDK

[![PyPI Downloads](https://static.pepy.tech/badge/xarm-python-sdk)](https://pepy.tech/projects/xarm-python-sdk)

## Overview

xArm Python SDK

## Caution

- Please keep away from the robot arm to avoid personal injury or equipment damage.
- Make sure to do a safety assessment before moving to prevent collisions.
- Protect the arm before unlocking the joint.

## Installation

&ensp;&ensp;you can run examples without installation.Only Python3 is supported.

- Install from source code
  - download
    ```bash
    git clone https://github.com/xArm-Developer/xArm-Python-SDK.git
    cd xArm-Python-SDK
    ```
  - install
    - install with build
      ```bash
      pip install build
      python -m build
      pip install dist/xarm_python_sdk-1.16.0-py3-none-any.whl
      ```
    - install with source code
      ```bash
      pip install .
      ```
- Install from pypi
  ```bash
  pip install xarm-python-sdk
  ```

## Doc

- #### [API Document](doc/api/xarm_api.md)

- #### [API Code Document](doc/api/xarm_api_code.md)

- #### [UFACTORY ModbusTCP Manual](doc/UF_ModbusTCP_Manual.md)
