# xArm-Python-Surgical-Tool-Use

Using the xArm and DexKit hands to operate as a tool-using and voice-commanded assistant for hand surgeons

Project funded by Carnegie Mellon Foam Robotics Lab and advised by Dr. Nancy Pollard

Forked from https://github.com/xArm-Developer/xArm-Python-SDK.git

## Setup

1. Clone this repository

```bash
    git clone https://github.com/maxtlee/xArm-Python-Surgical-Tool-Use.git
    cd xArm-Python-Surgical-Tool-Use
```

2. Install packages (Python 3 required)

Linux/Mac

```bash
    sudo apt install portaudio19-dev python3-tk
    python3 -m venv venv
    source venv/bin/activate
    pip install -r requirements.txt
```

Windows (CMD)

Tkinter is bundled with the standard Python installer (ensure "tcl/tk and IDLE" is checked). PyAudio installs via pip without a separate PortAudio download. You might also need to install C++. Python 3.11 is needed. You also probably need to upgrade pip

```bash
    python3 -m venv venv
    .\venv\Scripts\activate.bat
    pip install -r requirements.txt
```

Windows (PowerShell)

Tkinter is bundled with the standard Python installer (ensure "tcl/tk and IDLE" is checked). PyAudio installs via pip without a separate PortAudio download. You might also need to install C++. Python 3.11 is needed. You also probably need to upgrade pip

```bash
    python3 -m venv venv
    .\venv\Scripts\Activate.ps1
    pip install -r requirements.txt
```

3. Connect to the xArm

Plug in via ethernet and power on the xArm. Set your host machine's wired network adapter to a **static IPv4** address on the same subnet as the robot:

| Field   | Value         |
| ------- | ------------- |
| IP      | 192.168.1.50  |
| Netmask | 255.255.255.0 |

The robot's default IP is **192.168.1.195**. Verify connectivity with `ping 192.168.1.195`.

5. Run the voice-control app

```bash
    python3 surgical_test/voice-movement.py
```

A GUI window will open. Click **Connect** (the IP is pre-filled from `robot.conf`), then **Start Listening**. Speak commands to move individual joints or send the arm home:

| Voice command                  | Action                  |
| ------------------------------ | ----------------------- |
| "joint one/two/.../seven up"   | Move that joint +20°    |
| "joint one/two/.../seven down" | Move that joint −20°    |
| "go home"                      | Return to home position |
| "stop"                         | Emergency stop          |

# DexKit Hand Usage

In order to run the dexkit hand control script (dexKit_control/NIBIB1.py), you will have to change the port name to match the one it is on your computer. the port name is hardcoded in NIBIB1.py line 206. 

## Simulation modes

The system supports three simulation layers, controlled by `surgical.conf`:

### Mock (no dependencies beyond base install)
Set `mode = mock` in `surgical.conf`. All commands print to console.
No display required. Good for CI and logic testing.

```bash
python surgical_test/voice-movement.py
```

### MuJoCo 3D simulation (recommended for local development)
Install MuJoCo, then run:

**Unix/Linux/Mac:**
```bash
pip install mujoco mediapy
bash scripts/fetch_xarm_urdf.sh    # downloads xArm6 URDF + meshes
python scripts/urdf_to_mjcf.py     # optional: converts to MJCF
```

**Windows:**
```bash
pip install mujoco mediapy
scripts\fetch_xarm_urdf.bat        # downloads xArm6 URDF + meshes
python scripts\urdf_to_mjcf.py     # optional: converts to MJCF
```

Set `mode = mujoco` and `vis_mode = viewer` in `surgical.conf`.

```bash
python surgical_test/voice-movement.py
```

A 3D window opens showing the xArm, the soft foam hand (flexcomp), and the
deformable tissue surface. Foam hand ↔ tissue contact forces are computed by
MuJoCo's unified contact engine and printed to the console.

For headless rendering (no display, exports mp4):

```
Set vis_mode = headless in surgical.conf
```

For browser-based 3D view:

```bash
pip install meshcat
# Set vis_mode = meshcat in surgical.conf
# Opens at http://localhost:7000
```

### Real hardware (lab only)
Set `mode = real` in `surgical.conf`. Connect xArm via Ethernet and foam hand
via USB serial. See the connection setup section above for network configuration.

## Foam hand CAD → simulation

To add accurate foam hand geometry to the simulation:
- SolidWorks: use sw_urdf_exporter, then reference the mesh in scene.xml flexcomp
- OnShape: use onshape-to-robot (https://github.com/Rhoban/onshape-to-robot)
- Place exported meshes in `urdf/meshes/` and update the flexcomp `file=` attribute
  in `surgical_test/sim/scene.xml`

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
