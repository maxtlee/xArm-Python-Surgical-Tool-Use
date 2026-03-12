# MuJoCo Simulation Setup Guide

This guide explains how to set up and use the MuJoCo physics simulation for the surgical robot.

## Quick Status Check

To verify your MuJoCo installation is complete and working:

```bash
python test_mujoco.py
```

This script will check:
- ✓ MuJoCo is installed
- ✓ Viewer module is available
- ✓ Scene file exists and loads
- ✓ Robot class can be instantiated
- ✓ Basic operations work

## Installation Steps

### Step 1: Install MuJoCo (required)

```bash
pip install mujoco>=3.1.0
```

### Step 2: Install MuJoCo Viewer (for interactive visualization)

```bash
pip install mujoco[viewer]
```

Or if that doesn't work:

```bash
pip install mujoco-viewer
```

### Step 3: Optional - Install Rendering Tools

For video recording:

```bash
pip install mediapy
```

For web-based 3D viewer:

```bash
pip install meshcat
```

## Simulation Modes

The simulation supports three visualization modes:

### 1. Headless (No Display - Recommended for CI/Testing)

```bash
# In surgical.conf:
[sim]
mode     = mujoco
vis_mode = headless
```

Use this for:
- Continuous integration
- Running on remote machines
- Automated testing
- Running without a display

The simulation runs at full speed with no visualization overhead.

### 2. Viewer (Interactive 3D Window - Recommended for Development)

```bash
# In surgical.conf:
[sim]
mode     = mujoco
vis_mode = viewer
```

**Requires:** `pip install mujoco[viewer]`

Use this for:
- Interactive testing
- Debugging movements
- Watching hand-tissue interaction in real-time

If the viewer fails to initialize, it will automatically fall back to headless mode.

### 3. Meshcat (Web-Based 3D Viewer)

```bash
# In surgical.conf:
[sim]
mode     = mujoco
vis_mode = meshcat
```

**Requires:** `pip install meshcat`

Use this for:
- Viewing on a different machine (access via web browser)
- Running on headless servers with remote access
- Sharing visualizations

Access at: http://localhost:7000

## Troubleshooting

### Error: "module 'mujoco' has no attribute 'viewer'"

**Cause:** Viewer module not installed or incompatible version.

**Fix:**
```bash
pip install --upgrade mujoco[viewer]
```

Or fall back to headless mode in `surgical.conf`.

### Error: "XML Error: Spacing must be larger than geometry size"

**Cause:** Flexcomp soft body elements have invalid spacing parameters.

**Status:** ✓ Already fixed in scene.xml (spacing increased to valid values)

### Error: "Failed to initialize simulation: [XML parsing error]"

**Check:**
1. Ensure `surgical_test/sim/scene.xml` exists
2. Ensure you're running from the repo root:
   ```bash
   cd xArm-Python-Surgical-Tool-Use
   python surgical_test/voice-movement.py
   ```

### Simulation runs slow or freezes

**Cause:** Interactive viewer has high overhead.

**Fix:**
1. Use headless mode for normal testing
2. Use viewer mode only when debugging
3. Ensure timestep is reasonable (default: 0.002s)

## Default Configuration

The repo defaults to **mock mode** (`surgical.conf`), which requires no dependencies and works everywhere.

To use MuJoCo:
1. Install MuJoCo: `pip install mujoco>=3.1.0`
2. Run the test: `python test_mujoco.py`
3. Update `surgical.conf`:
   ```ini
   [sim]
   mode = mujoco
   vis_mode = headless    # or viewer, or meshcat
   ```
4. Run: `python surgical_test/voice-movement.py`

## Scene File Reference

The simulation scene is defined in `surgical_test/sim/scene.xml`:

- **Floor:** Simple ground plane for reference
- **xArm6 stub:** 6-DOF arm model (placeholder, replace with real URDF)
- **Foam hand:** Soft body (flexcomp) simulating the tendon-driven gripper
- **Tissue:** Deformable surface for surgery simulation

### Replacing with Real URDF

If you have the real xArm6 URDF:

1. Download URDF:
   ```bash
   # Unix/Linux/Mac
   bash scripts/fetch_xarm_urdf.sh

   # Windows
   scripts\fetch_xarm_urdf.bat
   ```

2. Convert to MuJoCo format:
   ```bash
   python scripts/urdf_to_mjcf.py
   ```

3. Include in scene.xml:
   ```xml
   <include file="../urdf/xarm6.xml"/>
   ```

## Performance Tips

1. **Headless mode is faster** - Use for repeated testing
2. **Reduce timestep for precision** - Edit scene.xml: `timestep="0.001"`
3. **Increase timestep for speed** - `timestep="0.005"`
4. **Disable visualization updates** - Don't sync viewer every step
5. **Use implicit integrator** - Already set in scene.xml (faster, more stable)

## Testing Contact Forces

The simulator computes contact forces between the foam hand and tissue. Example:

```python
from surgical_test.sim import get_robot

robot = get_robot('surgical.conf')
robot.move_relative(dz=-10)  # Push down into tissue
forces = robot.get_contact_forces()
for geom1, geom2, force_mag in forces:
    print(f"Contact: {geom1} <-> {geom2}: {force_mag:.2f} N")
```

## Next Steps

1. Run `python test_mujoco.py` to verify your setup
2. Choose a vis_mode in `surgical.conf`
3. Run `python surgical_test/voice-movement.py` to test with the GUI
4. Replace the stub arm with real URDF if available
