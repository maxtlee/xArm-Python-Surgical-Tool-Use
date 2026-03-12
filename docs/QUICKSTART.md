# Quick Start Guide - MuJoCo Simulation

## The System is Now Fully Functional ✓

All issues with the MuJoCo simulation have been fixed. Here's how to use it:

## Default Mode: Mock (Works Everywhere)

The system defaults to **mock mode**, which requires no special dependencies:

```bash
python surgical_test/voice-movement.py
```

This mode simulates the robot by printing to console. Perfect for:
- Testing the GUI logic
- CI/CD pipelines
- Running without a display
- Quick iteration

## Optional: MuJoCo 3D Physics Simulation

To use the physics simulation:

### 1. Install MuJoCo
```bash
pip install mujoco>=3.1.0
```

### 2. Verify Installation
```bash
python test_mujoco.py
```

This tests everything and reports any issues.

### 3. Update Configuration
Edit `surgical.conf`:
```ini
[sim]
mode = mujoco
vis_mode = headless
```

**vis_mode options:**
- `headless` - No display (fastest, recommended)
- `viewer` - Interactive 3D window (requires `pip install mujoco[viewer]`)
- `meshcat` - Web viewer (requires `pip install meshcat`)

### 4. Run
```bash
python surgical_test/voice-movement.py
```

## Verification Tests

### Quick Test (30 seconds)
```bash
python scripts/smoke_test.py
```
Tests the entire pipeline with all 17 commands.

### Full MuJoCo Test (2 minutes)
```bash
python test_mujoco.py
```
Comprehensive check of MuJoCo setup with detailed diagnostics.

## What Was Fixed

| Issue | Status |
|-------|--------|
| Module import error | ✓ Fixed |
| Destructor crash | ✓ Fixed |
| Viewer initialization | ✓ Fixed with fallback |
| Scene validation | ✓ Fixed |
| Path resolution | ✓ Fixed |

## Common Issues

**Q: "No module named 'surgical_test'"**
- A: Run from repo root: `cd xArm-Python-Surgical-Tool-Use`

**Q: "Failed to launch viewer"**
- A: Install viewer: `pip install mujoco[viewer]`
- Or use headless mode: set `vis_mode = headless`

**Q: "XML Error: Spacing must be larger than geometry size"**
- A: Already fixed in scene.xml ✓

**Q: "No module named 'SpeechRecognition'"**
- A: Install: `pip install SpeechRecognition pyaudio`

## Architecture Summary

```
Three simulation layers:
├── Mock (default)     - Zero dependencies, console output
├── MuJoCo            - Physics simulation with soft bodies
└── Real Hardware     - Physical xArm6 with foam hand
```

All three modes work with the same interface:
```python
from surgical_test.sim import get_robot
from surgical_test.command_dispatcher import CommandDispatcher

robot = get_robot('surgical.conf')  # Auto-selects mode
dispatcher = CommandDispatcher(robot)

# All commands work the same way:
dispatcher.dispatch({'command': 'move_right', 'magnitude': 'small'})
dispatcher.dispatch({'command': 'grip'})
dispatcher.dispatch({'command': 'go_home'})
```

## Full Documentation

- **Setup Guide:** `MUJOCO_SETUP.md`
- **Detailed Fixes:** `FIXES_SUMMARY.md`
- **Project Guide:** `CLAUDE.md`
- **README:** `README.md`

## Next Steps

1. ✓ System is ready to use with mock mode
2. (Optional) Run `python test_mujoco.py` to test MuJoCo
3. (Optional) Update `surgical.conf` to switch modes
4. Install `SpeechRecognition` to enable voice commands
5. Run the GUI: `python surgical_test/voice-movement.py`
