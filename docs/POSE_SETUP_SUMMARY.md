# Default Arm Orientation - Setup Summary

## What Was Added

A new configuration system that lets you set the arm's default starting position and orientation through `surgical.conf`, without changing code.

## Key Features

✅ **Cartesian Position Control** (Mock mode)
- Set X, Y, Z coordinates in millimeters
- Set Roll, Pitch, Yaw angles in degrees
- All movements are relative to this home position

✅ **Joint Angle Control** (MuJoCo mode)
- Set 6 joint angles in radians for precise arm configuration
- Useful for avoiding singularities or reaching specific poses

✅ **Persistent Home Position**
- `go_home` command always returns to configured position
- Used as reference point for all relative movements

✅ **Configuration File Based**
- No code changes needed
- Easy to switch between different scenarios
- Copy-paste ready examples provided

## Current Configuration

Your `surgical.conf` now includes:

```ini
[pose]
; Default home position for mock robot (x, y, z in mm)
home_x = 300.0      ; Forward distance
home_y = 0.0        ; Left/right (0=center)
home_z = 300.0      ; Up/down

; Default orientation (roll, pitch, yaw in degrees)
home_roll = 0.0
home_pitch = 0.0
home_yaw = 0.0

; MuJoCo only: default joint angles (radians) for arm joints 1-6
; Leave empty or comment out to use all zeros
; joint_angles = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
```

## Files Changed

| File | What Changed |
|------|--------------|
| `surgical.conf` | Added `[pose]` section with defaults |
| `surgical_test/sim/__init__.py` | Factory now reads pose config |
| `surgical_test/sim/mock_robot.py` | Accepts custom home_pose |
| `surgical_test/sim/mujoco_robot.py` | Accepts custom joint_angles |

## Files Created

| File | Purpose |
|------|---------|
| `POSE_CONFIGURATION.md` | Complete configuration guide |
| `POSE_EXAMPLES.md` | 10 ready-to-use configuration examples |
| `POSE_SETUP_SUMMARY.md` | This file |

## Quick Examples

### Example 1: Surgical Field Access

```ini
[pose]
home_x = 350.0
home_y = 0.0
home_z = 150.0
home_pitch = -45.0
```

Positions arm ready to work on a surgical field.

### Example 2: Safe Retracted Position

```ini
[pose]
home_x = 100.0
home_y = 0.0
home_z = 500.0
home_pitch = -80.0
```

Arm pulled back and up, safely out of the way.

### Example 3: MuJoCo Reaching Down

```ini
[pose]
joint_angles = 0.0, -0.5, 1.57, 0.0, 1.57, 0.0
```

Physics-based simulation with arm reaching toward surgical site.

For 10 complete examples, see `POSE_EXAMPLES.md`.

## How to Use

### 1. Choose Your Scenario

Look through `POSE_EXAMPLES.md` or design your own.

### 2. Edit `surgical.conf`

Replace the `[pose]` section with your desired values:

```ini
[pose]
home_x = 350.0      ; Your value
home_y = 0.0        ; Your value
home_z = 150.0      ; Your value
home_roll = 0.0
home_pitch = -45.0  ; Your value
home_yaw = 0.0
```

### 3. Test It

```bash
python surgical_test/voice-movement.py
```

The arm will start at your configured position.

### 4. Run Commands

All movements are relative to your home position:
```
Voice: "move forward" → Moves forward from configured position
Voice: "go home" → Returns to your configured position
```

## Motion Coordinate System

```
    +Y (right)
     ↑
     |
     +----→ +X (forward)
    /
   /
  +Z (up)


[home_x, home_y, home_z] = distance from robot base
[home_roll, home_pitch, home_yaw] = orientation angles
```

### Coordinate Values

| Parameter | Range | Unit | Notes |
|-----------|-------|------|-------|
| home_x | 100-500 | mm | Forward/backward |
| home_y | -300-300 | mm | Left/right |
| home_z | 100-500 | mm | Up/down |
| home_roll | -180-180 | deg | Rotation around X |
| home_pitch | -90-90 | deg | Rotation around Y (tilt) |
| home_yaw | -180-180 | deg | Rotation around Z |

## Advanced: Testing Positions

### Check Starting Position

```bash
python -c "
from surgical_test.sim import get_robot
robot = get_robot('surgical.conf')
print(f'Starting position: {robot.pose}')
"
```

### Verify Position is Reachable

For MuJoCo mode:
```bash
python -c "
from surgical_test.sim import get_robot
import numpy as np

robot = get_robot('surgical.conf')
target = np.array([350, 0, 150]) / 1000.0  # Convert mm to m
if robot._ik_simple(target):
    print('Position is reachable!')
"
```

## Troubleshooting

### Q: "go_home" doesn't go to my configured position

**Check:** Run the test above to verify position is loaded.

### Q: MuJoCo arm doesn't move to joint_angles

**Cause:** Joint angles outside valid ranges.

**Fix:** Verify angles are in radians, not degrees. Example:
- 90 degrees = 1.57 radians
- 45 degrees = 0.785 radians

### Q: Position looks wrong in viewer

**Check:** Set `vis_mode = viewer` and watch the arm position visually.

## Integration with Voice Commands

The configured home position works seamlessly with voice commands:

```
Voice Command          →  Effect
─────────────────────────────────────────────────
"move forward"         →  Move +20mm in X from home
"move right"           →  Move +20mm in Y from home
"move up"              →  Move +20mm in Z from home
"go home"              →  Return to [home_x, home_y, home_z]
"grip"                 →  Close gripper (independent of position)
```

All 17 commands work relative to your configured home position.

## Common Configurations

### For General Testing
```ini
[pose]
home_x = 300.0
home_y = 0.0
home_z = 300.0
```

### For Surgical Simulation
```ini
[pose]
home_x = 350.0
home_y = 0.0
home_z = 200.0
home_pitch = -45.0
```

### For Safety (Between Cases)
```ini
[pose]
home_x = 100.0
home_y = 0.0
home_z = 500.0
home_pitch = -80.0
```

## Next Steps

1. ✅ Pose configuration is now available
2. Choose your scenario from `POSE_EXAMPLES.md`
3. Edit `surgical.conf` with your desired pose
4. Run and test with `python surgical_test/voice-movement.py`
5. Iterate and refine based on your needs

## Documentation

- **`POSE_CONFIGURATION.md`** - Complete reference guide
- **`POSE_EXAMPLES.md`** - 10 ready-to-use examples
- **`surgical.conf`** - Configuration file with defaults

---

The system is ready to use with custom default orientations! 🎉
