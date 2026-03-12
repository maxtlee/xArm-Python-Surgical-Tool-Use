# Default Arm Orientation Configuration

The system now supports configurable default arm orientations through the `[pose]` section in `surgical.conf`.

## Quick Start

### Default Position (Provided)

The system comes with sensible defaults:
```ini
[pose]
home_x = 300.0      # mm forward
home_y = 0.0        # mm (0=centered, negative=left, positive=right)
home_z = 300.0      # mm up
home_roll = 0.0     # degrees
home_pitch = 0.0    # degrees
home_yaw = 0.0      # degrees
```

### Example: Surgical Reaching Pose

For reaching into a surgical field, you might want to start with the arm tilted forward:

```ini
[pose]
home_x = 350.0      # Further forward
home_y = 0.0        # Centered
home_z = 200.0      # Lower (toward surgical field)
home_roll = 0.0     # No rotation around x-axis
home_pitch = -30.0  # Tilted forward (nose down)
home_yaw = 0.0      # Facing forward
```

### Example: Side Approach

For accessing from the side of a patient:

```ini
[pose]
home_x = 200.0      # Less forward
home_y = 150.0      # Offset to the right
home_z = 350.0      # Higher
home_roll = 0.0
home_pitch = 0.0
home_yaw = 90.0     # Rotated to face right
```

## Configuration Reference

### Mock Robot Pose Parameters

All values apply to the mock robot (console-based simulation).

**Position (Cartesian Coordinates):**
- `home_x` - Forward/backward distance in mm (0=base, positive=forward)
- `home_y` - Left/right distance in mm (negative=left, 0=center, positive=right)
- `home_z` - Up/down distance in mm (0=base level, positive=up)

**Orientation (Euler Angles):**
- `home_roll` - Rotation around X-axis (forward/back) in degrees
- `home_pitch` - Rotation around Y-axis (left/right) in degrees
- `home_yaw` - Rotation around Z-axis (vertical) in degrees

### MuJoCo Robot Joint Angles

For physics-based simulation, you can specify joint angles directly.

**Parameter:**
- `joint_angles` - Comma-separated list of 6 joint angles in radians

The angles correspond to the 6 joints of the arm stub (or real xArm6 if using URDF):
- Joint 1: Base rotation (pan)
- Joint 2: Shoulder pitch
- Joint 3: Elbow
- Joint 4: Wrist roll
- Joint 5: Wrist pitch
- Joint 6: Wrist roll

**Example: Neutral pose**
```ini
[pose]
joint_angles = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
```

**Example: Reaching down**
```ini
[pose]
joint_angles = 0.0, -0.5, 1.57, 0.0, 1.57, 0.0
```

**Example: Horizontal reach**
```ini
[pose]
joint_angles = 0.0, -1.57, 0.0, 0.0, 0.0, 0.0
```

## How It Works

### Initialization Flow

```
surgical.conf
    ↓
get_robot()
    ├─ Reads [pose] section
    ├─ Parses home position (mock) or joint_angles (mujoco)
    ↓
Robot Class __init__()
    ├─ Stores home_pose or home_qpos
    ├─ Sets initial position/orientation
    ↓
Ready to use
```

### Home Command Behavior

When you issue a "go home" command (or call `robot.go_home()`):

1. **Mock Mode:**
   ```
   Current pose → Reset to configured home_pose → Print new pose
   ```

2. **MuJoCo Mode:**
   ```
   Current joint angles → Reset to configured joint_angles → Simulate settling
   ```

## Practical Use Cases

### 1. Starting Ready for Surgery

Position the arm already aimed at the surgical site:

```ini
[pose]
home_x = 400.0
home_y = -30.0
home_z = 180.0
home_pitch = -45.0
```

Then movements like "move_forward" are relative to this already-positioned arm.

### 2. Safe Position Between Cases

Store the arm in a safe, out-of-the-way position:

```ini
[pose]
home_x = 100.0      # Retracted
home_y = 0.0
home_z = 500.0      # High
home_pitch = -80.0  # Pointing down (safely away)
```

### 3. Calibration Reference

Use a known reference pose for calibration:

```ini
[pose]
home_x = 300.0
home_y = 0.0
home_z = 300.0
home_roll = 0.0
home_pitch = 0.0
home_yaw = 0.0
```

## Testing Your Configuration

### Test with Mock Mode

```bash
# Edit surgical.conf with your desired pose
# Then run:
python -c "
from surgical_test.sim import get_robot
robot = get_robot('surgical.conf')
print(f'Home pose: {robot.pose}')
robot.move_relative(dz=20)
robot.go_home()
"
```

### Test with MuJoCo Mode

```bash
# Edit surgical.conf with joint_angles and set mode=mujoco
python test_mujoco_movement.py
```

## Common Issues

### Q: "KeyError" when reading pose values

**Cause:** Missing `[pose]` section in surgical.conf

**Fix:**
```ini
[pose]
home_x = 300.0
home_y = 0.0
home_z = 300.0
home_roll = 0.0
home_pitch = 0.0
home_yaw = 0.0
```

### Q: MuJoCo doesn't move to the specified joint angles

**Possible causes:**
1. Joint angles are in radians, not degrees
2. Angles are outside joint limits
3. Configuration was overridden by code

**Check:**
```python
robot = get_robot('surgical.conf')
print(f"Joint angles: {robot.home_qpos}")
```

### Q: "go_home" doesn't return to the right position

**Cause:** Home position was modified during movement

**Fix:**
```python
# Reset manually
robot.go_home()  # Uses stored home_pose/home_qpos
```

## Advanced: Custom Poses

### Computing Reachable Positions

For MuJoCo, you can find reachable poses by running IK:

```python
from surgical_test.sim import get_robot

robot = get_robot('surgical.conf')
target = [350, 50, 250]  # mm: target position
success = robot._ik_simple(np.array(target) / 1000.0)  # Convert to meters

if success:
    print(f"Reachable! Joint angles: {robot.data.qpos[:6]}")
    # Save these angles to surgical.conf
```

### Forward Kinematics

Check where the arm points with given joint angles:

```python
robot = get_robot('surgical.conf')
pose = robot.get_pose()
print(f"End-effector at: {pose}")
```

## Integration with Voice Commands

The default pose affects all voice-based movements:

```
Voice: "move_forward"
  → Current home position + 20mm
  → Works seamlessly with any configured pose
```

All 17 commands (move_left, move_right, grip, etc.) work relative to the configured home position.

## Summary

- **Edit `[pose]` section** in `surgical.conf` to set default orientation
- **Mock mode:** Cartesian coordinates (x, y, z) + Euler angles (roll, pitch, yaw)
- **MuJoCo mode:** 6 joint angles in radians (optional, defaults to zeros)
- **All movements** are relative to the configured home position
- **go_home command** returns to the configured default

This allows you to optimize the arm's starting position for your specific surgical workflow.
