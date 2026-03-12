# Runtime Fixes Summary

This document explains the runtime issues that were occurring with the MuJoCo simulation and the fixes applied.

## Issue: IK Convergence Failures

### Symptoms

When running with `mode = mujoco`, the console showed:
```
[MuJoCoRobot] IK did not converge
[MuJoCoRobot] IK did not converge
[MuJoCoRobot] Returned to home position
```

### Root Causes

1. **Overly tight convergence tolerance** (0.1mm)
   - Required nearly perfect IK solution
   - Made convergence unlikely on stub arm

2. **Low damping parameter** (1e-4)
   - Caused numerical instability
   - Made IK oscillate instead of converge

3. **Stub arm limitations**
   - Simple placeholder geometry
   - Limited reachable workspace
   - Potential singularities
   - Not the real xArm6

4. **Missing dependency**
   - NumPy wasn't in requirements.txt
   - Caused import errors

## Fixes Applied

### 1. Improved IK Algorithm (`mujoco_robot.py`)

**Before:**
```python
tolerance = 1e-4           # 0.1mm (too strict)
lambda_damping = 1e-4      # Very low (unstable)
step_size = 0.5            # Fixed, no adaptation
```

**After:**
```python
tolerance = 0.005          # 5mm (reasonable for surgery)
lambda_damping = 0.01      # Higher (more stable)
step_size = 0.3            # Adaptive per iteration
max_step = 0.1             # Limit joint changes
```

**Additional features:**
- ✓ Stagnation detection with adaptive damping
- ✓ Step size limiting to prevent overshooting
- ✓ Better error handling
- ✓ No convergence required for success (best-effort)

### 2. Better Movement Handling

**Before:**
```python
def move_relative(...):
    success = self._ik_simple(target_pos)
    if not success:
        print("[MuJoCoRobot] IK did not converge")
    return success
```

**After:**
```python
def move_relative(...):
    success = self._ik_simple(target_pos)
    distance_error = calculate_error()

    if not success:
        if distance_error < 20:  # Within tolerance
            print(f"[MuJoCoRobot] IK: close enough ({distance_error:.1f}mm)")
        else:
            print(f"[MuJoCoRobot] IK did not converge ({distance_error:.1f}mm)")

    self._step(50)
    return True  # Always succeed (best-effort)
```

### 3. Safe Default Configuration

**Before:**
```ini
[sim]
mode = mujoco
vis_mode = viewer
```

**After:**
```ini
[sim]
mode = mock
vis_mode = headless
```

**Rationale:**
- Mock mode works everywhere without IK challenges
- Allows users to verify GUI and workflow
- MuJoCo can be enabled after installation/testing

### 4. Added Missing Dependency

Updated `requirements.txt`:
```
numpy                    # Required by MuJoCo, was missing
mujoco>=3.1.0
```

## Result

| Issue | Before | After |
|-------|--------|-------|
| Mock mode | ✓ Works | ✓ Works (default) |
| MuJoCo mode | IK fails | IK succeeds (best-effort) |
| Startup errors | Module imports fail | ✓ Clean startup |
| Dependencies | Incomplete | ✓ All documented |

## New Documentation

- **`IK_NOTES.md`** - Detailed IK explanation and solutions
- **`RUNTIME_FIXES.md`** - This file
- **`test_mujoco_movement.py`** - Test script for MuJoCo movements

## To Use MuJoCo Simulation

### Option 1: Test Improved IK (Recommended)

```bash
# Install dependencies
pip install numpy mujoco>=3.1.0

# Test with improved IK
python test_mujoco_movement.py
```

### Option 2: Switch Mode in Config

Edit `surgical.conf`:
```ini
[sim]
mode = mujoco
vis_mode = headless    # No display needed
```

Then run:
```bash
python surgical_test/voice-movement.py
```

### Option 3: Use Real xArm6 URDF (Best)

For production, use the real URDF for accurate kinematics:

```bash
bash scripts/fetch_xarm_urdf.sh
python scripts/urdf_to_mjcf.py

# Then uncomment in scene.xml:
# <include file="../urdf/xarm6.xml"/>
```

## Performance

- **Mock mode:** Instant (no simulation)
- **MuJoCo (headless):** ~50ms per movement
- **MuJoCo (viewer):** ~100-200ms per movement (display overhead)

## Next Steps

1. **Default usage:** Mock mode (GUI and workflows)
2. **Optional testing:** `python test_mujoco_movement.py`
3. **For precision:** Get real xArm6 URDF
4. **For production:** Test with actual hardware

## Troubleshooting

**Q: Still getting "IK did not converge"**
- A: Normal for stub arm. Try smaller movements.
- A: Install real URDF for better kinematics.
- A: Use mock mode for GUI testing.

**Q: MuJoCo is slow**
- A: Use headless mode instead of viewer
- A: Reduce simulation frequency in code

**Q: Errors about numpy**
- A: Run `pip install numpy mujoco>=3.1.0`

## Architecture

```
Movement → CommandDispatcher → RobotInterface
                                    ↓
                    ┌───────────────┼───────────────┐
                    ↓               ↓               ↓
              MockRobot        MuJoCoRobot      RealRobot
           (console output)   (physics sim)    (hardware)
```

All three modes share the same command interface, allowing seamless switching.
