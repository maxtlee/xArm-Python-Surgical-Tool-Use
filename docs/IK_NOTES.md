# Inverse Kinematics (IK) Implementation Notes

## Current Status

The MuJoCo simulation includes an inverse kinematics solver for the 6-DOF stub arm. The solver is a damped least-squares Jacobian-based approach, which works but has limitations with the simplified arm model.

## Why IK Sometimes Doesn't Converge

### 1. Stub Arm Limitations
The `scene.xml` includes a **placeholder arm**, not the full xArm6. This stub has:
- Simplified geometry (capsules instead of real links)
- Limited workspace
- Potential singularities
- No real-world constraints from actual hardware

**Solution:** Replace the stub with the real xArm6 URDF:
```bash
# Download real URDF
bash scripts/fetch_xarm_urdf.sh    # Unix/Linux/Mac
scripts\fetch_xarm_urdf.bat        # Windows

# Convert to MuJoCo format
python scripts/urdf_to_mjcf.py

# Then include in scene.xml:
# <include file="../urdf/xarm6.xml"/>
```

### 2. Target Workspace Issues
Some movement commands may target positions outside the reachable workspace.

**Example:**
- Home position: (300mm, 0mm, 300mm)
- Move right by 300mm: → (300mm, 300mm, 300mm)
- If stub arm can't reach here, IK fails

**Solution:** Verify target is reachable or use workspace bounds checking.

### 3. IK Algorithm Limitations
The damped least-squares solver can struggle with:
- Singularities (arm fully extended or folded)
- Bad initial joint configuration
- Numerical instability
- Highly constrained workspaces

## Current Improvements

The IK solver now includes:

✓ **Adaptive step size** - Prevents overshooting
✓ **Higher damping** - Improves stability
✓ **Stagnation detection** - Restarts with better parameters
✓ **Lenient convergence** - 5mm tolerance instead of 0.1mm
✓ **Best-effort movement** - Does best approximation if exact solution not found

## Configuration Options

### Tuning IK Parameters

In `mujoco_robot.py`, adjust these in `_ik_simple()`:

```python
tolerance = 0.005           # Convergence threshold (meters).
                           # Increase for faster but less precise
lambda_damping = 0.01      # Stability parameter.
                           # Increase for stability, decrease for speed
step_size = 0.3            # Step size per iteration
max_step = 0.1             # Maximum joint change per iteration
max_iter = 100             # Maximum iterations before giving up
```

### Recommended Tuning

**For speed (CI/testing):**
```python
tolerance = 0.010      # 10mm OK
lambda_damping = 0.02  # More damping
max_iter = 50          # Quit early
```

**For accuracy (precision surgery):**
```python
tolerance = 0.002      # 2mm
lambda_damping = 0.005 # Less damping
max_iter = 200         # More iterations
```

## Better IK Solutions

### Option 1: Use Real xArm6 URDF ✓ RECOMMENDED

The official xArm6 URDF has:
- Real joint limits
- Proper kinematics
- Better workspace definition
- Singularity-free regions

**Steps:**
1. Run `bash scripts/fetch_xarm_urdf.sh`
2. Run `python scripts/urdf_to_mjcf.py`
3. Include in scene.xml
4. Update `IK_NDOF` in code if arm has 7 DOF

### Option 2: Analytical IK Solution

For xArm6 (6-DOF arm), an analytical IK exists:
- Much faster than numerical IK
- Always returns solution if reachable
- No convergence issues

**Implementation:**
```python
def _ik_analytical(self, target_pos, target_rot):
    """
    Analytical IK for xArm6 (6 DOF).
    Requires: xarm-python-sdk already in dependencies
    """
    # Could use: arm.get_inverse_kinematics(target_pose)
    # from xarm.wrapper import XArmAPI
    pass
```

### Option 3: Hybrid Approach

Use analytical for most moves, fall back to numerical for edge cases:
```python
def move_relative(self, dx, dy, dz, ...):
    # Try analytical IK first
    joints = self._ik_analytical(target_pos)
    if joints is not None:
        self.data.qpos[:6] = joints
    else:
        # Fall back to numerical
        self._ik_simple(target_pos)
```

### Option 4: Disable Complex Moves

For surgical applications, you might not need full Cartesian movements:

```python
def move_relative(self, dx=0, dy=0, dz=0, ...):
    """
    Simple movement without IK:
    - Just modify joint angles directly
    - Safer than attempting potentially unreachable targets
    """
    # Move by small joint increments instead of Cartesian
    # This is what the legacy voice commands did
```

## Performance Comparison

| Method | Speed | Accuracy | Implementation |
|--------|-------|----------|-----------------|
| Damped LS IK | Slow | Medium | ✓ Implemented |
| Analytical IK | Fast | High | Requires xArm SDK |
| Direct joints | Fast | Low | Simple modification |

## For Surgical Simulation

For actual surgical use cases, consider:

1. **Keep small movements** - IK is easier for small increments
2. **Use joint velocity control** - Skip IK entirely
3. **Use workspace constraints** - Prevent unreachable targets
4. **Test with real robot** - Validation is critical

## Testing IK

```python
from surgical_test.sim import get_robot

robot = get_robot('surgical.conf')  # Use mujoco mode
robot.move_relative(dz=-10)  # Small move (easier)
# vs
robot.move_relative(dz=-100)  # Large move (harder)
```

## Debugging IK Issues

1. **Check current position:**
   ```python
   pose = robot.get_pose()
   print(f"Current: {pose}")
   ```

2. **Check if target is reachable:**
   - xArm6 max reach: ~880mm
   - Check workspace bounds in surgical.conf

3. **Visualize with viewer mode:**
   ```ini
   [sim]
   mode = mujoco
   vis_mode = viewer  # See arm movement
   ```

4. **Enable verbose logging:**
   ```python
   # In move_relative, add:
   print(f"Target: {target_pos}")
   print(f"Result: {result_pose[:3]}")
   print(f"Error: {distance_error:.1f}mm")
   ```

## Next Steps

1. **Test current setup with mock mode** (default)
2. **Verify movements work** (no convergence needed)
3. **[Optional] Get real xArm6 URDF** for better simulation
4. **[Optional] Implement analytical IK** for production use
5. **[For deployment] Test with actual robot hardware**

## Resources

- MuJoCo IK: https://mujoco.readthedocs.io/en/stable/APIReference/APIPossDynamics.html#mj-jaceBody
- xArm6 Kinematics: https://github.com/xArm-Developer/xArm-Python-SDK
- Damped LS Method: https://www.roboticstoday.com/how-to-solve-inverse-kinematics-problem/
