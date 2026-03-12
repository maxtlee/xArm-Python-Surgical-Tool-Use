# Pose Configuration Examples

Ready-to-use pose configurations for common surgical scenarios.

## Example 1: Default (Starting Position)

**Use case:** General testing, comfortable starting position

```ini
[pose]
home_x = 300.0
home_y = 0.0
home_z = 300.0
home_roll = 0.0
home_pitch = 0.0
home_yaw = 0.0
```

**Visual:** Arm extended forward and upward, pointing straight ahead

---

## Example 2: Surgical Field Access

**Use case:** Positioned over a surgical field (like on a patient's hand)

```ini
[pose]
home_x = 350.0      # Further forward into field
home_y = 0.0        # Centered
home_z = 150.0      # Low (near patient)
home_roll = 0.0
home_pitch = -45.0  # Tilted down toward patient
home_yaw = 0.0
```

**Visual:** Arm bent down toward the surgical site, ready to manipulate

---

## Example 3: Side Approach

**Use case:** Accessing the hand from the side (patient palm up)

```ini
[pose]
home_x = 200.0      # Slightly withdrawn
home_y = 200.0      # Offset to the right side
home_z = 250.0      # Medium height
home_roll = 0.0
home_pitch = -30.0  # Slightly tilted
home_yaw = 45.0     # Rotated to approach from side
```

**Visual:** Arm approaches from the right side at a shallow angle

---

## Example 4: High Reach

**Use case:** Starting high, then lowering into field

```ini
[pose]
home_x = 300.0
home_y = 0.0
home_z = 400.0      # High starting position
home_roll = 0.0
home_pitch = 0.0    # Level
home_yaw = 0.0
```

**Visual:** Arm fully extended upward, can move down easily

---

## Example 5: Retracted Safety Position

**Use case:** Between procedures, safe position away from patient

```ini
[pose]
home_x = 100.0      # Well back
home_y = 0.0
home_z = 500.0      # High and back
home_roll = 0.0
home_pitch = -80.0  # Pointing down (defensive)
home_yaw = 0.0
```

**Visual:** Arm pulled back and up, clearly out of the way

---

## Example 6: MuJoCo - Neutral Joints

**Use case:** Physics simulation with all joints at zero

```ini
[pose]
joint_angles = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
```

**Description:**
- Joint 1 (base): 0 rad (pointing forward)
- Joints 2-6: All at neutral angles

---

## Example 7: MuJoCo - Reaching Down

**Use case:** Physics simulation reaching toward surgical field

```ini
[pose]
joint_angles = 0.0, -0.5, 1.57, 0.0, 1.57, 0.0
```

**Description:**
- Joint 1: 0 rad (forward)
- Joint 2: -0.5 rad (tilted shoulder)
- Joint 3: 1.57 rad (90° elbow)
- Joint 4: 0 rad (neutral)
- Joint 5: 1.57 rad (wrist down)
- Joint 6: 0 rad (neutral)

---

## Example 8: MuJoCo - Horizontal Reach

**Use case:** Arm extended horizontally (ready for side approach)

```ini
[pose]
joint_angles = 0.0, -1.57, 0.0, 0.0, 0.0, 0.0
```

**Description:**
- Joint 1: 0 rad
- Joint 2: -1.57 rad (90° horizontal reach)
- Joints 3-6: 0 rad (neutral)

---

## Example 9: Precision Surgery - Centered

**Use case:** Fine motor control, centered over surgical site

```ini
[pose]
home_x = 320.0      # Slightly forward
home_y = 0.0        # Perfectly centered
home_z = 200.0      # Low for precision
home_roll = 0.0     # No tilt
home_pitch = -30.0  # Slight forward angle
home_yaw = 0.0      # Facing forward
```

**Visual:** Stable position for precise manipulations

---

## Example 10: Training Position

**Use case:** Teaching/training setup with good visibility

```ini
[pose]
home_x = 250.0      # Closer
home_y = -50.0      # Slight offset for visibility
home_z = 350.0      # Elevated for viewing
home_roll = 0.0
home_pitch = -20.0  # Gentle angle
home_yaw = 0.0
```

**Visual:** Arm positioned for clear demonstration

---

## How to Use These Examples

### 1. Copy the desired `[pose]` section

Choose the example that matches your use case.

### 2. Paste into `surgical.conf`

```ini
[sim]
mode = mock
vis_mode = headless

; ... other sections ...

[pose]
; (paste example here)
home_x = 300.0
home_y = 0.0
; ... etc ...
```

### 3. Test it

```bash
python -c "
from surgical_test.sim import get_robot
robot = get_robot('surgical.conf')
print(f'Home position: {robot.pose}')
"
```

### 4. Run the application

```bash
python surgical_test/voice-movement.py
```

---

## Customizing Examples

### Mix and Match

You can combine ideas from multiple examples:

```ini
[pose]
; Centered (Example 1) + Low (Example 2) + Safety distance
home_x = 280.0
home_y = 0.0
home_z = 180.0
home_roll = 0.0
home_pitch = -35.0
home_yaw = 0.0
```

### Fine-Tuning

Start with an example, then adjust:

```ini
[pose]
home_x = 300.0      ; Move forward if too far
home_y = -20.0      ; Move left if too far right
home_z = 280.0      ; Move down if too high
home_roll = 0.0
home_pitch = -25.0  ; Adjust tilt angle
home_yaw = 0.0
```

---

## Tips

- **Forward/Backward:** Adjust `home_x` in 50mm steps
- **Left/Right:** Adjust `home_y` (negative=left, positive=right)
- **Up/Down:** Adjust `home_z` in 50mm steps
- **Tilt:** Adjust `home_pitch` in 10-20° steps
- **Rotation:** Adjust `home_yaw` in 45° steps

---

## MuJoCo Joint Angle Ranges

For reference, typical ranges (in radians) for a 6-DOF arm:

```
Joint 1 (base):     -π to π          (360° rotation)
Joint 2 (shoulder): -π/2 to π/2      (±90°)
Joint 3 (elbow):    0 to π           (0° to 180°)
Joint 4 (wrist):    -π to π          (360° rotation)
Joint 5 (wrist):    -π/2 to π/2      (±90°)
Joint 6 (wrist):    -π to π          (360° rotation)
```

Convert degrees to radians: `radians = degrees * π / 180`

---

## Quick Reference

| Scenario | Config Entry | Value |
|----------|--------------|-------|
| Forward | home_x | 200-400 mm |
| Left | home_y | -200 to -50 mm |
| Right | home_y | 50 to 200 mm |
| Down | home_z | 100-200 mm |
| Up | home_z | 300-500 mm |
| Tilted down | home_pitch | -30 to -60° |
| Tilted up | home_pitch | 10 to 30° |
| Side approach | home_yaw | ±45 to ±90° |

---

## Testing Combinations

```bash
# Test position reachability (MuJoCo)
python -c "
from surgical_test.sim import get_robot
import numpy as np

robot = get_robot('surgical.conf')  # Set mode=mujoco first
target_mm = [320, 0, 200]  # Your desired position
target_m = np.array(target_mm) / 1000.0

success = robot._ik_simple(target_m)
if success:
    print('Position is reachable!')
else:
    print('Position may not be reachable')
"
```

These examples provide a solid starting point for your specific surgical workflow!
