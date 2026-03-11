# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is the **xArm Python SDK** — a Python library for controlling UFACTORY robotic arms (xArm 5/6/7, 850, Lite6) over TCP/IP or serial. The repo name "Surgical-Tool-Use" reflects a custom fork for surgical robotics use cases.

## Installation & Setup

```bash
# Install from source
pip install .

# Or build a wheel first
pip install build
python -m build
pip install dist/xarm_python_sdk-*.whl
```

No installation required to run examples — they add the repo root to `sys.path` directly.

## Running Examples

Before running any example, set the robot IP in `example/wrapper/robot.conf`:
```ini
[xArm]
ip = 192.168.1.113
```

Run an example:
```bash
python example/wrapper/common/0000-template.py
# Or pass IP as argument:
python example/wrapper/common/1001-move_line.py 192.168.1.113
```

There are no automated tests or linting configurations in this repo.

## Architecture

### Layer Structure

```
XArmAPI (xarm/wrapper/xarm_api.py)      ← Public API surface, thin wrapper
    └── XArm (xarm/x3/xarm.py)          ← Inherits from all capability mixins
            ├── Base (x3/base.py)        ← Connection management, state, reporting loop
            ├── Gripper (x3/gripper.py)
            ├── Servo (x3/servo.py)
            ├── Record (x3/record.py)
            ├── RobotIQ (x3/robotiq.py)
            ├── BaseBoard (x3/base_board.py)
            ├── LinearMotor (x3/linear_motor.py)
            ├── FtSensor (x3/ft_sensor.py)
            └── ModbusTcp (x3/modbus_tcp.py)
```

### Core Layer (`xarm/core/`)

- **`core/comm/`** — Low-level transport: `SocketPort` (TCP) and `SerialPort` (USB/serial)
- **`core/wrapper/`** — `UxbusCmdTcp` / `UxbusCmdSer`: encode/decode the binary UxBus protocol sent to the robot controller
- **`core/config/x_config.py`** — `XCONF` class with robot type constants, axis counts, hardware IDs
- **`core/config/x_code.py`** — Error/warning code enumerations

### Key Patterns

**Connection flow:**
```python
arm = XArmAPI('192.168.1.113', do_not_open=True)
arm.register_error_warn_changed_callback(handler)
arm.connect()
arm.motion_enable(enable=True)
arm.set_mode(0)   # 0=position, 1=servo, 2=joint-velocity, 3=cartesian-velocity
arm.set_state(0)  # 0=sport state
# ... commands ...
arm.disconnect()
```

**Decorators** (`x3/decorator.py`) guard methods: `@xarm_is_connected`, `@xarm_is_ready`, `@xarm_wait_until_cmdnum_lt_max` — these silently return error codes if preconditions aren't met.

**Return values:** Most methods return `(code, data)` tuples. `code == 0` means success. Non-zero codes map to `APIState` (`x3/code.py`) or controller error codes.

**Units:** Default angle unit is degrees (`is_radian=False`). Pass `is_radian=True` to `XArmAPI()` to use radians globally, or per-call with `is_radian=` parameter.

**Reporting thread:** `Base` starts a background thread that continuously reads state from the robot and fires registered callbacks (position, state, errors, temperature, etc.).

### Studio API (`xarm/wrapper/studio_api.py`, `xarm/x3/studio.py`)

Parallel API used by UFACTORY Studio software. Supports Blockly-to-Python conversion via `xarm/tools/blockly/`.

## API Rename History

Several APIs were renamed in v1.17.x. Key renames:
- `set_impedance*` → `set_ft_sensor_admittance_parameters`
- `ft_sensor_*` → `set_ft_sensor_*` / `get_ft_sensor_*`
- `get_linear_track_*` → `get_linear_motor_*`
- `getset_tgpio_modbus_data` → `set_rs485_data`
- `set_tgpio_modbus_baudrate` → `set_rs485_baudrate`
