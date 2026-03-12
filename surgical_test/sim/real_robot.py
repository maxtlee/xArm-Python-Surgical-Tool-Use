"""
Real robot layer - interfaces with physical xArm hardware.

Requires:
  - xarm-python-sdk
  - pyserial (for foam hand control)
"""

from typing import List, Dict, Optional
from .robot_interface import RobotInterface

try:
    from xarm.wrapper import XArmAPI
except ImportError:
    raise ImportError("xarm-python-sdk not installed. Run: pip install xarm-python-sdk")


class RealRobot(RobotInterface):
    """Real xArm hardware interface with workspace safety checks."""

    def __init__(
        self,
        arm_config: Dict,
        workspace_config: Dict,
        hand_config: Optional[Dict] = None
    ):
        """
        Initialize real robot connection.

        Args:
            arm_config: Dict with 'ip', 'speed_mm_s', 'accel_mm_s2'
            workspace_config: Dict with 'x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max'
            hand_config: Optional dict with 'port', 'baud' for serial hand control
        """
        self.workspace = workspace_config
        self.speed = arm_config['speed_mm_s']
        self.accel = arm_config['accel_mm_s2']

        # Connect to xArm
        print(f"[RealRobot] Connecting to xArm at {arm_config['ip']}...")
        self.arm = XArmAPI(arm_config['ip'])

        # Initialize arm
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)  # Position control mode
        self.arm.set_state(0)  # Sport state
        self.arm.clean_error()

        print("[RealRobot] xArm initialized")

        # Initialize serial connection for foam hand (optional)
        self.ser = None
        self.aperture = 50

        if hand_config:
            try:
                import serial
                self.ser = serial.Serial(
                    port=hand_config['port'],
                    baudrate=hand_config['baud'],
                    timeout=1
                )
                print(f"[RealRobot] Hand connected on {hand_config['port']}")
            except ImportError:
                print("[RealRobot] WARNING: pyserial not installed, hand control disabled")
                print("Run: pip install pyserial")
            except Exception as e:
                print(f"[RealRobot] WARNING: Could not open {hand_config['port']}: {e}")
                print("Arm will work without hand control")

    def _check_workspace(self, target_pose: List[float]) -> bool:
        """
        Check if target pose is within workspace limits.

        Args:
            target_pose: [x, y, z, roll, pitch, yaw] in mm and degrees

        Returns:
            True if safe, False otherwise
        """
        x, y, z = target_pose[:3]

        if not (self.workspace['x_min'] <= x <= self.workspace['x_max']):
            print(f"[RealRobot] WARNING: X={x:.1f} outside workspace [{self.workspace['x_min']}, {self.workspace['x_max']}]")
            return False

        if not (self.workspace['y_min'] <= y <= self.workspace['y_max']):
            print(f"[RealRobot] WARNING: Y={y:.1f} outside workspace [{self.workspace['y_min']}, {self.workspace['y_max']}]")
            return False

        if not (self.workspace['z_min'] <= z <= self.workspace['z_max']):
            print(f"[RealRobot] WARNING: Z={z:.1f} outside workspace [{self.workspace['z_min']}, {self.workspace['z_max']}]")
            return False

        return True

    def get_pose(self) -> List[float]:
        """Get current end-effector pose."""
        code, pose = self.arm.get_position()

        if code != 0:
            print(f"[RealRobot] Error getting position: code={code}")
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # pose is [x, y, z, roll, pitch, yaw] in mm and degrees
        return pose[:6]

    def move_relative(self, dx=0, dy=0, dz=0, droll=0, dpitch=0, dyaw=0) -> bool:
        """Move end-effector relative to current pose."""
        current_pose = self.get_pose()

        # Compute target pose
        target_pose = [
            current_pose[0] + dx,
            current_pose[1] + dy,
            current_pose[2] + dz,
            current_pose[3] + droll,
            current_pose[4] + dpitch,
            current_pose[5] + dyaw,
        ]

        # Workspace safety check
        if not self._check_workspace(target_pose):
            return False

        # Execute move
        code = self.arm.set_position(
            *target_pose,
            speed=self.speed,
            mvacc=self.accel,
            wait=True
        )

        if code != 0:
            print(f"[RealRobot] Move failed with code={code}")
            return False

        return True

    def go_home(self) -> bool:
        """Return to home position."""
        print("[RealRobot] Going home...")
        code = self.arm.move_gohome(speed=30, mvacc=300, wait=True)

        if code != 0:
            print(f"[RealRobot] Go home failed with code={code}")
            return False

        print("[RealRobot] Reached home position")
        return True

    def stop(self):
        """Emergency stop."""
        print("[RealRobot] EMERGENCY STOP")
        self.arm.emergency_stop()

    def set_hand_aperture(self, percent: int):
        """Set hand aperture via serial."""
        self.aperture = max(0, min(100, percent))

        if self.ser is not None:
            try:
                cmd = f"APERTURE:{self.aperture}\n"
                self.ser.write(cmd.encode('utf-8'))
                print(f"[RealRobot] Sent to hand: {cmd.strip()}")
            except Exception as e:
                print(f"[RealRobot] Failed to send to hand: {e}")
        else:
            print(f"[RealRobot] Hand aperture -> {self.aperture}% (no serial connection)")

    def get_hand_aperture(self) -> int:
        """Get current hand aperture."""
        return self.aperture

    def get_contact_forces(self) -> List:
        """Contact forces not available on real hardware."""
        print("[RealRobot] Contact force sensing not available on this hardware")
        return []

    def __del__(self):
        """Cleanup on destruction."""
        if hasattr(self, 'arm'):
            self.arm.disconnect()
        if hasattr(self, 'ser') and self.ser is not None:
            self.ser.close()
