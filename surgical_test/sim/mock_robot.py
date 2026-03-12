"""
Mock robot implementation - pure Python simulation with console output.

No external dependencies. Maintains internal state and prints actions.
Useful for testing, CI, and development without hardware or 3D simulation.
"""

from typing import List
from .robot_interface import RobotInterface


class MockRobot(RobotInterface):
    """Mock robot that tracks state and prints actions to console."""

    def __init__(self, home_pose=None):
        """
        Initialize mock robot with configurable home position.

        Args:
            home_pose: List of [x, y, z, roll, pitch, yaw] or None for defaults
                      (x, y, z in mm; angles in degrees)
        """
        if home_pose is None:
            # Default: 300mm forward, centered, 300mm up
            self.pose = [300.0, 0.0, 300.0, 0.0, 0.0, 0.0]
        else:
            self.pose = list(home_pose)
        self.home_pose = list(self.pose)  # Remember home position
        self.aperture = 50

    def move_relative(self, dx=0, dy=0, dz=0, droll=0, dpitch=0, dyaw=0) -> bool:
        """Update pose and print the new state."""
        self.pose[0] += dx
        self.pose[1] += dy
        self.pose[2] += dz
        self.pose[3] += droll
        self.pose[4] += dpitch
        self.pose[5] += dyaw

        # Print compact state line
        print(f"[MockRobot] pose -> {self.pose}")
        return True

    def go_home(self) -> bool:
        """Reset to home position."""
        self.pose = list(self.home_pose)
        print(f"[MockRobot] returning to home position")
        print(f"[MockRobot] pose -> {self.pose}")
        return True

    def stop(self):
        """Print stop message."""
        print("[MockRobot] STOP")

    def set_hand_aperture(self, percent: int):
        """Set aperture and print visual bar."""
        # Clamp to 0-100
        self.aperture = max(0, min(100, percent))

        # Create visual bar: 10 blocks total
        filled = int(self.aperture / 10)
        empty = 10 - filled
        bar = '#' * filled + '-' * empty

        print(f"[MockRobot] hand [{bar}] {self.aperture}%")

    def get_pose(self) -> List[float]:
        """Return current pose."""
        return self.pose.copy()

    def get_hand_aperture(self) -> int:
        """Return current aperture percentage."""
        return self.aperture
