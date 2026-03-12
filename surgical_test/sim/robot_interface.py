"""
Abstract robot interface for the surgical robot system.

Defines the common API that all robot implementations (mock, simulation, real)
must provide.
"""

from abc import ABC, abstractmethod
from typing import List


class RobotInterface(ABC):
    """Abstract base class for robot control."""

    @abstractmethod
    def move_relative(self, dx=0, dy=0, dz=0, droll=0, dpitch=0, dyaw=0) -> bool:
        """
        Move the end-effector relative to current pose.

        Args:
            dx, dy, dz: Translation in mm
            droll, dpitch, dyaw: Rotation in degrees

        Returns:
            True if move succeeded, False otherwise
        """
        pass

    @abstractmethod
    def go_home(self) -> bool:
        """Return to home position.

        Returns:
            True if successful, False otherwise
        """
        pass

    @abstractmethod
    def stop(self):
        """Emergency stop - halt all motion immediately."""
        pass

    @abstractmethod
    def set_hand_aperture(self, percent: int):
        """
        Set gripper/hand opening.

        Args:
            percent: 0 (closed) to 100 (fully open)
        """
        pass

    @abstractmethod
    def get_pose(self) -> List[float]:
        """
        Get current end-effector pose.

        Returns:
            [x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg]
        """
        pass

    @abstractmethod
    def get_hand_aperture(self) -> int:
        """
        Get current gripper/hand opening.

        Returns:
            Aperture as percentage (0-100)
        """
        pass
