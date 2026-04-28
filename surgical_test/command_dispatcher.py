"""
Command dispatcher - translates high-level voice commands to robot actions.

Takes JSON commands from the LLM and dispatches them to a RobotInterface.
"""

import configparser
from typing import Optional, Dict, Any
from surgical_test.sim import get_robot, RobotInterface


class CommandDispatcher:
    """Dispatches high-level commands to robot interface."""

    def __init__(self, robot: Optional[RobotInterface] = None, config_path: str = 'surgical.conf'):
        """
        Initialize dispatcher.

        Args:
            robot: Robot instance to control. If None, creates one via get_robot()
            config_path: Path to surgical.conf for reading step sizes
        """
        self.robot = robot if robot is not None else get_robot(config_path)

        # Load step size configuration
        config = configparser.ConfigParser()
        config.read(config_path)

        self.step_sizes = {
            'tiny': 2,
            'small': 5,
            'slight': 5,
            'medium': 10,
            'large': 20,
        }

        # Override with config if present
        if 'steps' in config:
            for key in self.step_sizes:
                if key in config['steps']:
                    self.step_sizes[key] = config['steps'].getint(key)

        self.default_magnitude = 5

    def _get_magnitude(self, magnitude_str: Optional[str]) -> float:
        """
        Convert magnitude string to numeric value.

        Args:
            magnitude_str: String like 'small', 'medium', etc.

        Returns:
            Numeric step size in mm or degrees
        """
        if not magnitude_str:
            return self.default_magnitude

        magnitude_lower = magnitude_str.lower()
        return self.step_sizes.get(magnitude_lower, self.default_magnitude)

    def dispatch(self, cmd: Dict[str, Any]) -> bool:
        """
        Dispatch a command to the robot.

        Args:
            cmd: Command dict with keys 'command' and optionally 'magnitude'
                 Example: {"command": "move_left", "magnitude": "small"}

        Returns:
            True if command executed successfully, False otherwise
        """
        command = cmd.get('command', '').lower()
        magnitude = self._get_magnitude(cmd.get('magnitude'))

        # Movement commands
        if command == 'move_left':
            return self.robot.move_relative(dy=-magnitude)

        elif command == 'move_right':
            return self.robot.move_relative(dy=magnitude)

        elif command == 'move_forward':
            return self.robot.move_relative(dx=magnitude)

        elif command == 'move_backward':
            return self.robot.move_relative(dx=-magnitude)

        elif command == 'move_up':
            return self.robot.move_relative(dz=magnitude)

        elif command == 'move_down':
            return self.robot.move_relative(dz=-magnitude)

        # Rotation commands
        elif command == 'tilt_left':
            return self.robot.move_relative(dyaw=-magnitude)

        elif command == 'tilt_right':
            return self.robot.move_relative(dyaw=magnitude)

        # Hand aperture commands - absolute
        elif command == 'open_hand':
            self.robot.set_hand_aperture(100)
            return True

        elif command == 'close_hand':
            self.robot.set_hand_aperture(0)
            return True

        elif command == 'grip':
            self.robot.set_hand_aperture(30)
            return True

        # Hand aperture commands - relative
        elif command == 'hold_wider':
            current = self.robot.get_hand_aperture()
            self.robot.set_hand_aperture(current + 10)
            return True

        elif command == 'hold_tighter':
            current = self.robot.get_hand_aperture()
            self.robot.set_hand_aperture(current - 10)
            return True

        # Control commands
        elif command == 'stop':
            self.robot.stop()
            return True

        elif command == 'go_home':
            return self.robot.go_home()

        elif command in ('go_to_named_pose', 'named_pose'):
            pose_name = cmd.get('pose', '')
            if not pose_name:
                print(f"[CommandDispatcher] WARNING: go_to_named_pose missing 'pose' key")
                return False
            return self.robot.go_to_named_pose(pose_name)

        # Unknown command
        else:
            print(f"[CommandDispatcher] WARNING: Unknown command '{command}'")
            return False
