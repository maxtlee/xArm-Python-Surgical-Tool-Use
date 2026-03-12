"""
Robot simulation and control layer factory.

Supports three modes:
  - mock: Simple state tracking with console output (no dependencies)
  - mujoco: Full 3D physics simulation with soft body and contact (requires mujoco)
  - real: Physical xArm hardware control (requires xarm-python-sdk)
"""

import configparser
from typing import Optional
from .robot_interface import RobotInterface


def get_robot(config_path: str = 'surgical.conf') -> RobotInterface:
    """
    Factory function that returns the appropriate robot implementation
    based on the [sim] mode setting in the config file.

    Args:
        config_path: Path to surgical.conf

    Returns:
        A concrete RobotInterface implementation

    Raises:
        ValueError: If mode is invalid or required config is missing
        ImportError: If required dependencies for the chosen mode are not installed
    """
    config = configparser.ConfigParser()
    config.read(config_path)

    if 'sim' not in config:
        raise ValueError(f"Config file {config_path} missing [sim] section")

    mode = config['sim'].get('mode', 'mock').lower()

    if mode == 'mock':
        # Lazy import - no external dependencies needed
        from .mock_robot import MockRobot

        # Parse home position from config
        home_pose = None
        if 'pose' in config:
            try:
                home_pose = [
                    config['pose'].getfloat('home_x', 300.0),
                    config['pose'].getfloat('home_y', 0.0),
                    config['pose'].getfloat('home_z', 300.0),
                    config['pose'].getfloat('home_roll', 0.0),
                    config['pose'].getfloat('home_pitch', 0.0),
                    config['pose'].getfloat('home_yaw', 0.0),
                ]
            except (ValueError, KeyError):
                home_pose = None

        return MockRobot(home_pose=home_pose)

    elif mode == 'mujoco':
        # Lazy import - only import mujoco when needed
        from .mujoco_robot import MuJoCoRobot

        scene_path = config['sim'].get('scene', 'surgical_test/sim/scene.xml')
        vis_mode = config['sim'].get('vis_mode', 'viewer')

        # Parse default joint angles from config
        joint_angles = None
        if 'pose' in config:
            try:
                angles_str = config['pose'].get('joint_angles', '')
                if angles_str.strip():
                    joint_angles = [float(x.strip()) for x in angles_str.split(',')]
                    if len(joint_angles) != 6:
                        raise ValueError(f"Expected 6 joint angles, got {len(joint_angles)}")
            except (ValueError, KeyError):
                joint_angles = None

        return MuJoCoRobot(scene_path=scene_path, vis_mode=vis_mode, joint_angles=joint_angles)

    elif mode == 'real':
        # Lazy import - only import xarm when needed
        from .real_robot import RealRobot

        if 'arm' not in config:
            raise ValueError(f"Config file {config_path} missing [arm] section for real mode")
        if 'workspace' not in config:
            raise ValueError(f"Config file {config_path} missing [workspace] section for real mode")

        arm_config = {
            'ip': config['arm'].get('ip'),
            'speed_mm_s': config['arm'].getfloat('speed_mm_s', 40),
            'accel_mm_s2': config['arm'].getfloat('accel_mm_s2', 400),
        }

        workspace_config = {
            'x_min': config['workspace'].getfloat('x_min'),
            'x_max': config['workspace'].getfloat('x_max'),
            'y_min': config['workspace'].getfloat('y_min'),
            'y_max': config['workspace'].getfloat('y_max'),
            'z_min': config['workspace'].getfloat('z_min'),
            'z_max': config['workspace'].getfloat('z_max'),
        }

        hand_config = None
        if 'hand' in config:
            hand_config = {
                'port': config['hand'].get('port'),
                'baud': config['hand'].getint('baud', 9600),
            }

        return RealRobot(
            arm_config=arm_config,
            workspace_config=workspace_config,
            hand_config=hand_config
        )

    else:
        raise ValueError(f"Unknown simulation mode: '{mode}'. Must be 'mock', 'mujoco', or 'real'")
