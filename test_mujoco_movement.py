#!/usr/bin/env python3
"""
Test MuJoCo movement and IK solver.
Run with: python test_mujoco_movement.py
"""
import sys
import os
import configparser

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Temporarily switch to mujoco mode for this test
config = configparser.ConfigParser()
config.read('surgical.conf')
original_mode = config['sim']['mode']

# Switch to mujoco
config['sim']['mode'] = 'mujoco'
config['sim']['vis_mode'] = 'headless'

with open('surgical.conf.test', 'w') as f:
    config.write(f)

print("=" * 60)
print("Testing MuJoCo Movement and IK")
print("=" * 60)

try:
    from surgical_test.sim import get_robot
    from surgical_test.command_dispatcher import CommandDispatcher

    print("\n[1] Creating MuJoCo robot (headless)...")
    robot = get_robot('surgical.conf.test')
    print(f"    OK: {type(robot).__name__}")

    print("\n[2] Getting initial pose...")
    pose = robot.get_pose()
    print(f"    Position: ({pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f}) mm")
    print(f"    Rotation: ({pose[3]:.1f}, {pose[4]:.1f}, {pose[5]:.1f}) deg")

    print("\n[3] Testing small movements (easier IK targets)...")
    movements = [
        {'command': 'move_right', 'magnitude': 'tiny'},
        {'command': 'move_left', 'magnitude': 'tiny'},
        {'command': 'move_up', 'magnitude': 'tiny'},
        {'command': 'move_down', 'magnitude': 'tiny'},
        {'command': 'move_forward', 'magnitude': 'tiny'},
        {'command': 'move_backward', 'magnitude': 'tiny'},
    ]

    dispatcher = CommandDispatcher(robot=robot, config_path='surgical.conf.test')

    for i, cmd in enumerate(movements, 1):
        result = dispatcher.dispatch(cmd)
        pose = robot.get_pose()
        print(f"    [{i}] {cmd['command']:20} -> ({pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f})")

    print("\n[4] Testing hand commands...")
    dispatcher.dispatch({'command': 'grip'})
    dispatcher.dispatch({'command': 'hold_tighter'})
    dispatcher.dispatch({'command': 'hold_wider'})
    dispatcher.dispatch({'command': 'open_hand'})
    print("    OK: Hand commands executed")

    print("\n[5] Testing go_home...")
    robot.go_home()
    pose = robot.get_pose()
    print(f"    Home pose: ({pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f}) mm")

    print("\n" + "=" * 60)
    print("MuJoCo movement test complete!")
    print("=" * 60)

finally:
    # Clean up test config
    if os.path.exists('surgical.conf.test'):
        os.remove('surgical.conf.test')
