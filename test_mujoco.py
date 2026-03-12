#!/usr/bin/env python3
"""
Test MuJoCo simulation initialization and basic functionality.
Run with: python test_mujoco.py
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

print("=" * 60)
print("Testing MuJoCo Simulation Setup")
print("=" * 60)

# Test 1: Check if MuJoCo is installed
print("\n[1] Checking MuJoCo installation...")
try:
    import mujoco
    print(f"    OK: MuJoCo {mujoco.__version__} is installed")
except ImportError as e:
    print(f"    FAILED: {e}")
    print("    Install with: pip install mujoco>=3.1.0")
    sys.exit(1)

# Test 2: Check if viewer is available
print("\n[2] Checking MuJoCo viewer...")
try:
    from mujoco import viewer
    print("    OK: MuJoCo viewer module is available")
except (ImportError, AttributeError) as e:
    print(f"    WARNING: Viewer not available: {e}")
    print("    Install with: pip install mujoco[viewer]")

# Test 3: Check numpy
print("\n[3] Checking NumPy...")
try:
    import numpy as np
    print(f"    OK: NumPy {np.__version__} is installed")
except ImportError as e:
    print(f"    FAILED: {e}")
    print("    Install with: pip install numpy")
    sys.exit(1)

# Test 4: Check scene file exists
print("\n[4] Checking scene.xml file...")
scene_path = "surgical_test/sim/scene.xml"
if os.path.exists(scene_path):
    print(f"    OK: Found {scene_path}")
else:
    print(f"    FAILED: {scene_path} not found")
    print(f"    Current directory: {os.getcwd()}")
    sys.exit(1)

# Test 5: Try to load the scene
print("\n[5] Loading MuJoCo scene...")
try:
    model = mujoco.MjModel.from_xml_path(scene_path)
    print(f"    OK: Scene loaded")
    print(f"       Bodies: {model.nbody}")
    print(f"       Joints: {model.njnt}")
    print(f"       Geoms: {model.ngeom}")
except Exception as e:
    print(f"    FAILED: {e}")
    sys.exit(1)

# Test 6: Try to create data object
print("\n[6] Creating MuJoCo data object...")
try:
    data = mujoco.MjData(model)
    print(f"    OK: Data object created")
except Exception as e:
    print(f"    FAILED: {e}")
    sys.exit(1)

# Test 7: Try to find end-effector
print("\n[7] Finding end-effector 'link_eef'...")
try:
    ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link_eef")
    print(f"    OK: Found link_eef at body ID {ee_id}")
except KeyError as e:
    print(f"    FAILED: {e}")
    sys.exit(1)

# Test 8: Try to import the robot class
print("\n[8] Importing MuJoCoRobot class...")
try:
    from surgical_test.sim.mujoco_robot import MuJoCoRobot
    print(f"    OK: MuJoCoRobot class imported")
except Exception as e:
    print(f"    FAILED: {e}")
    sys.exit(1)

# Test 9: Try to instantiate with headless mode (no viewer needed)
print("\n[9] Creating MuJoCoRobot instance (headless mode)...")
try:
    robot = MuJoCoRobot(scene_path=scene_path, vis_mode='headless')
    print(f"    OK: MuJoCoRobot created successfully")
except Exception as e:
    print(f"    FAILED: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Test 10: Try basic operations
print("\n[10] Testing basic operations...")
try:
    pose = robot.get_pose()
    print(f"    OK: get_pose() = {pose}")

    success = robot.move_relative(dx=10, dy=0, dz=0)
    print(f"    OK: move_relative() = {success}")

    robot.set_hand_aperture(50)
    print(f"    OK: set_hand_aperture(50)")

    robot.go_home()
    print(f"    OK: go_home()")

except Exception as e:
    print(f"    FAILED: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("\n" + "=" * 60)
print("All tests passed! MuJoCo simulation is working correctly.")
print("=" * 60)
