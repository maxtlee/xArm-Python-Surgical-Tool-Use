xarm-python-sdk
===============

.. image:: https://badge.fury.io/py/xarm-python-sdk.svg
    :target: https://pypi.org/project/xarm-python-sdk/

.. image:: https://static.pepy.tech/badge/xarm-python-sdk
    :target: https://pepy.tech/projects/xarm-python-sdk

.. image:: https://img.shields.io/github/license/xArm-Developer/xArm-Python-SDK.svg
    :target: https://github.com/xArm-Developer/xArm-Python-SDK/blob/main/LICENSE

.. image:: https://img.shields.io/pypi/pyversions/xarm-python-sdk.svg
    :target: https://pypi.org/project/xarm-python-sdk/

Official Python SDK for UFACTORY robots.

Supported Products
------------------

- UFACTORY xArm 5/6/7
- UFACTORY 850
- UFACTORY Lite 6

Compatibility
-------------

- Python 3.5 - 3.13

Installation
------------

Install from PyPI:

.. code-block:: bash

    pip install xarm-python-sdk

Quick Start
-----------

.. code-block:: python

    from xarm.wrapper import XArmAPI

    # Connect to the robot
    arm = XArmAPI('192.168.1.100')  # Replace with your robot's IP address

    # Enable motion
    arm.motion_enable(enable=True)

    # Set robot to ready state
    arm.set_mode(0)    # Position control mode
    arm.set_state(0)   # Set to ready state

    # Move to a target position
    arm.set_position(x=300, y=0, z=200, roll=180, pitch=0, yaw=0, speed=100)

    # Disconnect
    arm.disconnect()

Surgical Tool Use Fork
----------------------

This fork extends the xArm SDK for voice-commanded surgical assistance with a tendon-driven soft foam hand for orthopedic hand surgery. The system supports three simulation modes:

**Mock Mode** (no dependencies)
  Zero-dependency simulation for CI and testing. All commands print to console.

**MuJoCo 3D Simulation** (recommended for development)
  Full 3D physics simulation with soft body dynamics for the foam hand and deformable tissue.

  **Unix/Linux/Mac:**

  .. code-block:: bash

      pip install mujoco mediapy
      bash scripts/fetch_xarm_urdf.sh
      python scripts/urdf_to_mjcf.py

  **Windows:**

  .. code-block:: bash

      pip install mujoco mediapy
      scripts\fetch_xarm_urdf.bat
      python scripts\urdf_to_mjcf.py

**Real Hardware Mode**
  Direct control of physical xArm6 and foam hand via Ethernet and USB serial.

To run the voice-control GUI:

.. code-block:: bash

    python surgical_test/voice-movement.py

Configure simulation mode in ``surgical.conf``. See the main README.md for complete setup instructions, voice commands, and workspace configuration.

Documentation
-------------

Full documentation and examples are available at:

https://github.com/xArm-Developer/xArm-Python-SDK

Website: https://www.ufactory.cc/

Release Note: https://github.com/xArm-Developer/xArm-Python-SDK/blob/master/README.md#update-summary

Bug Reports: support@ufactory.cc

License
-------

- License: BSD 3-Clause License. See `LICENSE <https://github.com/xArm-Developer/xArm-Python-SDK/blob/master/LICENSE>`_ for details.
