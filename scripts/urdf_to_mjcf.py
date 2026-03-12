"""
Converts the xArm6 URDF to a MuJoCo-compatible MJCF file.
MuJoCo can load URDFs directly, but an explicit MJCF gives more control.
Usage: python scripts/urdf_to_mjcf.py
"""
import sys, os
try:
    import mujoco
except ImportError:
    sys.exit("Run: pip install mujoco>=3.1.0")

urdf_path = os.path.join("urdf", "xarm6.urdf")
mjcf_path = os.path.join("urdf", "xarm6.xml")

if not os.path.exists(urdf_path):
    sys.exit(f"Not found: {urdf_path}\nRun: bash scripts/fetch_xarm_urdf.sh first")

model = mujoco.MjModel.from_xml_path(urdf_path)
mujoco.mj_saveLastXML(mjcf_path, model)
print(f"Saved: {mjcf_path}  ({model.nbody} bodies, {model.njnt} joints)")
