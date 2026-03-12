#!/usr/bin/env bash
# Downloads the xArm6 URDF from the official xArm ROS package.
# Does NOT require ROS to be installed.
# Usage: bash scripts/fetch_xarm_urdf.sh

set -e
DEST="urdf"
mkdir -p "$DEST"

echo "Cloning xarm_ros (shallow)..."
git clone --depth 1 \
  https://github.com/xArm-Developer/xarm_ros.git /tmp/xarm_ros_tmp

echo "Copying URDF and meshes..."
cp /tmp/xarm_ros_tmp/xarm_description/urdf/xarm6_robot.urdf.xacro "$DEST/"
cp -r /tmp/xarm_ros_tmp/xarm_description/meshes "$DEST/"

echo "Processing xacro → plain URDF (requires: pip install xacro)..."
if command -v xacro &> /dev/null; then
  xacro "$DEST/xarm6_robot.urdf.xacro" > "$DEST/xarm6.urdf"
  echo "Written: $DEST/xarm6.urdf"
else
  echo "xacro not found — install with: pip install xacro"
  echo "Then run: xacro $DEST/xarm6_robot.urdf.xacro > $DEST/xarm6.urdf"
fi

rm -rf /tmp/xarm_ros_tmp
echo "Done."
