@echo off
REM Downloads the xArm6 URDF from the official xArm ROS package.
REM Does NOT require ROS to be installed.
REM Usage: scripts\fetch_xarm_urdf.bat

setlocal enabledelayedexpansion

set "DEST=urdf"
if not exist "%DEST%" mkdir "%DEST%"

echo Cloning xarm_ros (shallow)...
git clone --depth 1 https://github.com/xArm-Developer/xarm_ros.git %TEMP%\xarm_ros_tmp

echo Copying URDF and meshes...
copy "%TEMP%\xarm_ros_tmp\xarm_description\urdf\xarm6_robot.urdf.xacro" "%DEST%\" >nul
xcopy "%TEMP%\xarm_ros_tmp\xarm_description\meshes" "%DEST%\meshes\" /E /I /Y >nul

echo Processing xacro to plain URDF (requires: pip install xacro)...
where xacro >nul 2>&1
if %errorlevel% equ 0 (
    xacro "%DEST%\xarm6_robot.urdf.xacro" > "%DEST%\xarm6.urdf"
    echo Written: %DEST%\xarm6.urdf
) else (
    echo xacro not found -- install with: pip install xacro
    echo Then run: xacro %DEST%\xarm6_robot.urdf.xacro ^> %DEST%\xarm6.urdf
)

echo Cleaning up...
rmdir /S /Q "%TEMP%\xarm_ros_tmp" 2>nul
echo Done.
