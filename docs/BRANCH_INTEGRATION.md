# Branch Integration Analysis

Current branch: `sim`. This document tracks what the other remote branches have that `sim` does not, and what needs to be merged or ported.

---

## What sim has that other branches lack

- `surgical_test/sim/` — full MuJoCo physics simulation package (`__init__.py`, `mock_robot.py`, `mujoco_robot.py`, `real_robot.py`, `robot_interface.py`, `scene.xml`)
- `surgical_test/command_dispatcher.py` — command dispatch layer over `RobotInterface`
- `surgical.conf` — config file driving sim/mock/real mode selection
- `scripts/` — URDF fetch and smoke-test utilities
- `docs/` — this directory

All of the above are **absent** from `origin/grippertune`, `origin/chat`, `origin/deepgram`, and `origin/toolOffsetsDeepgram`.

---

## What other branches have that sim lacks

### 1. `surgical_test/voice-movement-v2.py` (origin/grippertune, origin/chat)

Variable step sizes with magnitude qualifiers ("a little" → 5 mm, default → 20 mm, "more" → 40 mm, "a lot" → 80 mm). Uses `faster-whisper`. Targets physical xArm directly (no sim layer).

**Action:** Add this file to sim. No conflict with existing files.

---

### 2. `surgical_test/voice-movement-deepgram.py` (origin/deepgram, origin/toolOffsetsDeepgram)

Replaces local Whisper with Deepgram streaming WebSocket (lower latency). Adds:
- `GripperController` class — DexHand/Dynamixel motor control loaded from `gripper_poses.yaml`
- Named positions: `pickup`, `engage`, `retract` (7-DOF joint angle sets)
- Tool offset support (in `toolOffsetsDeepgram`)
- Hardcoded Deepgram API key (`a5aca16e...`) — **must be moved to env var or config before use**

Targets physical xArm directly (no sim layer).

**Action:** Add this file to sim. Sanitize the API key.

---

### 3. Updated `surgical_test/voice-movement.py` (origin/chat)

The `chat` branch version of the existing `voice-movement.py` adds:
- OpenAI GPT-4o-mini for NLP command parsing (replaces regex matching)
- `GripperController` class + `gripper_poses.yaml` loading
- Named positions: `pickup`, `engage`, `retract`
- `gripper_open` / `gripper_close` actions
- `faster-whisper` for local transcription
- Requires `OPENAI_API_KEY` env var

The sim branch version uses Google Speech / local Whisper fallback with no gripper or LLM.

**Action:** The sim `voice-movement.py` runs against the `RobotInterface` abstraction layer. The chat version bypasses this and talks to `XArmAPI` directly. Two options:
- **Option A (recommended):** Port the gripper controller and named-pose commands into the sim version, keeping the `RobotInterface` abstraction. Add `go_to_pose(name)` to `RobotInterface` and implement in `RealRobot` / `MuJoCoRobot`.
- **Option B:** Add the chat `voice-movement.py` as a separate script (`voice-movement-chat.py`) for hardware-only use.

---

### 4. `surgical_test/gripper_poses.yaml` (origin/grippertune, origin/toolOffsetsDeepgram)

YAML config for DexHand motor positions (degrees → ticks). Defines `open` and `close` poses. Referenced by `GripperController` in both deepgram and chat voice scripts.

**Action:** Add to sim. No conflicts.

---

### 5. `surgical_test/cv/` — Computer vision (origin/deepgram, origin/chat)

ArUco marker detection + RealSense camera calibration pipeline:
- `cv_aruco_test.py` — verify ArUco detection
- `cv_calibration.py` / `cv_calibration_points.py` — eye-to-hand calibration → `T_cam_to_base.npy`
- `cv_move_to_spot.py` — move arm to detected incision spots
- `view_camera.py` / `view_realsense_color.py` / `test_depth.py` — camera utilities
- `make_aruco_markers.py` — generate marker images

Dependencies: `pyrealsense2`, `opencv-python`, `numpy`.

**Action:** Add the entire `surgical_test/cv/` directory to sim.

---

### 6. `surgical_test/grab/grab_retractor.py` (origin/deepgram)

ArUco-guided retractor handle approach. Uses the calibration from `cv/` to compute a grab target in robot base frame, then moves the arm to hover above and descend. Keyboard-controlled (`G` to grab, `H` home, `Q` quit).

Depends on: `pyrealsense2`, `cv2`, `T_cam_to_base.npy` from `cv/` calibration.

**Action:** Add `surgical_test/grab/` directory to sim. Depends on `cv/` being present first.

---

## sim-specific files that need updates relative to other branches

### `surgical_test/sim/real_robot.py`

The sim branch added this file; it doesn't exist on other branches. It needs to support the named pose positions (`pickup`, `engage`, `retract`) from the chat/deepgram branches. Currently `RobotInterface` has no `go_to_named_pose()` method.

### `surgical_test/sim/mujoco_robot.py` + `scene.xml`

These are **newer** in sim than in any other branch (xArm7 upgrade, corrected joint axes, end-effector-attached foam hand). No action needed — sim is ahead here.

---

## Priority order

1. [x] Add `surgical_test/gripper_poses.yaml` — git checkout origin/grippertune
2. [x] Add `surgical_test/voice-movement-v2.py` — git checkout origin/grippertune
3. [x] Add `surgical_test/voice-movement-deepgram.py` — git checkout origin/deepgram; API key sanitized to `os.environ.get("DEEPGRAM_API_KEY", "")`
4. [x] Add `surgical_test/cv/` directory — git checkout origin/deepgram
5. [x] Add `surgical_test/grab/` directory — git checkout origin/deepgram
6. [x] `go_to_named_pose(name)` added to `RobotInterface` (abstract), `MockRobot`, `MuJoCoRobot`, `RealRobot`
7. [x] `[named_poses]` section added to `surgical.conf`; `__init__.py` loads and passes to constructors; `command_dispatcher.py` handles `go_to_named_pose` command
8. [ ] Option A/B decision for GPT/gripper integration into `voice-movement.py` — deferred
