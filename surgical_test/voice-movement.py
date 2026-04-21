#!/usr/bin/env python3
"""
Voice-Controlled xArm — 6-DOF Cartesian
========================================
Speak simple directional commands to move the end-effector.

Step sizes:
  "a little" / "slightly" / "small"  →   5 mm
  (no qualifier)                      →  20 mm
  "more" / "further" / "a bit more"  →  40 mm
  "a lot" / "much" / "way"           →  80 mm

Translation step sizes:
  "a little" / "slightly" / "small"  →   5 mm
  (no qualifier)                      →  20 mm
  "more" / "further"                  →  40 mm
  "a lot" / "much" / "way"           →  80 mm

Rotation step sizes:
  "a little" / "slightly" / "small"  →   2°
  (no qualifier)                      →  10°
  "more" / "further"                  →  20°
  "a lot" / "much" / "way"           →  45°

Voice commands:
  "forward [qualifier]"        → +X
  "backward [qualifier]"       → -X
  "left [qualifier]"           → +Y
  "right [qualifier]"          → -Y
  "up [qualifier]"             → +Z
  "down [qualifier]"           → -Z
  "roll left/right [qualifier]"  → ±Roll
  "pitch up/down [qualifier]"    → ±Pitch
  "yaw left/right [qualifier]"   → ±Yaw
  "go home"                    → return to home position
  "engage"                     → move to engage position
  "extend"                     → move to extend position
  "retract"                    → move to retract position
  "stop"                       → emergency stop
  "open"                       → open DexHand gripper
  "close"                      → close DexHand gripper

Dependencies:
  pip install SpeechRecognition pyaudio openai-whisper dynamixel-sdk
"""

import io
import os
import re
import sys
import queue
import threading
import tkinter as tk
from tkinter import scrolledtext, font as tkfont
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

try:
    import speech_recognition as sr
except ImportError:
    print("ERROR: SpeechRecognition not installed. Run: pip install SpeechRecognition pyaudio")
    sys.exit(1)

from xarm.wrapper import XArmAPI

try:
    from dynamixel_sdk import PortHandler, PacketHandler
    _DYNAMIXEL_AVAILABLE = True
except ImportError:
    _DYNAMIXEL_AVAILABLE = False

import yaml


# ---------------------------------------------------------------------------
# Gripper (DexHand via Dynamixel)
# ---------------------------------------------------------------------------

_POSES_FILE = os.path.join(os.path.dirname(__file__), "gripper_poses.yaml")

# Motor name → Dynamixel ID (matches NIBIB1.py Config.MOTOR_NAMES order)
_MOTOR_IDS = {
    "PinkyFlexion":   0,
    "RingFlexion":    1,
    "MiddleFlexion":  2,
    "IndexFlexion":   3,
    "ThumbAdduction": 4,
    "ThumbFlexion":   5,
}

_TICKS_PER_REV = 4096   # matches NIBIB1.py Config.TICKS_PER_REV

_ADDR_TORQUE_ENABLE    = 64
_ADDR_PROFILE_VELOCITY = 112
_ADDR_GOAL_POSITION    = 116
_ADDR_PRESENT_POSITION = 132


def _degrees_to_ticks(degrees):
    """Identical to NIBIB1.py Utility.degrees_to_ticks()."""
    return int(degrees * _TICKS_PER_REV / 360)


def _ticks_to_degrees(ticks):
    """Identical to NIBIB1.py Utility.ticks_to_degrees()."""
    return float(ticks * 360 / _TICKS_PER_REV)


def _load_gripper_config(path=_POSES_FILE):
    """Load gripper_poses.yaml and return (config_dict, poses_dict)."""
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    return data.get("config", {}), data.get("poses", {})


def _pose_to_tick_list(pose_data):
    """Convert a pose's motor_degrees mapping to an ordered tick list [ID0..ID5].

    Matches NIBIB1.py HandDemoWindow.run_sequence():
      ticks = [Utility.degrees_to_ticks(val) for val in pose_values]
    """
    motor_degrees = pose_data.get("motor_degrees", {})
    return [
        _degrees_to_ticks(float(motor_degrees.get(name, 0.0)))
        for name, _ in sorted(_MOTOR_IDS.items(), key=lambda kv: kv[1])
    ]


class GripperController:
    """Controls the DexHand via Dynamixel.

    Mirrors NIBIB1.py DynamixelController: degrees in YAML → ticks at runtime,
    home offsets recorded at init, per-motor profile velocity.
    """

    def __init__(self, poses_file=_POSES_FILE):
        if not _DYNAMIXEL_AVAILABLE:
            raise RuntimeError("dynamixel_sdk not installed — run: pip install dynamixel-sdk")

        cfg, poses = _load_gripper_config(poses_file)

        port      = cfg.get("port", "/dev/ttyACM0")
        baud      = int(cfg.get("baud_rate", 2_000_000))
        protocol  = float(cfg.get("protocol_version", 2.0))
        prof_vel  = int(cfg.get("profile_velocity", 100))

        self._open_ticks  = _pose_to_tick_list(poses.get("open",  {}))
        self._close_ticks = _pose_to_tick_list(poses.get("close", {}))

        self._ph = PortHandler(port)
        if not self._ph.openPort():
            raise RuntimeError(f"Failed to open gripper port {port}")
        if not self._ph.setBaudRate(baud):
            raise RuntimeError("Failed to set gripper baud rate")
        self._pkt = PacketHandler(protocol)

        # Enable torque and set profile velocity per motor individually,
        # matching NIBIB1.py DynamixelController.__init__()
        for dxl_id in _MOTOR_IDS.values():
            self._pkt.write1ByteTxRx(self._ph, dxl_id, _ADDR_TORQUE_ENABLE, 1)
            self._pkt.write4ByteTxRx(self._ph, dxl_id, _ADDR_PROFILE_VELOCITY, prof_vel)

        # Record present position of each motor as home offset,
        # matching NIBIB1.py HandPoseControllerWindow.__init__():
        #   self.home_offsets = self.ctrl.get_all_positions()
        self.home_offsets = self.get_all_positions()

    # -- Position readback (mirrors DynamixelController.get_position) -------

    def get_position(self, dxl_id):
        """Read present position in ticks, handling two's-complement rollover."""
        present, _, _ = self._pkt.read4ByteTxRx(
            self._ph, dxl_id, _ADDR_PRESENT_POSITION
        )
        return present - 0x100000000 if present & 0x80000000 else present

    def get_all_positions(self):
        return [self.get_position(dxl_id) for _, dxl_id in
                sorted(_MOTOR_IDS.items(), key=lambda kv: kv[1])]

    # -- Position commands (mirrors DynamixelController.set_position) --------

    def _set_positions(self, tick_list):
        """Send absolute tick positions to each motor individually,
        matching NIBIB1.py DynamixelController.set_all_positions()."""
        for _, dxl_id in sorted(_MOTOR_IDS.items(), key=lambda kv: kv[1]):
            ticks = tick_list[dxl_id]
            self._pkt.write4ByteTxRx(
                self._ph, dxl_id, _ADDR_GOAL_POSITION, ticks & 0xFFFFFFFF
            )

    def open(self):
        self._set_positions(self._open_ticks)

    def close(self):
        self._set_positions(self._close_ticks)

    def disconnect(self):
        """Disable torque and close port, matching DynamixelController.close()."""
        for dxl_id in _MOTOR_IDS.values():
            self._pkt.write1ByteTxRx(self._ph, dxl_id, _ADDR_TORQUE_ENABLE, 0)
        self._ph.closePort()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

SPEED = 50  # mm/s

# Hardcoded joint-angle positions [j1..j7] (degrees).  Edit these to match
# your robot's actual setpoints.
JOINT_POSITIONS = {
    'home':    [25, -71, 318, 45, -47, 99, -6],
    'extend':  [118, -99, 267, 70, -195, 92, 3.5], 
    'engage':  [116, -115, 267, 71, -195, 92, 3.5],
    'retract':  [109, -114, 270, 73, -156, 90, 16]
}

# TCP offsets per tool: [x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg]
# Measured from the 7th-joint flange centre to the tool tip.
# All values are placeholders — calibrate for each physical tool.
TOOL_OFFSETS = {
    'None (flange)':  [  0,   0,   0,   0,  0,  0],
    'big curvy one':       [  180, -150,  0,   0,  0,  0],
    'thin pointy one':       [  180, -150,  0,   0,  0,  0], # only tuned for thin pointy one
    'army navy one':       [  180, -150,  0,   0,  0,  0],
}

# RMS amplitude below which audio is considered silence and skipped (0–1 scale).
# Typical speech RMS is 0.05–0.2; background noise is usually under 0.01.
LOUDNESS_THRESHOLD = 0.05

# Step sizes for each magnitude qualifier (mm)
STEP = {
    'little':  5,
    'normal': 20,
    'more':   40,
    'lot':    80,
}

# Words that trigger each magnitude level
LITTLE_WORDS  = {'little', 'slightly', 'small', 'tiny', 'bit'}
MORE_WORDS    = {'more', 'further', 'farther', 'extra'}
LOT_WORDS     = {'lot', 'much', 'way', 'far', 'big', 'large', 'huge'}

# Maps direction keyword → unit vector (dx, dy, dz) and display label
DIRECTION_MAP = {
    'forward':  ( 1,  0,  0, 'Forward  +X'),
    'backward': (-1,  0,  0, 'Backward −X'),
    'back':     (-1,  0,  0, 'Backward −X'),
    'left':     ( 0,  1,  0, 'Left     +Y'),
    'right':    ( 0, -1,  0, 'Right    −Y'),
    'up':       ( 0,  0,  1, 'Up       +Z'),
    'down':     ( 0,  0, -1, 'Down     −Z'),
}

# Step sizes for rotation commands (degrees)
ROT_STEP = {
    'little':  2,
    'normal': 10,
    'more':   20,
    'lot':    45,
}

# Maps rotation axis keyword → (axis_index, positive_words, negative_words,
#                                positive_label, negative_label)
# Checked before DIRECTION_MAP so "yaw left" / "roll right" don't fall through
# to the translational "left" / "right" handlers.
ROTATION_MAP = {
    'roll':  (0, {'left',  'counterclockwise', 'ccw'},
                 {'right', 'clockwise',        'cw'},
                 'Roll Left   +Roll', 'Roll Right  −Roll'),
    'pitch': (1, {'up',    'forward'},
                 {'down',  'backward',         'back'},
                 'Pitch Up    +Pitch', 'Pitch Down  −Pitch'),
    'yaw':   (2, {'left',  'counterclockwise', 'ccw'},
                 {'right', 'clockwise',        'cw'},
                 'Yaw Left    +Yaw',  'Yaw Right   −Yaw'),
}


# ---------------------------------------------------------------------------
# Preset-position state machine
# ---------------------------------------------------------------------------
#
# Sparse directed graph of allowed transitions into each preset position.
# Edges are (source_state, joint_speed_deg_per_s): each entry under a target
# key lists the legal source states from which we may enter that target and
# the joint-space speed (°/s, passed to set_servo_angle) used for that move.
# Every edge can carry a distinct speed; tune per-motion safety as needed.
#
# A transition is legal when the current state matches one of the target's
# source states. Override by proximity is permitted when Cartesian TCP
# distance to the target is within POSITION_RADII[target] mm AND every joint
# is within JOINT_TOLERANCE° of the target joint angles. Manual Cartesian
# moves/rotations clear the state.

STATE_TRANSITIONS = {
    'home':    {('extend',  60)},
    'extend':  {('engage',  10), ('home',    60)},
    'engage':  {('extend',  10), ('retract', 10)},
    'retract': {('engage',  10)},
}


def _allowed_sources(target):
    """Return the set of source-state names that may enter `target`."""
    return {src for src, _ in STATE_TRANSITIONS.get(target, set())}


def _transition_speed(target, source):
    """Joint-speed (°/s) for source→target, or None if the edge is absent."""
    for src, speed in STATE_TRANSITIONS.get(target, set()):
        if src == source:
            return speed
    return None


def _override_speed(target):
    """Fallback speed for a proximity-override entry into `target`.

    Uses the slowest incoming edge speed — overrides happen from unknown
    source states, so we default to the safest legal approach."""
    speeds = [s for _, s in STATE_TRANSITIONS.get(target, set())]
    return min(speeds) if speeds else 20

# Cartesian radius (mm) within which override is permitted per target preset.
POSITION_RADII = {
    'home':    500.0,
    'extend':   50.0,
    'engage':   50.0,
    'retract':  50.0,
}

# Per-joint tolerance (degrees) required for override in addition to radius.
JOINT_TOLERANCE = {
    'home':    60.0,
    'extend':  45.0,
    'engage':  45.0,
    'retract': 45.0,
}
_DEFAULT_JOINT_TOLERANCE = 45.0


def _can_override_transition(arm, target):
    """True if the arm's current pose is close enough to `target` to override
    the state-machine check.

    Returns False on any read error or if the arm is too far in joint space or
    Cartesian space. Uses forward kinematics on the target joint angles so we
    don't need to pre-compute Cartesian target poses.
    """
    if arm is None or target not in JOINT_POSITIONS:
        return False
    target_joints = JOINT_POSITIONS[target]
    tol = JOINT_TOLERANCE.get(target, _DEFAULT_JOINT_TOLERANCE)

    code, current_joints = arm.get_servo_angle()
    if code != 0 or current_joints is None:
        return False
    for c, t in zip(current_joints, target_joints):
        if abs(c - t) > tol:
            return False

    code, current_pose = arm.get_position()
    if code != 0 or current_pose is None:
        return False
    code, target_pose = arm.get_forward_kinematics(
        target_joints, input_is_radian=False, return_is_radian=False)
    if code != 0 or target_pose is None:
        return False

    dx = current_pose[0] - target_pose[0]
    dy = current_pose[1] - target_pose[1]
    dz = current_pose[2] - target_pose[2]
    return (dx * dx + dy * dy + dz * dz) ** 0.5 <= POSITION_RADII.get(target, 0.0)


def _read_ip_from_conf():
    """Read the robot IP from example/wrapper/robot.conf, returning '' on failure."""
    conf_path = os.path.join(os.path.dirname(__file__), '../example/wrapper/robot.conf')
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read(conf_path)
        return parser.get('xArm', 'ip')
    except Exception:
        return ''


def _detect_magnitude(words):
    """Return the step size (mm) based on qualifier words in the utterance."""
    if words & LOT_WORDS:
        return STEP['lot']
    if words & MORE_WORDS:
        return STEP['more']
    if words & LITTLE_WORDS:
        return STEP['little']
    return STEP['normal']


def _detect_rot_magnitude(words):
    """Return the rotation step size (degrees) based on qualifier words."""
    if words & LOT_WORDS:
        return ROT_STEP['lot']
    if words & MORE_WORDS:
        return ROT_STEP['more']
    if words & LITTLE_WORDS:
        return ROT_STEP['little']
    return ROT_STEP['normal']


# Phrase aliases: (regex pattern, canonical replacement). Applied in order on
# the cleaned utterance before command parsing, so any alias on the left is
# interpreted as if the user had spoken the phrase on the right. Add new
# aliases here — they require no other code changes.
#
# Ordering rules:
#   • Put multi-word aliases before single-word ones that could be substrings.
#   • Use \b word boundaries so "origin" doesn't match inside "origins".
#   • Use negative lookbehinds when an alias is a substring of an existing
#     canonical phrase (e.g. "home" alone → "go home", but not inside
#     "go home" which is already the canonical form).
COMMAND_ALIASES = [
    (r'\btoes?\s+in\b',        'roll left'),
    (r'\btoes?\s+out\b',       'roll right'),
    (r'\borigin\b',            'go home'),
    (r'(?<!go\s)\bhome\b',     'go home'),
]


def _apply_aliases(cleaned):
    """Apply COMMAND_ALIASES substitutions to an already-cleaned utterance."""
    for pattern, replacement in COMMAND_ALIASES:
        cleaned = re.sub(pattern, replacement, cleaned)
    return cleaned


def parse_command(text):
    """
    Parses natural-language commands with optional magnitude qualifiers.

    Returns one of:
      ('move',   dx, dy, dz, label)          — translation, mm
      ('rotate', droll, dpitch, dyaw, label) — rotation, degrees
      ('home', label)
      ('stop', label)
      ('gripper_open', label)
      ('gripper_close', label)
      None
    """
    cleaned = _apply_aliases(re.sub(r'[^\w\s]', '', text.lower()))
    words = set(cleaned.split())

    if 'stop' in words:
        return ('stop', 'Emergency Stop')
    # Require the exact adjacent phrase to avoid hallucinated single words
    if 'go home' in cleaned:
        return ('home', 'Go Home')
    if 'engage' in words:
        return ('engage', 'Go to Engage')
    if 'extend' in words:
        return ('extend', 'Go to Extend')
    if 'retract' in words:
        return ('retract', 'Go to Retract')
    if 'open' in words:
        return ('gripper_open', 'Open Gripper')
    if 'close' in words:
        return ('gripper_close', 'Close Gripper')

    # Rotation commands — checked before DIRECTION_MAP so that compound phrases
    # like "yaw left" or "roll right" don't resolve as translational moves.
    for axis_word, (axis_idx, pos_words, neg_words, pos_label, neg_label) in ROTATION_MAP.items():
        if axis_word in words:
            step = _detect_rot_magnitude(words)
            sign = -1 if words & neg_words else +1
            drot = [0, 0, 0]
            drot[axis_idx] = sign * step
            rot_label = neg_label if sign < 0 else pos_label
            mag_names = {ROT_STEP['little']: 'a little', ROT_STEP['normal']: '',
                         ROT_STEP['more']: 'more', ROT_STEP['lot']: 'a lot'}
            qualifier = mag_names.get(step, '')
            label = f"{rot_label}  {qualifier}({step}°)".strip()
            return ('rotate', drot[0], drot[1], drot[2], label)

    for keyword, (ux, uy, uz, base_label) in DIRECTION_MAP.items():
        if keyword in words:
            step = _detect_magnitude(words)
            dx, dy, dz = ux * step, uy * step, uz * step
            mag_names = {STEP['little']: 'a little', STEP['normal']: '',
                         STEP['more']: 'more', STEP['lot']: 'a lot'}
            qualifier = mag_names.get(step, '')
            label = f"{base_label}  {qualifier}({step} mm)".strip()
            return ('move', dx, dy, dz, label)

    return None


def _cmd_label(cmd):
    """Human-readable label for a parsed command tuple."""
    return cmd[-1] if cmd else '—'


# ---------------------------------------------------------------------------
# Background threads
# ---------------------------------------------------------------------------

def listen_loop(cmd_queue, stop_event, on_mic_state, on_heard):
    """
    Captures mic audio, recognises speech, and enqueues parsed commands.

    Uses faster-whisper (CTranslate2 + INT8) which is 4–8× faster than
    openai-whisper on CPU. Key optimisations:
      - INT8 quantised tiny.en model (English-only, no language detection step)
      - beam_size=1 (greedy decode — single pass, no beam search overhead)
      - built-in VAD filter skips silent chunks before they reach the model
      - RMS loudness gate rejects quiet frames before any model work

    Install: pip install faster-whisper

    Calls on_mic_state(str) to update the mic indicator.
    Calls on_heard(raw_text, cmd_or_None) for every recognition result.
    """
    try:
        from faster_whisper import WhisperModel
    except ImportError:
        on_mic_state("ERROR: run  pip install faster-whisper")
        return

    on_mic_state("Loading Whisper model…")
    # int8 quantisation cuts memory and inference time ~2× on CPU
    whisper_model = WhisperModel("tiny.en", device="cpu", compute_type="int8")

    r = sr.Recognizer()
    r.pause_threshold = 0.4
    r.phrase_threshold = 0.1
    r.non_speaking_duration = 0.3

    try:
        mic = sr.Microphone()
    except Exception as e:
        on_mic_state(f"Mic error: {e}")
        return

    _PROMPT = ("move forward a little, move backward more, move left a lot, "
               "move right slightly, move up, move down, "
               "roll left, roll right a little, pitch up more, pitch down, yaw left, yaw right a lot, "
               "go home, engage, extend, retract, stop, open, close")

    with mic as source:
        on_mic_state("Calibrating microphone…")
        r.adjust_for_ambient_noise(source, duration=1)
        on_mic_state("● Listening…")

        while not stop_event.is_set():
            try:
                audio = r.listen(source, timeout=3, phrase_time_limit=1.5)
            except sr.WaitTimeoutError:
                on_mic_state("● Listening…")
                continue
            except Exception as e:
                on_mic_state(f"Listen error: {e}")
                continue

            on_mic_state("● Processing…")

            try:
                # Decode WAV bytes → float32 numpy array at 16 kHz mono
                wav_bytes = audio.get_wav_data(convert_rate=16000, convert_width=2)
                audio_array = np.frombuffer(wav_bytes[44:], dtype=np.int16).astype(np.float32) / 32768.0

                # RMS loudness gate — skip before touching the model
                if np.sqrt(np.mean(audio_array ** 2)) < LOUDNESS_THRESHOLD:
                    on_mic_state("● Listening…")
                    continue

                # beam_size=1 → greedy decoding (single decode pass, no beam search)
                # vad_filter → faster-whisper skips internal silent segments
                segments, _ = whisper_model.transcribe(
                    audio_array,
                    language="en",
                    initial_prompt=_PROMPT,
                    beam_size=1,
                    vad_filter=True,
                )
                text = " ".join(s.text for s in segments).lower().strip()
            except Exception as e:
                on_mic_state("● Listening…")
                on_heard(f"(error: {e})", None)
                continue

            if not text:
                on_mic_state("● Listening…")
                on_heard("(unclear)", None)
                continue

            cmd = parse_command(text)
            on_heard(text, cmd)

            if cmd is not None:
                cmd_queue.put(cmd)

            on_mic_state("● Listening…")

    on_mic_state("Stopped")


def execute_loop(arm, gripper, cmd_queue, stop_event,
                 on_active, on_done, on_log,
                 get_state, set_state, on_illegal, estop_event):
    """
    Drains cmd_queue and executes each command on the arm or gripper.

    Preset-position moves ('home', 'extend', 'engage', 'retract') are gated
    by the STATE_TRANSITIONS graph. If the transition is illegal, the arm
    pose is checked against the target radius + joint tolerance; if that also
    fails, on_illegal(target, current) is invoked and the move is skipped.

    Manual Cartesian 'move' / 'rotate' commands clear the tracked state so
    subsequent preset moves require either a re-entry via a legal source or
    a proximity override.

    If arm is None (demo mode), arm commands are logged but not sent. The
    state machine is still enforced; override is not available in demo since
    the arm can't be queried.
    Gripper commands are always executed if gripper is not None.
    Calls on_active(label) when a command starts, on_done() when it finishes.
    """
    while not stop_event.is_set():
        try:
            cmd = cmd_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        label = _cmd_label(cmd)
        kind = cmd[0]
        on_active(label)

        try:
            if kind in ('gripper_open', 'gripper_close'):
                if gripper is None:
                    on_log(f"[DEMO] Would execute: {label}")
                    import time; time.sleep(0.4)
                else:
                    on_log(f"Executing: {label}")
                    if kind == 'gripper_open':
                        gripper.open()
                    else:
                        gripper.close()

            elif kind == 'stop':
                if arm is None:
                    on_log(f"[DEMO] Would execute: {label}")
                else:
                    on_log(f"Executing: {label}")
                    arm.emergency_stop()
                set_state(None)

            elif kind == 'force_home':
                # Bypass the state machine entirely — used by the "Force Home"
                # GUI button. Slow speed since the starting pose is unknown.
                speed = 20
                if arm is None:
                    on_log(f"[DEMO] Would execute: {label} @ {speed}°/s")
                    import time; time.sleep(0.4)
                else:
                    on_log(f"FORCE: {label} @ {speed}°/s (bypassing state machine)")
                    arm.set_servo_angle(
                        angle=JOINT_POSITIONS['home'],
                        speed=speed, wait=True,
                    )
                if estop_event.is_set():
                    on_log("Force Home interrupted by stop — state cleared.")
                    set_state(None)
                    estop_event.clear()
                else:
                    set_state('home')

            elif kind in ('home', 'extend', 'engage', 'retract'):
                target = kind
                current = get_state()
                speed = _transition_speed(target, current)
                legal = speed is not None
                override = False if legal else _can_override_transition(arm, target)

                if not legal and not override:
                    on_log(f"ILLEGAL transition blocked: "
                           f"{current or 'unknown'} → {target}")
                    on_illegal(target, current)
                else:
                    if override:
                        speed = _override_speed(target)
                        on_log(f"Override allowed for '{target}' "
                               f"(within radius + joint tolerance). "
                               f"Using fallback speed {speed}°/s.")
                    if arm is None:
                        on_log(f"[DEMO] Would execute: {label} @ {speed}°/s")
                        import time; time.sleep(0.4)
                    else:
                        on_log(f"Executing: {label} @ {speed}°/s")
                        arm.set_servo_angle(
                            angle=JOINT_POSITIONS[target],
                            speed=speed, wait=True,
                        )
                    if estop_event.is_set():
                        on_log(f"Move to '{target}' interrupted by stop — state cleared.")
                        set_state(None)
                        estop_event.clear()
                    else:
                        set_state(target)

            elif kind == 'move':
                _, dx, dy, dz, _ = cmd
                if arm is None:
                    on_log(f"[DEMO] Would execute: {label}")
                    import time; time.sleep(0.4)
                else:
                    on_log(f"Executing: {label}")
                    arm.set_position(
                        x=dx, y=dy, z=dz,
                        roll=0, pitch=0, yaw=0,
                        relative=True,
                        speed=SPEED,
                        wait=True,
                    )
                set_state(None)

            elif kind == 'rotate':
                _, droll, dpitch, dyaw, _ = cmd
                if arm is None:
                    on_log(f"[DEMO] Would execute: {label}")
                    import time; time.sleep(0.4)
                else:
                    on_log(f"Executing: {label}")
                    arm.set_position(
                        x=0, y=0, z=0,
                        roll=droll, pitch=dpitch, yaw=dyaw,
                        relative=True,
                        speed=SPEED,
                        wait=True,
                    )
                set_state(None)

        except Exception as e:
            on_log(f"Error during '{label}': {e}")

        cmd_queue.task_done()
        on_done()

    on_active('—')


# ---------------------------------------------------------------------------
# Theme
# ---------------------------------------------------------------------------

C = {
    'bg':        '#0d0d0d',   # near-black page background
    'surface':   '#1a1a1a',   # card background
    'border':    '#2e2e2e',   # subtle dividers
    'text':      '#ffffff',   # white primary text
    'subtext':   '#a0a0a0',   # muted secondary text
    'green':     '#C41230',   # CMU red (connect / confirm actions)
    'green_dk':  '#8f0d22',   # CMU red dark hover
    'blue':      '#C41230',   # CMU red (start listening)
    'blue_dk':   '#8f0d22',
    'orange':    '#5a5a5a',   # mid-grey (demo mode)
    'orange_dk': '#3a3a3a',
    'red':       '#C41230',   # CMU red (emergency stop)
    'red_dk':    '#8f0d22',
    'grey':      '#2e2e2e',   # dark grey (inactive)
    'grey_dk':   '#1a1a1a',
}


def _btn(parent, text, cmd, color, color_dk, state='normal', width=14, big=False):
    """Flat-style button with hover effect. Text is near-black to contrast with
    the coloured button background."""
    fsize = 13 if big else 11
    b = tk.Button(
        parent, text=text, command=cmd,
        bg=color, fg='#111111', activebackground=color_dk, activeforeground='#111111',
        font=('Helvetica', fsize, 'bold'), width=width,
        relief='flat', bd=0, padx=12, pady=8,
        cursor='hand2', state=state,
    )
    b.bind('<Enter>', lambda e: b.config(bg=color_dk) if b['state'] != 'disabled' else None)
    b.bind('<Leave>', lambda e: b.config(bg=color)    if b['state'] != 'disabled' else None)
    return b


def _section(parent, title):
    """Dark card-style frame with a title label."""
    outer = tk.Frame(parent, bg=C['bg'])
    tk.Label(outer, text=title.upper(), bg=C['bg'], fg=C['subtext'],
             font=('Helvetica', 9, 'bold')).pack(anchor='w', padx=2, pady=(8, 2))
    inner = tk.Frame(outer, bg=C['surface'], bd=0,
                     highlightbackground=C['border'], highlightthickness=1)
    inner.pack(fill='x', padx=0, pady=0)
    return outer, inner


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------

class VoiceControlApp:
    """
    Tkinter GUI that connects to an xArm, listens for voice commands via
    Whisper, and executes Cartesian movements in real time.

    The app runs two daemon threads while listening is active:
      - listen_loop  — records audio and enqueues parsed commands
      - execute_loop — drains the queue and sends commands to the arm

    All GUI updates from those threads are posted via root.after() to stay
    on the main thread.
    """

    def __init__(self, root):
        """Initialise state and build the UI."""
        self.root = root
        self.root.title("xArm Voice Control")
        self.root.resizable(True, True)
        self.root.configure(bg=C['bg'])
        self.root.minsize(700, 700)

        self.arm = None
        self.gripper = None
        self._session_active = False  # True when connected or in demo mode
        self.listen_stop_event = threading.Event()
        self.exec_stop_event = threading.Event()
        self.estop_event = threading.Event()
        self.demo_stop_event = threading.Event()
        self.voice_thread = None
        self.exec_thread = None
        self.demo_thread = None
        self.keyboard_active = False
        self.cmd_queue = queue.Queue()
        self.tool_var = tk.StringVar(value=next(iter(TOOL_OFFSETS)))
        # Tracked preset-position state for the STATE_TRANSITIONS graph.
        # None means "unknown" — any preset move requires a proximity override.
        self.current_state = None
        self._state_lock = threading.Lock()

        # Attempt to connect to the DexHand gripper at startup
        try:
            self.gripper = GripperController()
        except Exception as e:
            self.gripper = None
            # Will log after UI is built; store message for later
            self._gripper_init_error = str(e)
        else:
            self._gripper_init_error = None

        self._build_ui()
        self.root.bind_all('<space>', self._on_space_stop)

    def _on_space_stop(self, event):
        """Spacebar triggers a soft stop, except while an *editable* text
        widget actually has focus (so typing spaces into the IP field isn't
        hijacked). A focused-but-disabled Text (e.g. the log) does not count."""
        focused = self.root.focus_get()
        if isinstance(focused, (tk.Entry, tk.Text)):
            try:
                if str(focused.cget('state')) == 'normal':
                    return
            except tk.TclError:
                pass
        if self._session_active:
            self._on_soft_stop()

    def _on_soft_stop(self):
        """Non-emergency stop: drop into state 4 to halt the current motion,
        then restore state 0 so the robot is immediately ready for new
        commands. Clears the tracked state (interrupted mid-transition) and
        flushes the pending queue. Also signals any running demo loop.

        Sets estop_event *before* issuing the stop so when the blocked move
        call on the executor thread returns, it sees the flag and clears the
        tracked state instead of claiming the target it never reached. (Same
        race as the E-stop path.)
        """
        self.estop_event.set()
        self.demo_stop_event.set()
        if self.arm:
            self.arm.set_state(4)   # stop current motion
            self.arm.set_state(0)   # re-enable motion state for next command
            self._log("Soft stop — motion halted, robot ready for new commands.")
        else:
            self._log("[DEMO] Soft stop simulated.")
        self._set_state(None)
        self._set_active('—')
        self.queue_listbox.delete(0, 'end')
        self._drain_cmd_queue()

    def _drain_cmd_queue(self):
        """Drain pending commands, calling task_done for each so that
        cmd_queue.join() (used by the demo loop) doesn't hang."""
        while not self.cmd_queue.empty():
            try:
                self.cmd_queue.get_nowait()
            except queue.Empty:
                break
            else:
                self.cmd_queue.task_done()

    # --- UI construction ---------------------------------------------------

    def _build_ui(self):
        """Construct all widgets: menubar, header, connection panel, voice controls,
        live status, and activity log/queue."""
        W = self.root
        P = {'padx': 16}

        # ── Menubar ─────────────────────────────────────────────────────────
        menubar = tk.Menu(W, bg=C['surface'], fg=C['text'],
                          activebackground='#C41230', activeforeground=C['text'],
                          relief='flat', bd=0)
        cmd_menu = tk.Menu(menubar, tearoff=0,
                           bg=C['surface'], fg=C['text'],
                           activebackground='#C41230', activeforeground=C['text'],
                           relief='flat', bd=0, font=('Courier', 12))
        cmd_menu.add_command(label='Translation qualifiers:  (none) = 20 mm  |  a little = 5 mm  |  more = 40 mm  |  a lot = 80 mm', state='disabled')
        cmd_menu.add_command(label='Rotation qualifiers:    (none) = 10°    |  a little = 2°    |  more = 20°    |  a lot = 45°',    state='disabled')
        cmd_menu.add_separator()
        cmd_menu.add_command(label='"forward [qualifier]"          →  +X', state='disabled')
        cmd_menu.add_command(label='"backward [qualifier]"         →  -X', state='disabled')
        cmd_menu.add_command(label='"left [qualifier]"             →  +Y', state='disabled')
        cmd_menu.add_command(label='"right [qualifier]"            →  -Y', state='disabled')
        cmd_menu.add_command(label='"up [qualifier]"               →  +Z', state='disabled')
        cmd_menu.add_command(label='"down [qualifier]"             →  -Z', state='disabled')
        cmd_menu.add_separator()
        cmd_menu.add_command(label='"roll left/right [qualifier]"  →  ±Roll', state='disabled')
        cmd_menu.add_command(label='"pitch up/down [qualifier]"    →  ±Pitch', state='disabled')
        cmd_menu.add_command(label='"yaw left/right [qualifier]"   →  ±Yaw',  state='disabled')
        cmd_menu.add_separator()
        cmd_menu.add_command(label='"go home"               →  return to home position', state='disabled')
        cmd_menu.add_command(label='"engage"                →  move to engage position', state='disabled')
        cmd_menu.add_command(label='"extend"                →  move to extend position', state='disabled')
        cmd_menu.add_command(label='"retract"               →  move to retract position', state='disabled')
        cmd_menu.add_command(label='"stop"                  →  emergency stop',          state='disabled')
        cmd_menu.add_separator()
        cmd_menu.add_command(label='"open"                  →  open DexHand gripper',   state='disabled')
        cmd_menu.add_command(label='"close"                 →  close DexHand gripper',  state='disabled')
        menubar.add_cascade(label='Commands', menu=cmd_menu)
        W.config(menu=menubar)

        # ── Header ──────────────────────────────────────────────────────────
        hdr = tk.Frame(W, bg='#C41230', pady=16)
        hdr.pack(fill='x')
        tk.Label(hdr, text="xArm  Voice Control", bg='#C41230', fg='#ffffff',
                 font=('Helvetica', 22, 'bold')).pack(side='left', padx=20)
        self.status_badge = tk.Label(hdr, text="  OFFLINE  ", bg='#8f0d22', fg='#ffffff',
                                     font=('Helvetica', 10, 'bold'), padx=8, pady=4)
        self.status_badge.pack(side='right', padx=20, pady=6)

        main = tk.Frame(W, bg=C['bg'])
        main.pack(fill='both', expand=True, padx=16, pady=8)

        # ── Connection ──────────────────────────────────────────────────────
        s_out, s_in = _section(main, "Connection")
        s_out.pack(fill='x', pady=0, **P)

        row = tk.Frame(s_in, bg=C['surface'])
        row.pack(fill='x', padx=12, pady=12)

        tk.Label(row, text="Robot IP", bg=C['surface'], fg=C['subtext'],
                 font=('Helvetica', 11)).grid(row=0, column=0, sticky='w', padx=(0, 8))

        self.ip_var = tk.StringVar(value=_read_ip_from_conf())
        ip_entry = tk.Entry(row, textvariable=self.ip_var, width=18,
                            bg=C['border'], fg=C['text'], insertbackground=C['text'],
                            relief='flat', font=('Helvetica', 12), bd=0)
        ip_entry.grid(row=0, column=1, ipady=6, padx=(0, 12))

        self.btn_connect = _btn(row, "Connect", self._on_connect, C['green'], C['green_dk'])
        self.btn_connect.grid(row=0, column=2, padx=4)

        self.btn_demo = _btn(row, "Demo Mode", self._on_demo, C['orange'], C['orange_dk'])
        self.btn_demo.grid(row=0, column=3, padx=4)

        self.btn_disconnect = _btn(row, "Disconnect", self._on_disconnect,
                                   C['grey'], C['grey_dk'], state='disabled')
        self.btn_disconnect.grid(row=0, column=4, padx=4)

        # ── Tool TCP Offset ──────────────────────────────────────────────────
        t_out, t_in = _section(main, "Tool TCP Offset")
        t_out.pack(fill='x', pady=(4, 0), **P)

        trow = tk.Frame(t_in, bg=C['surface'])
        trow.pack(fill='x', padx=12, pady=10)

        tk.Label(trow, text="Active tool", bg=C['surface'], fg=C['subtext'],
                 font=('Helvetica', 11)).grid(row=0, column=0, sticky='w', padx=(0, 8))

        tool_menu = tk.OptionMenu(trow, self.tool_var, *TOOL_OFFSETS.keys(),
                                  command=self._on_tool_changed)
        tool_menu.config(bg=C['border'], fg=C['text'],
                         activebackground=C['surface'], activeforeground=C['text'],
                         font=('Helvetica', 11), relief='flat',
                         highlightthickness=0, bd=0)
        tool_menu['menu'].config(bg=C['surface'], fg=C['text'],
                                 activebackground='#C41230', activeforeground=C['text'],
                                 font=('Helvetica', 11))
        tool_menu.grid(row=0, column=1, padx=(0, 12))

        self.tcp_offset_label = tk.Label(
            trow, text=self._offset_str(self.tool_var.get()),
            bg=C['surface'], fg=C['subtext'], font=('Courier', 11), anchor='w')
        self.tcp_offset_label.grid(row=0, column=2, sticky='w', padx=(0, 12))

        _btn(trow, "Apply Now", lambda: self._apply_tcp_offset(log=True),
             C['green'], C['green_dk'], width=10).grid(row=0, column=3, padx=4)

        # ── Input controls ──────────────────────────────────────────────────
        s_out, s_in = _section(main, "Input Controls")
        s_out.pack(fill='x', pady=(4, 0), **P)

        vrow = tk.Frame(s_in, bg=C['surface'])
        vrow.pack(fill='x', padx=12, pady=12)

        self.btn_start = _btn(vrow, "Start Listening", self._on_start,
                              C['blue'], C['blue_dk'], state='disabled', width=18)
        self.btn_start.pack(side='left', padx=(0, 8))

        self.btn_start_kb = _btn(vrow, "Start Keyboard Control", self._on_start_keyboard,
                                 C['blue'], C['blue_dk'], state='disabled', width=22)
        self.btn_start_kb.pack(side='left', padx=(0, 8))

        self.btn_start_demo = _btn(vrow, "Start Demo Loop", self._on_start_demo_loop,
                                   C['blue'], C['blue_dk'], state='disabled', width=18)
        self.btn_start_demo.pack(side='left', padx=(0, 8))

        self.btn_stop_all = _btn(vrow, "Stop All", self._on_stop_all,
                                 C['grey'], C['grey_dk'], state='disabled', width=14)
        self.btn_stop_all.pack(side='left', padx=(0, 16))

        self.btn_force_home = _btn(vrow, "Force Home", self._on_force_home,
                                   C['orange'], C['orange_dk'], state='disabled', width=14)
        self.btn_force_home.pack(side='left', padx=(0, 16))

        self.btn_estop = _btn(vrow, "EMERGENCY STOP", self._on_estop,
                              C['red'], C['red_dk'], state='disabled', width=18, big=True)
        self.btn_estop.pack(side='right')

        # ── Live Status ─────────────────────────────────────────────────────
        s_out, s_in = _section(main, "Live Status")
        s_out.pack(fill='x', pady=(4, 0), **P)

        grid = tk.Frame(s_in, bg=C['surface'])
        grid.pack(fill='x', padx=12, pady=12)
        grid.columnconfigure(1, weight=1)

        def _stat_row(r, label):
            tk.Label(grid, text=label, bg=C['surface'], fg=C['subtext'],
                     font=('Helvetica', 11, 'bold'), width=7, anchor='w').grid(
                row=r, column=0, sticky='w', pady=3)

        _stat_row(0, "Mic")
        self.mic_var = tk.StringVar(value="—")
        tk.Label(grid, textvariable=self.mic_var, bg=C['surface'], fg=C['subtext'],
                 font=('Helvetica', 12), anchor='w').grid(row=0, column=1, sticky='w', padx=8)

        _stat_row(1, "Heard")
        self.heard_var = tk.StringVar(value="—")
        self.heard_label = tk.Label(grid, textvariable=self.heard_var,
                                    bg=C['surface'], fg=C['text'],
                                    font=('Helvetica', 12), anchor='w')
        self.heard_label.grid(row=1, column=1, sticky='w', padx=8)

        _stat_row(2, "Active")
        self.active_var = tk.StringVar(value="—")
        self.active_label = tk.Label(grid, textvariable=self.active_var,
                                     bg=C['grey'], fg=C['text'],
                                     font=('Helvetica', 14, 'bold'),
                                     anchor='w', padx=12, pady=6, width=28)
        self.active_label.grid(row=2, column=1, sticky='w', padx=8, pady=(4, 0))

        # ── Bottom: queue + log ─────────────────────────────────────────────
        bot_out, bot_in = _section(main, "Activity")
        bot_out.pack(fill='both', expand=True, pady=(4, 8), **P)

        bot_in.columnconfigure(0, weight=1)
        bot_in.columnconfigure(1, weight=3)
        bot_in.rowconfigure(0, weight=1)

        # Queue
        q_wrap = tk.Frame(bot_in, bg=C['surface'])
        q_wrap.grid(row=0, column=0, sticky='nsew', padx=(12, 4), pady=12)
        tk.Label(q_wrap, text="Queue", bg=C['surface'], fg=C['subtext'],
                 font=('Helvetica', 10, 'bold')).pack(anchor='w', pady=(0, 4))

        self.queue_listbox = tk.Listbox(
            q_wrap, bg=C['bg'], fg=C['text'], selectbackground=C['blue'],
            font=('Courier', 12), relief='flat', bd=0,
            highlightthickness=0, activestyle='none',
        )
        self.queue_listbox.pack(fill='both', expand=True)

        # Log
        log_wrap = tk.Frame(bot_in, bg=C['surface'])
        log_wrap.grid(row=0, column=1, sticky='nsew', padx=(4, 12), pady=12)
        tk.Label(log_wrap, text="History", bg=C['surface'], fg=C['subtext'],
                 font=('Helvetica', 10, 'bold')).pack(anchor='w', pady=(0, 4))

        self.status_log = scrolledtext.ScrolledText(
            log_wrap, bg=C['bg'], fg=C['text'], insertbackground=C['text'],
            font=('Courier', 12), relief='flat', bd=0,
            highlightthickness=0, state='disabled', wrap='word',
        )
        self.status_log.pack(fill='both', expand=True)

        # ── Reference card ──────────────────────────────────────────────────
        ref_out, ref_in = _section(main, "Voice Commands")
        ref_out.pack(fill='x', pady=(0, 16), **P)

        commands = [
            ('forward [qualifier]',         '±X  (5 / 20 / 40 / 80 mm)'),
            ('backward [qualifier]',        '±X  (5 / 20 / 40 / 80 mm)'),
            ('left [qualifier]',            '±Y  (5 / 20 / 40 / 80 mm)'),
            ('right [qualifier]',           '±Y  (5 / 20 / 40 / 80 mm)'),
            ('up [qualifier]',              '±Z  (5 / 20 / 40 / 80 mm)'),
            ('down [qualifier]',            '±Z  (5 / 20 / 40 / 80 mm)'),
            ('roll left/right [qualifier]', '±Roll  (2 / 10 / 20 / 45°)'),
            ('pitch up/down [qualifier]',   '±Pitch  (2 / 10 / 20 / 45°)'),
            ('yaw left/right [qualifier]',  '±Yaw   (2 / 10 / 20 / 45°)'),
            ('go home',                     'move to home position'),
            ('engage',                      'move to engage position'),
            ('extend',                      'move to extend position'),
            ('retract',                     'move to retract position'),
            ('stop',                        'emergency stop'),
            ('open',                        'open DexHand gripper'),
            ('close',                       'close DexHand gripper'),
        ]
        ref_grid = tk.Frame(ref_in, bg=C['surface'])
        ref_grid.pack(fill='x', padx=12, pady=10)
        ref_grid.columnconfigure(0, minsize=220)
        ref_grid.columnconfigure(1, weight=1)
        for i, (cmd_txt, desc) in enumerate(commands):
            tk.Label(ref_grid, text=f'"{cmd_txt}"', bg=C['surface'], fg='#C41230',
                     font=('Courier', 12, 'bold'), anchor='w').grid(
                         row=i, column=0, sticky='w', pady=2)
            tk.Label(ref_grid, text=f'  ->  {desc}', bg=C['surface'], fg=C['subtext'],
                     font=('Helvetica', 12), anchor='w').grid(
                         row=i, column=1, sticky='w', pady=2)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # Report gripper connection status once UI is ready
        self.root.after(
            100,
            lambda: (
                self._log("Gripper connected on " + ".")
                if self.gripper is not None
                else self._log(f"Gripper not connected: {self._gripper_init_error}")
            ),
        )

    # --- Thread-safe GUI updates -------------------------------------------

    def _set_mic_state(self, text):
        """Thread-safe update of the mic status label."""
        self.root.after(0, lambda: self.mic_var.set(text))

    def _set_heard(self, raw_text, cmd):
        """Thread-safe update of the 'Heard' label. Shows green on a recognised
        command and red on an unrecognised one."""
        def _update():
            if cmd is None:
                self.heard_var.set(f'"{raw_text}"   ✗ unrecognised')
                self.heard_label.config(fg='#f87171')
            else:
                self.heard_var.set(f'"{raw_text}"   → {_cmd_label(cmd)}')
                self.heard_label.config(fg=C['green'])
        self.root.after(0, _update)

    def _set_active(self, label):
        """Thread-safe update of the 'Active' command badge with colour coding:
        grey for idle, dark-red for stop, blue-dark for home, green-dark for moves."""
        def _update():
            self.active_var.set(label)
            if label == '—':
                self.active_label.config(bg=C['grey'])
            elif 'Stop' in label or 'stop' in label:
                self.active_label.config(bg=C['red_dk'])
            elif 'Home' in label or 'home' in label:
                self.active_label.config(bg=C['blue_dk'])
            else:
                self.active_label.config(bg=C['green_dk'])
        self.root.after(0, _update)

    def _on_cmd_done(self):
        """Called by executor thread when a command finishes."""
        def _update():
            self.active_var.set('—')
            self.active_label.config(bg=C['grey'])
            if self.queue_listbox.size() > 0:
                self.queue_listbox.delete(0)
        self.root.after(0, _update)

    def _enqueue_display(self, cmd):
        """Add a command label to the listbox (called from listen_loop via on_heard)."""
        def _update():
            self.queue_listbox.insert('end', _cmd_label(cmd))
        self.root.after(0, _update)

    def _log(self, msg):
        """Thread-safe append of a message to the scrolling history log."""
        def _append():
            self.status_log.config(state='normal')
            self.status_log.insert('end', msg + '\n')
            self.status_log.see('end')
            self.status_log.config(state='disabled')
        self.root.after(0, _append)

    # --- on_heard callback (called from listen thread) ---------------------

    def _on_heard(self, raw_text, cmd):
        """Callback from listen_loop. Updates the heard label, logs the result,
        and adds the command to the queue display if recognised."""
        self._set_heard(raw_text, cmd)
        if cmd is not None:
            self._log(f'Queued: {_cmd_label(cmd)}  ("{raw_text}")')
            self._enqueue_display(cmd)
        else:
            self._log(f'Unrecognised: "{raw_text}"')

    # --- Button callbacks --------------------------------------------------

    def _offset_str(self, tool):
        """Format a tool's TCP offset as a compact human-readable string."""
        o = TOOL_OFFSETS[tool]
        return (f"x={o[0]:+.0f}  y={o[1]:+.0f}  z={o[2]:+.0f}  "
                f"roll={o[3]:+.0f}°  pitch={o[4]:+.0f}°  yaw={o[5]:+.0f}°")

    def _apply_tcp_offset(self, log=True):
        """Send the selected tool's TCP offset to the robot controller.

        set_tcp_offset resets the controller state, so we re-issue set_state(0)
        afterwards when a session is already active (mid-session tool switches).
        During initial connect the caller issues set_state(0) after us, so we
        skip it there (self._session_active is still False at that point).
        """
        tool = self.tool_var.get()
        offset = TOOL_OFFSETS[tool]
        if self.arm is not None:
            code = self.arm.set_tcp_offset(offset, is_radian=False, wait=True)
            if self._session_active:
                self.arm.set_state(state=0)  # restore sport state after config reset
            if log:
                self._log(f"TCP offset applied — '{tool}': {offset}  (code={code})")
        elif log:
            self._log(f"[DEMO] TCP offset for '{tool}': {offset}")

    def _on_tool_changed(self, tool):
        """Called when the tool dropdown selection changes."""
        self.tcp_offset_label.config(text=self._offset_str(tool))
        if self._session_active:
            self._apply_tcp_offset(log=True)

    def _on_demo(self):
        """Start with arm offline — gripper (if connected) is still live."""
        self.arm = None
        self._session_active = True
        self._apply_tcp_offset()
        if self.gripper is not None:
            self._log("Gripper-only mode — gripper is live, arm commands will be simulated.")
            self.status_badge.config(text="  GRIPPER ONLY  ", bg='#8f0d22')
        else:
            self._log("Demo mode — commands will be simulated, no robot connected.")
            self.status_badge.config(text="  DEMO  ", bg='#5a5a5a')
        self.btn_connect.config(state='disabled')
        self.btn_demo.config(state='disabled')
        self.btn_disconnect.config(state='normal', bg=C['grey'])
        self.btn_start.config(state='normal', bg=C['blue'])
        self.btn_start_kb.config(state='normal', bg=C['blue'])
        self.btn_start_demo.config(state='normal', bg=C['blue'])
        self.btn_force_home.config(state='normal', bg=C['orange'])
        self.btn_estop.config(state='normal', bg=C['red'])
        self._ensure_executor()
        # Release IP Entry focus so the spacebar soft-stop binding isn't
        # swallowed by the Entry's edit mode.
        self.root.focus_set()

    def _on_connect(self):
        """Connect to the robot at the entered IP, enable motion, and set
        position-control mode. Updates button states on success or failure."""
        ip = self.ip_var.get().strip()
        if not ip:
            self._log("ERROR: Enter a robot IP address.")
            return
        self._log(f"Connecting to {ip}…")
        try:
            self.arm = XArmAPI(ip, do_not_open=True)
            self.arm.register_error_warn_changed_callback(self._on_arm_error)
            self.arm.connect()
            self.arm.motion_enable(enable=True)
            self.arm.set_mode(0)
            self._apply_tcp_offset()        # set offset before enabling sport state
            self.arm.set_state(state=0)
            self._log("Connected. Motion enabled.")
            self._session_active = True
        except Exception as e:
            self._log(f"Connection failed: {e}")
            self.arm = None
            if self.gripper is not None:
                self._log("Arm unavailable — falling back to gripper-only mode.")
                self._on_demo()
            return

        self.btn_connect.config(state='disabled')
        self.btn_disconnect.config(state='normal', bg=C['grey'])
        self.btn_start.config(state='normal', bg=C['blue'])
        self.btn_start_kb.config(state='normal', bg=C['blue'])
        self.btn_start_demo.config(state='normal', bg=C['blue'])
        self.btn_force_home.config(state='normal', bg=C['orange'])
        self.btn_estop.config(state='normal', bg=C['red'])
        self._ensure_executor()
        # Release IP Entry focus so the spacebar soft-stop binding isn't
        # swallowed by the Entry's edit mode.
        self.root.focus_set()
        if self.gripper is not None:
            self.status_badge.config(text="  LIVE  ", bg='#8f0d22')
        else:
            self.status_badge.config(text="  ARM ONLY  ", bg='#8f0d22')

    def _on_disconnect(self):
        """Stop listening, disconnect the arm, and reset all buttons to their
        disconnected state."""
        self._session_active = False
        self._on_stop_all()
        self.exec_stop_event.set()
        if self.arm:
            self.arm.disconnect()
            self.arm = None
        self._set_state(None)
        self._log("Disconnected.")
        self.status_badge.config(text="  OFFLINE  ", bg='#2e2e2e')
        self.btn_connect.config(state='normal', bg=C['green'])
        self.btn_demo.config(state='normal', bg=C['orange'])
        self.btn_disconnect.config(state='disabled', bg=C['grey_dk'])
        self.btn_start.config(state='disabled', bg=C['blue_dk'])
        self.btn_start_kb.config(state='disabled', bg=C['blue_dk'])
        self.btn_start_demo.config(state='disabled', bg=C['blue_dk'])
        self.btn_stop_all.config(state='disabled', bg=C['grey_dk'])
        self.btn_force_home.config(state='disabled', bg=C['orange_dk'])
        self.btn_estop.config(state='disabled', bg=C['red_dk'])

    def _ensure_executor(self):
        """Start the executor thread if it isn't already running."""
        if self.exec_thread and self.exec_thread.is_alive():
            return
        self.exec_stop_event.clear()
        self.exec_thread = threading.Thread(
            target=execute_loop,
            args=(self.arm, self.gripper, self.cmd_queue, self.exec_stop_event,
                  self._set_active, self._on_cmd_done, self._log,
                  self._get_state, self._set_state, self._on_illegal_transition,
                  self.estop_event),
            daemon=True,
        )
        self.exec_thread.start()

    def _clear_queue(self):
        self.queue_listbox.delete(0, 'end')
        while not self.cmd_queue.empty():
            try:
                self.cmd_queue.get_nowait()
            except queue.Empty:
                break

    def _on_start(self):
        """Launch the voice listener daemon thread. Executor is started on
        session activation and stays up independently."""
        if self.voice_thread and self.voice_thread.is_alive():
            return
        self._clear_queue()
        self._ensure_executor()
        self.listen_stop_event.clear()

        self.voice_thread = threading.Thread(
            target=listen_loop,
            args=(self.cmd_queue, self.listen_stop_event,
                  self._set_mic_state, self._on_heard),
            daemon=True,
        )
        self.voice_thread.start()

        self.btn_start.config(state='disabled')
        self.btn_stop_all.config(state='normal')
        self._log("Voice listener started.")

    def _on_start_keyboard(self):
        """Bind number keys 0-4 to preset moves / demo. Keys fire only while
        this control is active — Stop All clears the bindings."""
        if self.keyboard_active:
            return
        self._ensure_executor()
        self.keyboard_active = True
        self.root.bind_all('<Key-1>', lambda e: self._keyboard_cmd('home',    'Go Home'))
        self.root.bind_all('<Key-2>', lambda e: self._keyboard_cmd('extend',  'Go to Extend'))
        self.root.bind_all('<Key-3>', lambda e: self._keyboard_cmd('engage',  'Go to Engage'))
        self.root.bind_all('<Key-4>', lambda e: self._keyboard_cmd('retract', 'Go to Retract'))
        self.root.bind_all('<Key-6>', lambda e: self._keyboard_cmd('gripper_open',  'Open Gripper'))
        self.root.bind_all('<Key-7>', lambda e: self._keyboard_cmd('gripper_close', 'Close Gripper'))
        self.root.bind_all('<Key-0>', lambda e: self._on_start_demo_loop())

        self.btn_start_kb.config(state='disabled')
        self.btn_stop_all.config(state='normal')
        self._log("Keyboard control started — 1=Home  2=Extend  3=Engage  "
                  "4=Retract  6=Open  7=Close  0=Demo Loop.")

    def _keyboard_cmd(self, kind, label):
        """Enqueue a preset-move command from a keyboard press."""
        cmd = (kind, label)
        self.cmd_queue.put(cmd)
        self._enqueue_display(cmd)
        self._log(f"Keyboard: {label}")

    def _on_stop_all(self):
        """Stop the voice listener, keyboard bindings, and demo loop."""
        self.listen_stop_event.set()
        if self.keyboard_active:
            for k in ('<Key-0>', '<Key-1>', '<Key-2>', '<Key-3>', '<Key-4>',
                      '<Key-6>', '<Key-7>'):
                self.root.unbind_all(k)
            self.keyboard_active = False
            self._log("Keyboard control stopped.")
        if self.demo_thread and self.demo_thread.is_alive():
            self.demo_stop_event.set()
            self._log("Demo loop stop requested.")
        self._log("Voice listener stopped.")
        self._set_mic_state("Stopped")
        enabled = 'normal' if self._session_active else 'disabled'
        self.btn_start.config(state=enabled)
        self.btn_start_kb.config(state=enabled)
        self.btn_start_demo.config(state=enabled)
        self.btn_stop_all.config(state='disabled')

    # --- Demo loop ---------------------------------------------------------
    #
    # Cycle 1→2→3→4 (home→extend→engage→retract), pause 6.7 s, then
    # 4→3→2→1 and pause 16.7 s, repeating. The loop enqueues preset commands
    # onto cmd_queue and waits for each to complete via queue.join(), so the
    # normal executor path (state-machine gating, UI updates, estop handling)
    # is reused for free. Stopped by the spacebar soft-stop, Stop All, the
    # E-stop button, or disconnect — all of which set demo_stop_event.

    DEMO_ORDER = ('home', 'extend', 'engage', 'retract')
    DEMO_LABELS = {
        'home':    'Demo → Home',
        'extend':  'Demo → Extend',
        'engage':  'Demo → Engage',
        'retract': 'Demo → Retract',
    }
    DEMO_PAUSE_AT_RETRACT = 6.7
    DEMO_PAUSE_AT_HOME    = 16.7

    def _pick_demo_start_state(self):
        """Return a starting preset name for the demo loop.

        Prefers the currently tracked state. If unknown, probes each preset
        in order and picks the first one whose override criteria (Cartesian
        radius + per-joint tolerance) the arm is currently within. Returns
        None if no preset is acceptable.
        """
        current = self._get_state()
        if current in self.DEMO_ORDER:
            return current
        if self.arm is None:
            return None  # can't probe without a live arm
        for state in self.DEMO_ORDER:
            if _can_override_transition(self.arm, state):
                return state
        return None

    def _on_start_demo_loop(self):
        """Start the demo cycle from the nearest acceptable state."""
        if self.demo_thread and self.demo_thread.is_alive():
            return
        if not self._session_active:
            return

        start = self._pick_demo_start_state()
        if start is None:
            self._log("Demo loop cannot start: no tracked state and the arm "
                      "is not within any preset's override radius + joint "
                      "tolerance. Move to a known preset (e.g. Force Home) "
                      "first.")
            return

        # If we found the start via proximity override, adopt it as the
        # tracked state so the executor's transition checks succeed.
        if self._get_state() != start:
            self._set_state(start)
            self._log(f"Demo loop: adopting '{start}' as current state "
                      f"(within override radius).")

        self.demo_stop_event.clear()
        self.demo_thread = threading.Thread(
            target=self._run_demo_loop,
            args=(start,),
            daemon=True,
        )
        self.demo_thread.start()
        self.btn_start_demo.config(state='disabled')
        self.btn_stop_all.config(state='normal')
        self._log(f"Demo loop started at '{start}'.")

    def _run_demo_loop(self, start_state):
        """Run the 1→4→1 cycle with the two pauses. See class-level notes."""
        try:
            idx = self.DEMO_ORDER.index(start_state)
            first = True

            while not self.demo_stop_event.is_set():
                # Forward leg toward retract.
                start_i = idx if first else 0
                for i in range(start_i + 1, len(self.DEMO_ORDER)):
                    if not self._demo_step(self.DEMO_ORDER[i]):
                        return
                # Pause at retract.
                if self.demo_stop_event.wait(self.DEMO_PAUSE_AT_RETRACT):
                    return
                # Reverse leg toward home.
                for i in range(len(self.DEMO_ORDER) - 2, -1, -1):
                    if not self._demo_step(self.DEMO_ORDER[i]):
                        return
                # Pause at home.
                if self.demo_stop_event.wait(self.DEMO_PAUSE_AT_HOME):
                    return
                first = False
        finally:
            self._log("Demo loop ended.")
            self.root.after(0, self._demo_button_reset)

    def _demo_step(self, target):
        """Push one preset move and wait for the executor to finish it.

        Returns False if the demo was stopped, or the executor failed to
        reach `target` (illegal transition, estop-interrupted, etc.).
        """
        if self.demo_stop_event.is_set():
            return False
        cmd = (target, self.DEMO_LABELS[target])
        self.cmd_queue.put(cmd)
        self._enqueue_display(cmd)
        self.cmd_queue.join()   # drains on stop via _drain_cmd_queue
        if self.demo_stop_event.is_set():
            return False
        if self._get_state() != target:
            self._log(f"Demo loop: expected '{target}' but tracked state is "
                      f"'{self._get_state() or 'unknown'}' — aborting.")
            return False
        return True

    def _demo_button_reset(self):
        """Restore Start Demo button availability after the thread exits."""
        if self._session_active:
            self.btn_start_demo.config(state='normal', bg=C['blue'])

    def _on_force_home(self):
        """Enqueue a state-machine-bypassing move to the home preset."""
        cmd = ('force_home', 'FORCE Home')
        self.cmd_queue.put(cmd)
        self._enqueue_display(cmd)
        self._log("Force Home requested — bypassing state machine. "
                  "Ensure the arm is in a known-safe configuration.")

    def _on_estop(self):
        """Send an immediate emergency stop to the arm (or simulate in demo mode),
        then clear the pending command queue and display.

        Sets estop_event *before* triggering the stop so that when the blocked
        move call on the executor thread returns, it sees the flag and clears
        the tracked state instead of claiming the target it never reached.
        """
        self.estop_event.set()
        self.demo_stop_event.set()
        if self.arm:
            self.arm.emergency_stop()
            self._log("EMERGENCY STOP sent.")
        else:
            self._log("[DEMO] EMERGENCY STOP simulated.")
        self._set_state(None)
        self._set_active('—')
        self.queue_listbox.delete(0, 'end')
        self._drain_cmd_queue()

    # --- State-machine helpers --------------------------------------------

    def _get_state(self):
        with self._state_lock:
            return self.current_state

    def _set_state(self, value):
        with self._state_lock:
            self.current_state = value
        self.root.after(0, lambda: self._log(
            f"State → {value if value is not None else 'unknown'}"))

    def _on_illegal_transition(self, target, current):
        """Called from the executor thread when a preset transition is blocked.
        Logs the details so the user can see why the move was refused."""
        allowed = sorted(_allowed_sources(target)) or ['(none)']
        radius = POSITION_RADII.get(target, 0.0)
        tol = JOINT_TOLERANCE.get(target, _DEFAULT_JOINT_TOLERANCE)
        self._log(
            f"Illegal transition: '{current or 'unknown'}' → '{target}'. "
            f"Allowed sources: {', '.join(allowed)}. "
            f"Override requires TCP within {radius:.0f} mm AND every joint "
            f"within {tol:.0f}° of the target."
        )

    def _on_arm_error(self, item):
        """Registered xArm error/warn callback — logs the error and warn codes."""
        self._log(f"Arm error/warn — code={item.get('error_code')} warn={item.get('warn_code')}")

    def _on_close(self):
        """Window close handler — disconnect cleanly before destroying the window."""
        self._on_disconnect()
        if self.gripper:
            self.gripper.disconnect()
            self.gripper = None
        self.root.destroy()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    root = tk.Tk()
    app = VoiceControlApp(root)
    root.mainloop()
