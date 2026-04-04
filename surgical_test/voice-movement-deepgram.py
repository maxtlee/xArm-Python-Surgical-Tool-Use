#!/usr/bin/env python3
"""
Voice-Controlled xArm — 6-DOF Cartesian  (Deepgram Edition)
=============================================================
Speak simple directional commands to move the end-effector.
Transcription is powered by Deepgram's streaming WebSocket API
(nova-2 model) instead of local Whisper, giving lower latency
and no GPU/CPU model-load overhead.

Step sizes:
  "a little" / "slightly" / "small"  →   5 mm
  (no qualifier)                      →  20 mm
  "more" / "further" / "a bit more"  →  40 mm
  "a lot" / "much" / "way"           →  80 mm

Voice commands:
  "forward [qualifier]"   → +X
  "backward [qualifier]"  → -X
  "left [qualifier]"      → +Y
  "right [qualifier]"     → -Y
  "up [qualifier]"        → +Z
  "down [qualifier]"      → -Z
  "go home"               → return to home position
  "pickup"                → move to pickup position
  "engage"                → move to engage position
  "retract"               → move to retract position
  "stop"                  → emergency stop
  "open"                  → open DexHand gripper
  "close"                 → close DexHand gripper

Dependencies:
  pip install pyaudio deepgram-sdk dynamixel-sdk
"""

import os
import re
import sys
import queue
import threading
import tkinter as tk
from tkinter import scrolledtext
import pyaudio

sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

from xarm.wrapper import XArmAPI

try:
    from dynamixel_sdk import PortHandler, PacketHandler
    _DYNAMIXEL_AVAILABLE = True
except ImportError:
    _DYNAMIXEL_AVAILABLE = False

try:
    from deepgram import (
        DeepgramClient,
        LiveTranscriptionEvents,
        LiveOptions,
    )
    _DEEPGRAM_AVAILABLE = True
except ImportError:
    _DEEPGRAM_AVAILABLE = False

import yaml


# ---------------------------------------------------------------------------
# Deepgram configuration
# ---------------------------------------------------------------------------

DEEPGRAM_API_KEY = "a5aca16e0cef871e915e3054375224fca9a746f2"

_DG_CHUNK    = 2048
_DG_FORMAT   = pyaudio.paInt16
_DG_CHANNELS = 1
_DG_RATE     = 16000


# ---------------------------------------------------------------------------
# Gripper (DexHand via Dynamixel)
# ---------------------------------------------------------------------------

_POSES_FILE = os.path.join(os.path.dirname(__file__), "gripper_poses.yaml")

_MOTOR_IDS = {
    "PinkyFlexion":   0,
    "RingFlexion":    1,
    "MiddleFlexion":  2,
    "IndexFlexion":   3,
    "ThumbAdduction": 4,
    "ThumbFlexion":   5,
}

_TICKS_PER_REV = 4096
_ADDR_TORQUE_ENABLE    = 64
_ADDR_PROFILE_VELOCITY = 112
_ADDR_GOAL_POSITION    = 116
_ADDR_PRESENT_POSITION = 132


def _degrees_to_ticks(degrees):
    return int(degrees * _TICKS_PER_REV / 360)

def _ticks_to_degrees(ticks):
    return float(ticks * 360 / _TICKS_PER_REV)

def _load_gripper_config(path=_POSES_FILE):
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    return data.get("config", {}), data.get("poses", {})

def _pose_to_tick_list(pose_data):
    motor_degrees = pose_data.get("motor_degrees", {})
    return [
        _degrees_to_ticks(float(motor_degrees.get(name, 0.0)))
        for name, _ in sorted(_MOTOR_IDS.items(), key=lambda kv: kv[1])
    ]


class GripperController:
    def __init__(self, poses_file=_POSES_FILE):
        if not _DYNAMIXEL_AVAILABLE:
            raise RuntimeError("dynamixel_sdk not installed — run: pip install dynamixel-sdk")
        cfg, poses = _load_gripper_config(poses_file)
        port     = cfg.get("port", "/dev/ttyACM0")
        baud     = int(cfg.get("baud_rate", 2_000_000))
        protocol = float(cfg.get("protocol_version", 2.0))
        prof_vel = int(cfg.get("profile_velocity", 100))
        self._open_ticks  = _pose_to_tick_list(poses.get("open",  {}))
        self._close_ticks = _pose_to_tick_list(poses.get("close", {}))
        self._ph = PortHandler(port)
        if not self._ph.openPort():
            raise RuntimeError(f"Failed to open gripper port {port}")
        if not self._ph.setBaudRate(baud):
            raise RuntimeError("Failed to set gripper baud rate")
        self._pkt = PacketHandler(protocol)
        for dxl_id in _MOTOR_IDS.values():
            self._pkt.write1ByteTxRx(self._ph, dxl_id, _ADDR_TORQUE_ENABLE, 1)
            self._pkt.write4ByteTxRx(self._ph, dxl_id, _ADDR_PROFILE_VELOCITY, prof_vel)
        self.home_offsets = self.get_all_positions()

    def get_position(self, dxl_id):
        present, _, _ = self._pkt.read4ByteTxRx(self._ph, dxl_id, _ADDR_PRESENT_POSITION)
        return present - 0x100000000 if present & 0x80000000 else present

    def get_all_positions(self):
        return [self.get_position(dxl_id) for _, dxl_id in
                sorted(_MOTOR_IDS.items(), key=lambda kv: kv[1])]

    def _set_positions(self, tick_list):
        for _, dxl_id in sorted(_MOTOR_IDS.items(), key=lambda kv: kv[1]):
            ticks = tick_list[dxl_id]
            self._pkt.write4ByteTxRx(self._ph, dxl_id, _ADDR_GOAL_POSITION, ticks & 0xFFFFFFFF)

    def open(self):   self._set_positions(self._open_ticks)
    def close(self):  self._set_positions(self._close_ticks)

    def disconnect(self):
        for dxl_id in _MOTOR_IDS.values():
            self._pkt.write1ByteTxRx(self._ph, dxl_id, _ADDR_TORQUE_ENABLE, 0)
        self._ph.closePort()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

SPEED = 50  # mm/s

JOINT_POSITIONS = {
    'home':    [-156,  93,  97,   26,   -93,  94,  -95],
    'pickup':  [-139,  91,  98,   34.5, -48,  15,  -17],
    'engage':  [ -83,  91,  97,   59,  -100,  88.9, 38.2],
    'retract': [ -87,  91,  95,   63,  -100,  89,   28],
}

STEP = {'little': 5, 'normal': 20, 'more': 40, 'lot': 80}

LITTLE_WORDS = {'little', 'slightly', 'small', 'tiny', 'bit'}
MORE_WORDS   = {'more', 'further', 'farther', 'extra'}
LOT_WORDS    = {'lot', 'much', 'way', 'far', 'big', 'large', 'huge'}

DIRECTION_MAP = {
    'forward':  ( 1,  0,  0, 'Forward  +X'),
    'backward': (-1,  0,  0, 'Backward -X'),
    'back':     (-1,  0,  0, 'Backward -X'),
    'left':     ( 0,  1,  0, 'Left     +Y'),
    'right':    ( 0, -1,  0, 'Right    -Y'),
    'up':       ( 0,  0,  1, 'Up       +Z'),
    'down':     ( 0,  0, -1, 'Down     -Z'),
}


def _read_ip_from_conf():
    conf_path = os.path.join(os.path.dirname(__file__), '../example/wrapper/robot.conf')
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read(conf_path)
        return parser.get('xArm', 'ip')
    except Exception:
        return ''

def _detect_magnitude(words):
    if words & LOT_WORDS:    return STEP['lot']
    if words & MORE_WORDS:   return STEP['more']
    if words & LITTLE_WORDS: return STEP['little']
    return STEP['normal']

def _tokenize(text):
    return set(re.sub(r'[^\w\s]', '', text.lower()).split())

def parse_command(text):
    words   = _tokenize(text)
    cleaned = re.sub(r'[^\w\s]', '', text.lower())

    if 'stop'    in words:                         return ('stop',          'Emergency Stop')
    if 'go home' in cleaned:                        return ('home',          'Go Home')
    if 'pickup'  in words or 'pick up' in cleaned:  return ('pickup',        'Go to Pickup')
    if 'engage'  in words:                          return ('engage',         'Go to Engage')
    if 'retract' in words:                          return ('retract',        'Go to Retract')
    if 'open'    in words:                          return ('gripper_open',  'Open Gripper')
    if 'close'   in words:                          return ('gripper_close', 'Close Gripper')

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
    return cmd[-1] if cmd else '—'


# ---------------------------------------------------------------------------
# listen_loop — stopped by listen_stop only
# ---------------------------------------------------------------------------

def listen_loop(cmd_queue, listen_stop, on_mic_state, on_heard):
    """
    Streams mic audio to Deepgram until listen_stop is set.
    Final transcripts are parsed and put on cmd_queue.
    Interim transcripts update the UI only (prefixed '[live]').
    """
    if not _DEEPGRAM_AVAILABLE:
        on_mic_state("ERROR: run  pip install deepgram-sdk")
        return

    on_mic_state("Connecting to Deepgram...")
    try:
        client        = DeepgramClient(DEEPGRAM_API_KEY)
        dg_connection = client.listen.websocket.v("1")
    except Exception as e:
        on_mic_state(f"Deepgram init error: {e}")
        return

    _last_interim = {"text": ""}

    def on_transcript(self_dg, result, **kwargs):
        try:
            transcript = result.channel.alternatives[0].transcript
        except (AttributeError, IndexError):
            return
        if not transcript:
            return
        if result.is_final:
            cmd = parse_command(transcript)
            on_heard(transcript, cmd)
            if cmd is not None:
                cmd_queue.put(cmd)
            _last_interim["text"] = ""
        else:
            if transcript != _last_interim["text"]:
                _last_interim["text"] = transcript
                on_heard(f"[live] {transcript}", None)

    dg_connection.on(LiveTranscriptionEvents.Transcript, on_transcript)

    options = LiveOptions(
        model="nova-2",
        language="en-US",
        smart_format=True,
        interim_results=True,
        encoding="linear16",
        sample_rate=_DG_RATE,
    )

    if dg_connection.start(options) is False:
        on_mic_state("ERROR: Deepgram connection failed")
        return

    audio = pyaudio.PyAudio()
    try:
        stream = audio.open(
            format=_DG_FORMAT, channels=_DG_CHANNELS,
            rate=_DG_RATE, input=True,
            frames_per_buffer=_DG_CHUNK,
        )
    except Exception as e:
        on_mic_state(f"Mic error: {e}")
        dg_connection.finish()
        audio.terminate()
        return

    on_mic_state("● Listening...")
    try:
        while not listen_stop.is_set():
            try:
                data = stream.read(_DG_CHUNK, exception_on_overflow=False)
                dg_connection.send(data)
            except Exception as e:
                on_mic_state(f"Stream error: {e}")
                break
    finally:
        stream.stop_stream()
        stream.close()
        audio.terminate()
        try:
            dg_connection.finish()
        except Exception:
            pass
        on_mic_state("Stopped")


# ---------------------------------------------------------------------------
# execute_loop — stopped by exec_stop only (NOT by listen_stop)
# ---------------------------------------------------------------------------

def execute_loop(arm, gripper, cmd_queue, exec_stop, on_active, on_done, on_log):
    """
    Drains cmd_queue until exec_stop is set.

    exec_stop is set ONLY on Disconnect / window close.
    'Stop Listening' does NOT set it — so the executor stays alive across
    every stop/start mic cycle and will always process queued commands.
    """
    while not exec_stop.is_set():
        try:
            cmd = cmd_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        label = _cmd_label(cmd)
        on_active(label)

        is_gripper_cmd = cmd[0] in ('gripper_open', 'gripper_close')

        if is_gripper_cmd:
            if gripper is None:
                on_log(f"[DEMO] Would execute: {label}")
                import time; time.sleep(0.4)
            else:
                on_log(f"Executing: {label}")
                try:
                    if cmd[0] == 'gripper_open':  gripper.open()
                    else:                          gripper.close()
                except Exception as e:
                    on_log(f"Gripper error during '{label}': {e}")
        elif arm is None:
            on_log(f"[DEMO] Would execute: {label}")
            import time; time.sleep(0.4)
        else:
            on_log(f"Executing: {label}")
            try:
                if cmd[0] == 'stop':
                    arm.emergency_stop()
                elif cmd[0] in ('home', 'pickup', 'engage', 'retract'):
                    arm.set_servo_angle(angle=JOINT_POSITIONS[cmd[0]], speed=30, wait=True)
                elif cmd[0] == 'move':
                    _, dx, dy, dz, _ = cmd
                    arm.set_position(x=dx, y=dy, z=dz,
                                     roll=0, pitch=0, yaw=0,
                                     relative=True, speed=SPEED, wait=True)
            except Exception as e:
                on_log(f"Arm error during '{label}': {e}")

        cmd_queue.task_done()
        on_done()

    on_active('—')


# ---------------------------------------------------------------------------
# Theme
# ---------------------------------------------------------------------------

C = {
    'bg':        '#0d0d0d',
    'surface':   '#1a1a1a',
    'border':    '#2e2e2e',
    'text':      '#ffffff',
    'subtext':   '#a0a0a0',
    'green':     '#C41230',
    'green_dk':  '#8f0d22',
    'blue':      '#C41230',
    'blue_dk':   '#8f0d22',
    'orange':    '#5a5a5a',
    'orange_dk': '#3a3a3a',
    'red':       '#C41230',
    'red_dk':    '#8f0d22',
    'grey':      '#2e2e2e',
    'grey_dk':   '#1a1a1a',
}


def _btn(parent, text, cmd, color, color_dk, state='normal', width=14, big=False):
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
    Threading model
    ---------------
    listen_stop  — set by 'Stop Listening', cleared by 'Start Listening'.
                   Only listen_loop watches this event.

    exec_stop    — set ONLY by Disconnect / window close.
                   execute_loop watches this event.
                   It is NEVER touched by Stop/Start Listening, so the
                   executor survives across mic cycles and always drains
                   the queue.

    The executor is spawned once when the session starts (Connect / Demo)
    and lives until the session ends (Disconnect / close).
    """

    def __init__(self, root):
        self.root = root
        self.root.title("xArm Voice Control  [Deepgram]")
        self.root.resizable(True, True)
        self.root.configure(bg=C['bg'])
        self.root.minsize(700, 700)

        self.arm     = None
        self.gripper = None
        self._session_active = False

        self.listen_stop = threading.Event()  # mic only
        self.exec_stop   = threading.Event()  # executor (session end)

        self.voice_thread = None
        self.exec_thread  = None
        self.cmd_queue    = queue.Queue()

        try:
            self.gripper = GripperController()
        except Exception as e:
            self.gripper = None
            self._gripper_init_error = str(e)
        else:
            self._gripper_init_error = None

        self._build_ui()

    # -----------------------------------------------------------------------
    # Session helpers
    # -----------------------------------------------------------------------

    def _start_executor(self):
        """Spawn the executor thread once per session."""
        if self.exec_thread and self.exec_thread.is_alive():
            return
        self.exec_stop.clear()
        self.exec_thread = threading.Thread(
            target=execute_loop,
            args=(self.arm, self.gripper, self.cmd_queue, self.exec_stop,
                  self._set_active, self._on_cmd_done, self._log),
            daemon=True,
        )
        self.exec_thread.start()

    def _stop_executor(self):
        """Kill the executor (disconnect / close only)."""
        self.exec_stop.set()

    # -----------------------------------------------------------------------
    # UI construction
    # -----------------------------------------------------------------------

    def _build_ui(self):
        W = self.root
        P = {'padx': 16}

        # Menubar
        menubar  = tk.Menu(W, bg=C['surface'], fg=C['text'],
                           activebackground='#C41230', activeforeground=C['text'],
                           relief='flat', bd=0)
        cmd_menu = tk.Menu(menubar, tearoff=0,
                           bg=C['surface'], fg=C['text'],
                           activebackground='#C41230', activeforeground=C['text'],
                           relief='flat', bd=0, font=('Courier', 12))
        cmd_menu.add_command(
            label='Qualifiers:  (none)=20mm  a little=5mm  more=40mm  a lot=80mm',
            state='disabled')
        cmd_menu.add_separator()
        for line in [
            '"forward [qualifier]"  -> +X', '"backward [qualifier]" -> -X',
            '"left [qualifier]"     -> +Y', '"right [qualifier]"    -> -Y',
            '"up [qualifier]"       -> +Z', '"down [qualifier]"     -> -Z',
        ]:
            cmd_menu.add_command(label=line, state='disabled')
        cmd_menu.add_separator()
        for line in ['"go home" -> home', '"pickup" -> pickup', '"engage" -> engage',
                     '"retract" -> retract', '"stop" -> emergency stop',
                     '"open" -> open gripper', '"close" -> close gripper']:
            cmd_menu.add_command(label=line, state='disabled')
        menubar.add_cascade(label='Commands', menu=cmd_menu)
        W.config(menu=menubar)

        # Header
        hdr = tk.Frame(W, bg='#C41230', pady=16)
        hdr.pack(fill='x')
        tk.Label(hdr, text="xArm  Voice Control", bg='#C41230', fg='#ffffff',
                 font=('Helvetica', 22, 'bold')).pack(side='left', padx=20)
        tk.Label(hdr, text="  Deepgram  ", bg='#101010', fg='#13ef95',
                 font=('Helvetica', 9, 'bold'), padx=6, pady=3).pack(
                     side='left', padx=(0, 12), pady=8)
        self.status_badge = tk.Label(hdr, text="  OFFLINE  ", bg='#8f0d22', fg='#ffffff',
                                     font=('Helvetica', 10, 'bold'), padx=8, pady=4)
        self.status_badge.pack(side='right', padx=20, pady=6)

        main = tk.Frame(W, bg=C['bg'])
        main.pack(fill='both', expand=True, padx=16, pady=8)

        # Connection
        s_out, s_in = _section(main, "Connection")
        s_out.pack(fill='x', pady=0, **P)
        row = tk.Frame(s_in, bg=C['surface'])
        row.pack(fill='x', padx=12, pady=12)
        tk.Label(row, text="Robot IP", bg=C['surface'], fg=C['subtext'],
                 font=('Helvetica', 11)).grid(row=0, column=0, sticky='w', padx=(0, 8))
        self.ip_var = tk.StringVar(value=_read_ip_from_conf())
        tk.Entry(row, textvariable=self.ip_var, width=18,
                 bg=C['border'], fg=C['text'], insertbackground=C['text'],
                 relief='flat', font=('Helvetica', 12), bd=0).grid(
                     row=0, column=1, ipady=6, padx=(0, 12))
        self.btn_connect    = _btn(row, "Connect",    self._on_connect,    C['green'],  C['green_dk'])
        self.btn_demo       = _btn(row, "Demo Mode",  self._on_demo,       C['orange'], C['orange_dk'])
        self.btn_disconnect = _btn(row, "Disconnect", self._on_disconnect, C['grey'],   C['grey_dk'], state='disabled')
        self.btn_connect.grid(row=0, column=2, padx=4)
        self.btn_demo.grid(row=0, column=3, padx=4)
        self.btn_disconnect.grid(row=0, column=4, padx=4)

        # Voice control
        s_out, s_in = _section(main, "Voice Control  (Deepgram streaming)")
        s_out.pack(fill='x', pady=(4, 0), **P)
        vrow = tk.Frame(s_in, bg=C['surface'])
        vrow.pack(fill='x', padx=12, pady=12)
        self.btn_start       = _btn(vrow, "Start Listening", self._on_start,
                                    C['blue'], C['blue_dk'], state='disabled', width=18)
        self.btn_stop_listen = _btn(vrow, "Stop Listening",  self._on_stop_listening,
                                    C['grey'], C['grey_dk'], state='disabled', width=18)
        self.btn_estop       = _btn(vrow, "EMERGENCY STOP",  self._on_estop,
                                    C['red'],  C['red_dk'],  state='disabled', width=18, big=True)
        self.btn_start.pack(side='left', padx=(0, 8))
        self.btn_stop_listen.pack(side='left', padx=(0, 16))
        self.btn_estop.pack(side='right')

        # Live Status
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
        self.heard_var   = tk.StringVar(value="—")
        self.heard_label = tk.Label(grid, textvariable=self.heard_var,
                                    bg=C['surface'], fg=C['text'],
                                    font=('Helvetica', 12), anchor='w')
        self.heard_label.grid(row=1, column=1, sticky='w', padx=8)

        _stat_row(2, "Active")
        self.active_var   = tk.StringVar(value="—")
        self.active_label = tk.Label(grid, textvariable=self.active_var,
                                     bg=C['grey'], fg=C['text'],
                                     font=('Helvetica', 14, 'bold'),
                                     anchor='w', padx=12, pady=6, width=28)
        self.active_label.grid(row=2, column=1, sticky='w', padx=8, pady=(4, 0))

        # Activity
        bot_out, bot_in = _section(main, "Activity")
        bot_out.pack(fill='both', expand=True, pady=(4, 8), **P)
        bot_in.columnconfigure(0, weight=1)
        bot_in.columnconfigure(1, weight=3)
        bot_in.rowconfigure(0, weight=1)

        q_wrap = tk.Frame(bot_in, bg=C['surface'])
        q_wrap.grid(row=0, column=0, sticky='nsew', padx=(12, 4), pady=12)
        tk.Label(q_wrap, text="Queue", bg=C['surface'], fg=C['subtext'],
                 font=('Helvetica', 10, 'bold')).pack(anchor='w', pady=(0, 4))
        self.queue_listbox = tk.Listbox(
            q_wrap, bg=C['bg'], fg=C['text'], selectbackground=C['blue'],
            font=('Courier', 12), relief='flat', bd=0,
            highlightthickness=0, activestyle='none')
        self.queue_listbox.pack(fill='both', expand=True)

        log_wrap = tk.Frame(bot_in, bg=C['surface'])
        log_wrap.grid(row=0, column=1, sticky='nsew', padx=(4, 12), pady=12)
        tk.Label(log_wrap, text="History", bg=C['surface'], fg=C['subtext'],
                 font=('Helvetica', 10, 'bold')).pack(anchor='w', pady=(0, 4))
        self.status_log = scrolledtext.ScrolledText(
            log_wrap, bg=C['bg'], fg=C['text'], insertbackground=C['text'],
            font=('Courier', 12), relief='flat', bd=0,
            highlightthickness=0, state='disabled', wrap='word')
        self.status_log.pack(fill='both', expand=True)

        # Reference card
        ref_out, ref_in = _section(main, "Voice Commands")
        ref_out.pack(fill='x', pady=(0, 16), **P)
        commands = [
            ('forward [qualifier]',  '+-X  (5/20/40/80 mm)'),
            ('backward [qualifier]', '+-X  (5/20/40/80 mm)'),
            ('left [qualifier]',     '+-Y  (5/20/40/80 mm)'),
            ('right [qualifier]',    '+-Y  (5/20/40/80 mm)'),
            ('up [qualifier]',       '+-Z  (5/20/40/80 mm)'),
            ('down [qualifier]',     '+-Z  (5/20/40/80 mm)'),
            ('go home',   'move to home position'),
            ('pickup',    'move to pickup position'),
            ('engage',    'move to engage position'),
            ('retract',   'move to retract position'),
            ('stop',      'emergency stop'),
            ('open',      'open DexHand gripper'),
            ('close',     'close DexHand gripper'),
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
        self.root.after(100, lambda: (
            self._log("Gripper connected.")
            if self.gripper is not None
            else self._log(f"Gripper not connected: {self._gripper_init_error}")
        ))

    # -----------------------------------------------------------------------
    # Thread-safe GUI updates
    # -----------------------------------------------------------------------

    def _set_mic_state(self, text):
        self.root.after(0, lambda: self.mic_var.set(text))

    def _set_heard(self, raw_text, cmd):
        def _update():
            if raw_text.startswith("[live]"):
                self.heard_var.set(raw_text)
                self.heard_label.config(fg='#6b7280')
            elif cmd is None:
                self.heard_var.set(f'"{raw_text}"   x unrecognised')
                self.heard_label.config(fg='#f87171')
            else:
                self.heard_var.set(f'"{raw_text}"   -> {_cmd_label(cmd)}')
                self.heard_label.config(fg=C['green'])
        self.root.after(0, _update)

    def _set_active(self, label):
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
        def _update():
            self.active_var.set('—')
            self.active_label.config(bg=C['grey'])
            if self.queue_listbox.size() > 0:
                self.queue_listbox.delete(0)
        self.root.after(0, _update)

    def _enqueue_display(self, cmd):
        self.root.after(0, lambda: self.queue_listbox.insert('end', _cmd_label(cmd)))

    def _log(self, msg):
        def _append():
            self.status_log.config(state='normal')
            self.status_log.insert('end', msg + '\n')
            self.status_log.see('end')
            self.status_log.config(state='disabled')
        self.root.after(0, _append)

    # -----------------------------------------------------------------------
    # on_heard callback
    # -----------------------------------------------------------------------

    def _on_heard(self, raw_text, cmd):
        self._set_heard(raw_text, cmd)
        if raw_text.startswith("[live]"):
            return
        if cmd is not None:
            self._log(f'Queued: {_cmd_label(cmd)}  ("{raw_text}")')
            self._enqueue_display(cmd)
        else:
            self._log(f'Unrecognised: "{raw_text}"')

    # -----------------------------------------------------------------------
    # Button callbacks
    # -----------------------------------------------------------------------

    def _on_demo(self):
        self.arm = None
        self._session_active = True
        if self.gripper is not None:
            self._log("Gripper-only mode — arm commands will be simulated.")
            self.status_badge.config(text="  GRIPPER ONLY  ", bg='#8f0d22')
        else:
            self._log("Demo mode — all commands will be simulated.")
            self.status_badge.config(text="  DEMO  ", bg='#5a5a5a')
        self.btn_connect.config(state='disabled')
        self.btn_demo.config(state='disabled')
        self.btn_disconnect.config(state='normal', bg=C['grey'])
        self.btn_start.config(state='normal', bg=C['blue'])
        self.btn_estop.config(state='normal', bg=C['red'])
        self._start_executor()

    def _on_connect(self):
        ip = self.ip_var.get().strip()
        if not ip:
            self._log("ERROR: Enter a robot IP address.")
            return
        self._log(f"Connecting to {ip}...")
        try:
            self.arm = XArmAPI(ip, do_not_open=True)
            self.arm.register_error_warn_changed_callback(self._on_arm_error)
            self.arm.connect()
            self.arm.motion_enable(enable=True)
            self.arm.set_mode(0)
            self.arm.set_state(state=0)
            self._log("Connected. Motion enabled.")
            self._session_active = True
        except Exception as e:
            self._log(f"Connection failed: {e}")
            self.arm = None
            if self.gripper is not None:
                self._log("Falling back to gripper-only mode.")
                self._on_demo()
            return

        self.btn_connect.config(state='disabled')
        self.btn_disconnect.config(state='normal', bg=C['grey'])
        self.btn_start.config(state='normal', bg=C['blue'])
        self.btn_estop.config(state='normal', bg=C['red'])
        self.status_badge.config(
            text="  LIVE  " if self.gripper else "  ARM ONLY  ", bg='#8f0d22')
        self._start_executor()

    def _on_disconnect(self):
        self._on_stop_listening()   # stop mic thread
        self._stop_executor()       # stop executor thread
        if self.arm:
            self.arm.disconnect()
            self.arm = None
        self._session_active = False
        self._log("Disconnected.")
        self.status_badge.config(text="  OFFLINE  ", bg='#2e2e2e')
        self.btn_connect.config(state='normal',      bg=C['green'])
        self.btn_demo.config(state='normal',         bg=C['orange'])
        self.btn_disconnect.config(state='disabled', bg=C['grey_dk'])
        self.btn_start.config(state='disabled',      bg=C['blue_dk'])
        self.btn_stop_listen.config(state='disabled',bg=C['grey_dk'])
        self.btn_estop.config(state='disabled',      bg=C['red_dk'])

    def _on_start(self):
        if self.voice_thread and self.voice_thread.is_alive():
            return
        self.listen_stop.clear()  # arm the mic event for this session
        self.voice_thread = threading.Thread(
            target=listen_loop,
            args=(self.cmd_queue, self.listen_stop,
                  self._set_mic_state, self._on_heard),
            daemon=True,
        )
        self.voice_thread.start()
        self.btn_start.config(state='disabled')
        self.btn_stop_listen.config(state='normal')
        self._log("Voice listener started (Deepgram streaming).")

    def _on_stop_listening(self):
        # Sets listen_stop → kills mic thread only.
        # exec_stop is untouched → executor keeps running.
        self.listen_stop.set()
        self.btn_start.config(state='normal' if self._session_active else 'disabled')
        self.btn_stop_listen.config(state='disabled')
        self._set_mic_state("Stopped")
        self._log("Voice listener stopped.")

    def _on_estop(self):
        if self.arm:
            self.arm.emergency_stop()
            self._log("EMERGENCY STOP sent.")
        else:
            self._log("[DEMO] EMERGENCY STOP simulated.")
        self._set_active('—')
        self.queue_listbox.delete(0, 'end')
        while not self.cmd_queue.empty():
            try:
                self.cmd_queue.get_nowait()
            except queue.Empty:
                break

    def _on_arm_error(self, item):
        self._log(f"Arm error — code={item.get('error_code')} warn={item.get('warn_code')}")

    def _on_close(self):
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