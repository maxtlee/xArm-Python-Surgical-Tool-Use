#!/usr/bin/env python3
"""
Voice-Controlled xArm — 6-DOF Cartesian (v2: variable step sizes)
==================================================================
Natural-language directional commands with magnitude qualifiers.

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
  "stop"                  → emergency stop

Dependencies:
  pip install SpeechRecognition pyaudio openai-whisper
"""

import os
import re
import sys
import queue
import threading
import tkinter as tk
from tkinter import scrolledtext, font as tkfont

sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

try:
    import speech_recognition as sr
except ImportError:
    print("ERROR: SpeechRecognition not installed. Run: pip install SpeechRecognition pyaudio")
    sys.exit(1)

from xarm.wrapper import XArmAPI


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

SPEED = 50  # mm/s

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


def _detect_magnitude(words):
    """Return the step size (mm) based on qualifier words in the utterance."""
    if words & LOT_WORDS:
        return STEP['lot']
    if words & MORE_WORDS:
        return STEP['more']
    if words & LITTLE_WORDS:
        return STEP['little']
    return STEP['normal']


def _tokenize(text):
    """Lowercase and strip punctuation, return a set of words."""
    return set(re.sub(r'[^\w\s]', '', text.lower()).split())



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


def parse_command(text):
    """
    Parses natural-language commands with optional magnitude qualifiers.

    Returns one of:
      ('move', dx, dy, dz, label)  — scaled by detected magnitude
      ('home', label)
      ('stop', label)
      None
    """
    words = _tokenize(text)

    if 'stop' in words:
        return ('stop', 'Emergency Stop')
    # Require the exact adjacent phrase to avoid hallucinated single words
    cleaned = re.sub(r'[^\w\s]', '', text.lower())
    if 'go home' in cleaned:
        return ('home', 'Go Home')

    for keyword, (ux, uy, uz, base_label) in DIRECTION_MAP.items():
        if keyword in words:
            step = _detect_magnitude(words)
            dx, dy, dz = ux * step, uy * step, uz * step
            # Build a label that shows the magnitude used
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

    Two-phase detection for low latency + qualifier support:
      Phase 1 — short pause_threshold (0.4s) so direction words fire quickly.
      Phase 2 — if phase 1 detected a direction with NO qualifier, do one
                 extra listen() with a QUALIFIER_WINDOW_S timeout. If the
                 user adds a qualifier ("a little", "more", etc.) it is merged
                 before the command is enqueued. If silence times out the
                 command fires immediately with the default step.

    Calls on_mic_state(str) to update the mic indicator.
    Calls on_heard(raw_text, cmd_or_None) for every recognition result.
    """
    r = sr.Recognizer()
    r.pause_threshold = 0.4        # short silence window → fast detection
    r.phrase_threshold = 0.1
    r.non_speaking_duration = 0.3

    try:
        mic = sr.Microphone()
    except Exception as e:
        on_mic_state(f"Mic error: {e}")
        return

    with mic as source:
        on_mic_state("Calibrating microphone…")
        r.adjust_for_ambient_noise(source, duration=1)
        on_mic_state("● Listening…")

        while not stop_event.is_set():
            try:
                audio = r.listen(source, timeout=3, phrase_time_limit=3)
            except sr.WaitTimeoutError:
                on_mic_state("● Listening…")
                continue
            except Exception as e:
                on_mic_state(f"Listen error: {e}")
                continue

            on_mic_state("● Processing…")

            try:
                text = r.recognize_whisper(
                    audio, model="small", language="english",
                    initial_prompt="move forward a little, move backward more, move left a lot, move right slightly, move up, move down, go home, stop",
                    no_speech_threshold=0.8,
                    fp16=False,
                ).lower()
            except sr.UnknownValueError:
                on_mic_state("● Listening…")
                on_heard("(unclear)", None)
                continue

            cmd = parse_command(text)
            on_heard(text, cmd)

            if cmd is not None:
                cmd_queue.put(cmd)

            on_mic_state("● Listening…")

    on_mic_state("Stopped")


def execute_loop(arm, cmd_queue, stop_event, on_active, on_done, on_log):
    """
    Drains cmd_queue and executes each command on the arm.
    If arm is None (demo mode), commands are logged but not sent.
    Calls on_active(label) when a command starts, on_done() when it finishes.
    """
    while not stop_event.is_set():
        try:
            cmd = cmd_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        label = _cmd_label(cmd)
        on_active(label)

        if arm is None:
            on_log(f"[DEMO] Would execute: {label}")
            import time; time.sleep(0.4)
        else:
            on_log(f"Executing: {label}")
            try:
                if cmd[0] == 'stop':
                    arm.emergency_stop()
                elif cmd[0] == 'home':
                    arm.set_servo_angle(
                        angle=[-3.5, 9.4, 0, 13.9, 0, -23, 0],
                        speed=30, wait=True,
                    )
                elif cmd[0] == 'move':
                    _, dx, dy, dz, _ = cmd
                    arm.set_position(
                        x=dx, y=dy, z=dz,
                        roll=0, pitch=0, yaw=0,
                        relative=True,
                        speed=SPEED,
                        wait=True,
                    )
            except Exception as e:
                on_log(f"Arm error during '{label}': {e}")

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
        self._session_active = False  # True when connected or in demo mode
        self.stop_event = threading.Event()
        self.voice_thread = None
        self.exec_thread = None
        self.cmd_queue = queue.Queue()

        self._build_ui()

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
        cmd_menu.add_command(label='Qualifiers:  (none) = 20 mm  |  a little = 5 mm  |  more = 40 mm  |  a lot = 80 mm', state='disabled')
        cmd_menu.add_separator()
        cmd_menu.add_command(label='"forward [qualifier]"   →  +X', state='disabled')
        cmd_menu.add_command(label='"backward [qualifier]"  →  -X', state='disabled')
        cmd_menu.add_command(label='"left [qualifier]"      →  +Y', state='disabled')
        cmd_menu.add_command(label='"right [qualifier]"     →  -Y', state='disabled')
        cmd_menu.add_command(label='"up [qualifier]"        →  +Z', state='disabled')
        cmd_menu.add_command(label='"down [qualifier]"      →  -Z', state='disabled')
        cmd_menu.add_separator()
        cmd_menu.add_command(label='"go home"               →  return to home position', state='disabled')
        cmd_menu.add_command(label='"stop"                  →  emergency stop',          state='disabled')
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

        # ── Voice control ───────────────────────────────────────────────────
        s_out, s_in = _section(main, "Voice Control")
        s_out.pack(fill='x', pady=(4, 0), **P)

        vrow = tk.Frame(s_in, bg=C['surface'])
        vrow.pack(fill='x', padx=12, pady=12)

        self.btn_start = _btn(vrow, "Start Listening", self._on_start,
                              C['blue'], C['blue_dk'], state='disabled', width=18)
        self.btn_start.pack(side='left', padx=(0, 8))

        self.btn_stop_listen = _btn(vrow, "Stop Listening", self._on_stop_listening,
                                    C['grey'], C['grey_dk'], state='disabled', width=18)
        self.btn_stop_listen.pack(side='left', padx=(0, 16))

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

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

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

    def _on_demo(self):
        """Start in demo mode — no robot required."""
        self.arm = None
        self._session_active = True
        self._log("Demo mode — commands will be simulated, no robot connected.")
        self.status_badge.config(text="  DEMO  ", bg='#5a5a5a')
        self.btn_connect.config(state='disabled')
        self.btn_demo.config(state='disabled')
        self.btn_disconnect.config(state='normal', bg=C['grey'])
        self.btn_start.config(state='normal', bg=C['blue'])
        self.btn_estop.config(state='normal', bg=C['red'])

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
            self.arm.set_state(state=0)
            self._log("Connected. Motion enabled.")
            self._session_active = True
        except Exception as e:
            self._log(f"Connection failed: {e}")
            self.arm = None
            return

        self.btn_connect.config(state='disabled')
        self.btn_disconnect.config(state='normal', bg=C['grey'])
        self.btn_start.config(state='normal', bg=C['blue'])
        self.btn_estop.config(state='normal', bg=C['red'])
        self.status_badge.config(text="  LIVE  ", bg='#8f0d22')

    def _on_disconnect(self):
        """Stop listening, disconnect the arm, and reset all buttons to their
        disconnected state."""
        self._on_stop_listening()
        if self.arm:
            self.arm.disconnect()
            self.arm = None
        self._session_active = False
        self._log("Disconnected.")
        self.status_badge.config(text="  OFFLINE  ", bg='#2e2e2e')
        self.btn_connect.config(state='normal', bg=C['green'])
        self.btn_demo.config(state='normal', bg=C['orange'])
        self.btn_disconnect.config(state='disabled', bg=C['grey_dk'])
        self.btn_start.config(state='disabled', bg=C['blue_dk'])
        self.btn_stop_listen.config(state='disabled', bg=C['grey_dk'])
        self.btn_estop.config(state='disabled', bg=C['red_dk'])

    def _on_start(self):
        """Clear any stale queue, then launch the listen and execute daemon
        threads. No-ops if a listen session is already running."""
        if self.voice_thread and self.voice_thread.is_alive():
            return
        # Clear stale queue display
        self.queue_listbox.delete(0, 'end')
        while not self.cmd_queue.empty():
            try:
                self.cmd_queue.get_nowait()
            except queue.Empty:
                break

        self.stop_event.clear()

        self.voice_thread = threading.Thread(
            target=listen_loop,
            args=(self.cmd_queue, self.stop_event,
                  self._set_mic_state, self._on_heard),
            daemon=True,
        )
        self.exec_thread = threading.Thread(
            target=execute_loop,
            args=(self.arm, self.cmd_queue, self.stop_event,
                  self._set_active, self._on_cmd_done, self._log),
            daemon=True,
        )
        self.voice_thread.start()
        self.exec_thread.start()

        self.btn_start.config(state='disabled')
        self.btn_stop_listen.config(state='normal')
        self._log("Voice listener started.")

    def _on_stop_listening(self):
        """Signal both daemon threads to stop via the shared stop_event."""
        self.stop_event.set()
        self.btn_start.config(state='normal' if self._session_active else 'disabled')
        self.btn_stop_listen.config(state='disabled')
        self._set_mic_state("Stopped")
        self._log("Voice listener stopped.")

    def _on_estop(self):
        """Send an immediate emergency stop to the arm (or simulate in demo mode),
        then clear the pending command queue and display."""
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
        """Registered xArm error/warn callback — logs the error and warn codes."""
        self._log(f"Arm error/warn — code={item.get('error_code')} warn={item.get('warn_code')}")

    def _on_close(self):
        """Window close handler — disconnect cleanly before destroying the window."""
        self._on_disconnect()
        self.root.destroy()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    root = tk.Tk()
    app = VoiceControlApp(root)
    root.mainloop()
