#!/usr/bin/env python3
"""
Voice-Controlled xArm — 6-DOF Cartesian
========================================
Speak simple directional commands to move the end-effector.

Voice commands:
  "forward"   → +X  (20 mm)
  "backward"  → -X  (20 mm)
  "left"      → +Y  (20 mm)
  "right"     → -Y  (20 mm)
  "up"        → +Z  (20 mm)
  "down"      → -Z  (20 mm)
  "go home"   → return to home position
  "stop"      → emergency stop

Dependencies:
  pip install SpeechRecognition pyaudio
"""

import os
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

STEP_MM = 20    # mm moved per voice command
SPEED   = 50    # mm/s

# Maps spoken words → (dx, dy, dz, label)
DIRECTION_MAP = {
    'forward':  ( STEP_MM,       0,       0, 'Forward  +X'),
    'backward': (-STEP_MM,       0,       0, 'Backward −X'),
    'back':     (-STEP_MM,       0,       0, 'Backward −X'),
    'left':     (      0,  STEP_MM,       0, 'Left     +Y'),
    'right':    (      0, -STEP_MM,       0, 'Right    −Y'),
    'up':       (      0,       0,  STEP_MM, 'Up       +Z'),
    'down':     (      0,       0, -STEP_MM, 'Down     −Z'),
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


def parse_command(text):
    """
    Returns one of:
      ('move', dx, dy, dz, label)
      ('home', label)
      ('stop', label)
      None
    """
    text = text.lower().strip()

    if 'stop' in text:
        return ('stop', 'Emergency Stop')
    if 'home' in text:
        return ('home', 'Go Home')

    for word, (dx, dy, dz, label) in DIRECTION_MAP.items():
        if word in text:
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
    Calls on_mic_state(str) to update the mic indicator.
    Calls on_heard(raw_text, cmd_or_None) for every recognition result.
    """
    r = sr.Recognizer()
    r.pause_threshold = 0.6

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
                audio = r.listen(source, timeout=3, phrase_time_limit=5)
            except sr.WaitTimeoutError:
                on_mic_state("● Listening…")
                continue
            except Exception as e:
                on_mic_state(f"Listen error: {e}")
                continue

            on_mic_state("● Processing…")

            try:
                text = r.recognize_google(audio).lower()
            except sr.UnknownValueError:
                on_mic_state("● Listening…")
                on_heard("(unclear)", None)
                continue
            except sr.RequestError as e:
                on_mic_state(f"API error: {e}")
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
                    arm.move_gohome(wait=True)
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
    """Flat-style button with hover effect."""
    fsize = 13 if big else 11
    b = tk.Button(
        parent, text=text, command=cmd,
        bg=color, fg=C['text'], activebackground=color_dk, activeforeground=C['text'],
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
    def __init__(self, root):
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
        W = self.root
        P = {'padx': 16}

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

        # ── Reference card ──────────────────────────────────────────────────
        ref_out, ref_in = _section(main, "Voice Commands")
        ref_out.pack(fill='x', pady=(0, 16), **P)

        commands = [
            ('forward / backward', f'±X  ({STEP_MM} mm)'),
            ('left / right',       f'±Y  ({STEP_MM} mm)'),
            ('up / down',          f'±Z  ({STEP_MM} mm)'),
            ('go home',            'return to home position'),
            ('stop',               'emergency stop'),
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

    # --- Thread-safe GUI updates -------------------------------------------

    def _set_mic_state(self, text):
        self.root.after(0, lambda: self.mic_var.set(text))

    def _set_heard(self, raw_text, cmd):
        def _update():
            if cmd is None:
                self.heard_var.set(f'"{raw_text}"   ✗ unrecognised')
                self.heard_label.config(fg='#f87171')
            else:
                self.heard_var.set(f'"{raw_text}"   → {_cmd_label(cmd)}')
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
        def _append():
            self.status_log.config(state='normal')
            self.status_log.insert('end', msg + '\n')
            self.status_log.see('end')
            self.status_log.config(state='disabled')
        self.root.after(0, _append)

    # --- on_heard callback (called from listen thread) ---------------------

    def _on_heard(self, raw_text, cmd):
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
        self.stop_event.set()
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
        self._log(f"Arm error/warn — code={item.get('error_code')} warn={item.get('warn_code')}")

    def _on_close(self):
        self._on_disconnect()
        self.root.destroy()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    root = tk.Tk()
    app = VoiceControlApp(root)
    root.mainloop()
