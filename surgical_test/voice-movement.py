#!/usr/bin/env python3
"""
Voice-Controlled xArm7 Movement
================================
Speak simple commands to move any of the 7 joints on the xArm7.

Voice commands:
  "joint one up/down"    → joint 1  ±20°
  "joint two up/down"    → joint 2  ±20°
  "joint three up/down"  → joint 3  ±20°
  "joint four up/down"   → joint 4  ±20°
  "joint five up/down"   → joint 5  ±20°
  "joint six up/down"    → joint 6  ±20°
  "joint seven up/down"  → joint 7  ±20°
  "go home"              → return to home position
  "stop"                 → emergency stop

Dependencies:
  pip install SpeechRecognition pyaudio
"""

import os
import sys
import queue
import threading
import tkinter as tk
from tkinter import scrolledtext

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

WORD_TO_NUM = {
    'one': 1, 'two': 2, 'three': 3, 'four': 4,
    'five': 5, 'six': 6, 'seven': 7,
}

STEP_DEG = 20   # degrees moved per voice command
SPEED = 30      # deg/s


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
      ('joint', servo_id, delta, label)
      ('home', label)
      ('stop', label)
      None
    """
    text = text.lower().strip()

    if 'stop' in text:
        return ('stop', 'Emergency Stop')
    if 'home' in text:
        return ('home', 'Go Home')

    for word, num in WORD_TO_NUM.items():
        if word in text and 'joint' in text:
            if 'up' in text:
                return ('joint', num, +STEP_DEG, f'Joint {num}  +{STEP_DEG}°')
            if 'down' in text:
                return ('joint', num, -STEP_DEG, f'Joint {num}  -{STEP_DEG}°')

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
    Calls on_active(label) when a command starts, on_done() when it finishes.
    """
    while not stop_event.is_set():
        try:
            cmd = cmd_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        label = _cmd_label(cmd)
        on_active(label)
        on_log(f"Executing: {label}")

        try:
            if cmd[0] == 'stop':
                arm.emergency_stop()
            elif cmd[0] == 'home':
                arm.move_gohome(wait=True)
            elif cmd[0] == 'joint':
                _, servo_id, delta, _ = cmd
                arm.set_servo_angle(
                    servo_id=servo_id,
                    angle=delta,
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
# GUI
# ---------------------------------------------------------------------------

class VoiceControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("xArm Voice Control")
        self.root.resizable(False, False)

        self.arm = None
        self.stop_event = threading.Event()
        self.voice_thread = None
        self.exec_thread = None
        self.cmd_queue = queue.Queue()

        self._build_ui()

    # --- UI construction ---------------------------------------------------

    def _build_ui(self):
        pad = {'padx': 8, 'pady': 4}

        # ── Connection ──────────────────────────────────────────────────────
        conn_frame = tk.LabelFrame(self.root, text="Connection", padx=8, pady=6)
        conn_frame.grid(row=0, column=0, columnspan=3, sticky='ew', **pad)

        tk.Label(conn_frame, text="Robot IP:").grid(row=0, column=0, sticky='w')
        self.ip_var = tk.StringVar(value=_read_ip_from_conf())
        tk.Entry(conn_frame, textvariable=self.ip_var, width=18).grid(row=0, column=1, padx=4)

        self.btn_connect = tk.Button(conn_frame, text="Connect", width=10,
                                     bg='#4CAF50', fg='white', command=self._on_connect)
        self.btn_connect.grid(row=0, column=2, padx=4)

        self.btn_disconnect = tk.Button(conn_frame, text="Disconnect", width=10,
                                        state='disabled', command=self._on_disconnect)
        self.btn_disconnect.grid(row=0, column=3, padx=4)

        # ── Voice control buttons ───────────────────────────────────────────
        voice_frame = tk.LabelFrame(self.root, text="Voice Control", padx=8, pady=6)
        voice_frame.grid(row=1, column=0, columnspan=3, sticky='ew', **pad)

        self.btn_start = tk.Button(voice_frame, text="Start Listening", width=14,
                                   state='disabled', bg='#2196F3', fg='white',
                                   command=self._on_start)
        self.btn_start.grid(row=0, column=0, padx=4)

        self.btn_stop_listen = tk.Button(voice_frame, text="Stop Listening", width=14,
                                         state='disabled', command=self._on_stop_listening)
        self.btn_stop_listen.grid(row=0, column=1, padx=4)

        self.btn_estop = tk.Button(voice_frame, text="EMERGENCY STOP", width=16,
                                   state='disabled', bg='#f44336', fg='white',
                                   font=('Helvetica', 9, 'bold'), command=self._on_estop)
        self.btn_estop.grid(row=0, column=2, padx=8)

        # ── Live status row (mic + active command) ──────────────────────────
        live_frame = tk.LabelFrame(self.root, text="Live Status", padx=8, pady=6)
        live_frame.grid(row=2, column=0, columnspan=3, sticky='ew', **pad)

        # Mic detection
        tk.Label(live_frame, text="Mic:", font=('Helvetica', 9, 'bold')).grid(
            row=0, column=0, sticky='w')
        self.mic_var = tk.StringVar(value="—")
        tk.Label(live_frame, textvariable=self.mic_var, width=22, anchor='w',
                 font=('Courier', 9), fg='#555').grid(row=0, column=1, sticky='w', padx=4)

        # Last heard
        tk.Label(live_frame, text="Heard:", font=('Helvetica', 9, 'bold')).grid(
            row=1, column=0, sticky='w')
        self.heard_var = tk.StringVar(value="—")
        self.heard_label = tk.Label(live_frame, textvariable=self.heard_var, width=32,
                                    anchor='w', font=('Courier', 9), fg='#333')
        self.heard_label.grid(row=1, column=1, columnspan=2, sticky='w', padx=4)

        # Active command
        tk.Label(live_frame, text="Active:", font=('Helvetica', 9, 'bold')).grid(
            row=2, column=0, sticky='w')
        self.active_var = tk.StringVar(value="—")
        self.active_label = tk.Label(live_frame, textvariable=self.active_var, width=32,
                                     anchor='w', font=('Courier', 10, 'bold'),
                                     fg='white', bg='#555', relief='sunken', padx=4)
        self.active_label.grid(row=2, column=1, columnspan=2, sticky='w', padx=4, pady=2)

        # ── Command queue + history side by side ────────────────────────────
        bottom_frame = tk.Frame(self.root)
        bottom_frame.grid(row=3, column=0, columnspan=3, sticky='ew', **pad)

        # Queue listbox
        queue_frame = tk.LabelFrame(bottom_frame, text="Command Queue", padx=6, pady=4)
        queue_frame.grid(row=0, column=0, sticky='ns', padx=(0, 6))

        self.queue_listbox = tk.Listbox(queue_frame, width=22, height=8,
                                        font=('Courier', 9), selectmode='browse',
                                        activestyle='none')
        self.queue_listbox.grid(row=0, column=0)
        q_scroll = tk.Scrollbar(queue_frame, orient='vertical',
                                 command=self.queue_listbox.yview)
        q_scroll.grid(row=0, column=1, sticky='ns')
        self.queue_listbox.config(yscrollcommand=q_scroll.set)

        # History log
        log_frame = tk.LabelFrame(bottom_frame, text="History", padx=6, pady=4)
        log_frame.grid(row=0, column=1, sticky='ns')

        self.status_log = scrolledtext.ScrolledText(
            log_frame, height=8, width=34, state='disabled',
            font=('Courier', 9), wrap='word'
        )
        self.status_log.grid(row=0, column=0)

        # ── Command reference ───────────────────────────────────────────────
        ref_frame = tk.LabelFrame(self.root, text="Voice Commands", padx=8, pady=4)
        ref_frame.grid(row=4, column=0, columnspan=3, sticky='ew', **pad)

        commands_text = (
            '"joint one/two/.../seven up"   → move that joint +20°\n'
            '"joint one/two/.../seven down" → move that joint −20°\n'
            '"go home"  →  home position        "stop"  →  emergency stop'
        )
        tk.Label(ref_frame, text=commands_text, justify='left',
                 font=('Courier', 9)).grid(row=0, column=0, sticky='w')

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # --- Thread-safe GUI updates -------------------------------------------

    def _set_mic_state(self, text):
        self.root.after(0, lambda: self.mic_var.set(text))

    def _set_heard(self, raw_text, cmd):
        def _update():
            if cmd is None:
                self.heard_var.set(f'"{raw_text}"  ✗ unrecognised')
                self.heard_label.config(fg='#c00')
            else:
                self.heard_var.set(f'"{raw_text}"  → {_cmd_label(cmd)}')
                self.heard_label.config(fg='#060')
        self.root.after(0, _update)

    def _set_active(self, label):
        def _update():
            self.active_var.set(label)
            if label == '—':
                self.active_label.config(bg='#555', fg='white')
            elif 'Stop' in label or 'stop' in label:
                self.active_label.config(bg='#c62828', fg='white')
            elif 'Home' in label or 'home' in label:
                self.active_label.config(bg='#1565c0', fg='white')
            else:
                self.active_label.config(bg='#2e7d32', fg='white')
        self.root.after(0, _update)

    def _on_cmd_done(self):
        """Called by executor thread when a command finishes."""
        def _update():
            self.active_var.set('—')
            self.active_label.config(bg='#555', fg='white')
            # Remove the first item from the queue display
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
        except Exception as e:
            self._log(f"Connection failed: {e}")
            self.arm = None
            return

        self.btn_connect.config(state='disabled')
        self.btn_disconnect.config(state='normal')
        self.btn_start.config(state='normal')
        self.btn_estop.config(state='normal')

    def _on_disconnect(self):
        self._on_stop_listening()
        if self.arm:
            self.arm.disconnect()
            self.arm = None
        self._log("Disconnected.")
        self.btn_connect.config(state='normal')
        self.btn_disconnect.config(state='disabled')
        self.btn_start.config(state='disabled')
        self.btn_stop_listen.config(state='disabled')
        self.btn_estop.config(state='disabled')

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
        self.btn_start.config(state='normal' if self.arm else 'disabled')
        self.btn_stop_listen.config(state='disabled')
        self._set_mic_state("Stopped")
        self._log("Voice listener stopped.")

    def _on_estop(self):
        if self.arm:
            self.arm.emergency_stop()
            self._log("EMERGENCY STOP sent.")
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
