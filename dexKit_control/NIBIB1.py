"""
NIBIB DexHand Demo
A GUI application for controlling the DexHand for a NIH/NIBIB demo.
Author: Jonathan P. King <jking2@andrew.cmu.edu>
License: MIT
"""

# _________________________ METADATA _________________________
__title__ = "NIBIB DexHand Demo"
__description__ = "A GUI application for controlling the DexHand for a NIH/NIBIB demo. Provides functionality to set motor positions, save and load poses, and execute predefined hand gestures."
__author__ = "Jonathan P. King"
__maintainer__ = "Jonathan P. King"
__email__ = "jking2@andrew.cmu.edu"
__copyright__ = "Copyright 2025, Jonathan P. King"
__license__ = "MIT"
__version__ = "1.0.0"
__status__ = "Development"


# _________________________ IMPORTS _________________________
import sys
import os
import time
import logging
import threading
from pathlib import Path
import yaml
import serial.tools.list_ports

from PyQt5.QtCore import Qt, QEvent
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QSlider,
    QLabel,
    QPushButton,
    QLineEdit,
    QMessageBox,
    QDoubleSpinBox,
    QComboBox,
    QTabWidget,
    QMainWindow,
)

from dynamixel_sdk import PortHandler, PacketHandler
# _________________________ PATH_HANDLER _________________________
def get_base_path():
    if getattr(sys, 'frozen', False) and hasattr(sys, '_MEIPASS'):
        # PyInstaller bundle
        return sys._MEIPASS # type: ignore
    else:
        # Normal script run
        return os.path.dirname(os.path.abspath(__file__))

BASE_PATH = get_base_path()
# _________________________ LOGGER _______________________________
class LoggerSetup:
    class ColorFormatter(logging.Formatter):
        COLORS = {
            "DEBUG": "\033[36m",  # Cyan
            "INFO": "\033[32m",  # Green
            "WARNING": "\033[33m",  # Yellow
            "ERROR": "\033[31m",  # Red
            "CRITICAL": "\033[41m",  # Red background
        }
        RESET = "\033[0m"

        def format(self, record):
            color = self.COLORS.get(record.levelname, "")
            msg = super().format(record)
            return f"{color}{msg}{self.RESET}"

    @staticmethod    
    def get_logger(name="DynamixelGUI", log_file=os.path.join(BASE_PATH, Path("NIBIB.log")), level=logging.INFO):
        log_format = "%(asctime)s [%(levelname)s] %(message)s"
        logger = logging.getLogger(name)
        logger.setLevel(level)
        logger.handlers.clear()

        # Console handler with color
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setFormatter(LoggerSetup.ColorFormatter(log_format))
        logger.addHandler(console_handler)

        # File handler without color
        file_handler = logging.FileHandler(log_file, encoding="utf-8")
        file_handler.setFormatter(logging.Formatter(log_format))
        logger.addHandler(file_handler)

        return logger

logger = LoggerSetup.get_logger()  # Initialize logger
# _________________________ DXL_ATTRIBUTES _______________________
class DXL:
    class ADDR:
        """Dynamixel Protocol 2.0 Addresses."""

        # EEPROM Area (Non-volatile, saved on power off)
        MODEL_NUMBER = 0  # 2 bytes
        MODEL_INFORMATION = 2  # 4 bytes
        FIRMWARE_VERSION = 6  # 1 byte
        ID = 7  # 1 byte
        BAUD_RATE = 8  # 1 byte
        RETURN_DELAY_TIME = 9  # 1 byte
        DRIVE_MODE = 10  # 1 byte
        OPERATING_MODE = 11  # 1 byte
        SECONDARY_ID = 12  # 1 byte
        PROTOCOL_VERSION = 13  # 1 byte
        HOMING_OFFSET = 20  # 4 bytes
        MOVING_THRESHOLD = 24  # 4 bytes
        TEMPERATURE_LIMIT = 31  # 1 byte
        MAX_VOLTAGE_LIMIT = 32  # 2 bytes
        MIN_VOLTAGE_LIMIT = 34  # 2 bytes
        PWM_LIMIT = 36  # 2 bytes
        CURRENT_LIMIT = 38  # 2 bytes
        VELOCITY_LIMIT = 44  # 4 bytes
        MAX_POSITION_LIMIT = 48  # 4 bytes
        MIN_POSITION_LIMIT = 52  # 4 bytes
        STARTUP_CONFIGURATION = 60  # 1 byte
        SHUTDOWN = 63  # 1 byte

        # RAM Area (Volatile, reset on power off)
        TORQUE_ENABLE = 64  # 1 byte
        LED = 65  # 1 byte
        STATUS_RETURN_LEVEL = 68  # 1 byte
        REGISTERED_INSTRUCTION = 69  # 1 byte
        HARDWARE_ERROR_STATUS = 70  # 1 byte
        VELOCITY_I_GAIN = 76  # 2 bytes
        VELOCITY_P_GAIN = 78  # 2 bytes
        POSITION_D_GAIN = 80  # 2 bytes
        POSITION_I_GAIN = 82  # 2 bytes
        POSITION_P_GAIN = 84  # 2 bytes
        FEEDFORWARD_2ND_GAIN = 88  # 2 bytes
        FEEDFORWARD_1ST_GAIN = 90  # 2 bytes
        BUS_WATCHDOG = 98  # 1 byte
        GOAL_PWM = 100  # 2 bytes
        GOAL_CURRENT = 102  # 2 bytes
        GOAL_VELOCITY = 104  # 4 bytes
        PROFILE_ACCELERATION = 108  # 4 bytes
        PROFILE_VELOCITY = 112  # 4 bytes
        GOAL_POSITION = 116  # 4 bytes
        REALTIME_TICK = 120  # 2 bytes
        MOVING = 122  # 1 byte
        MOVING_STATUS = 123  # 1 byte
        PRESENT_PWM = 124  # 2 bytes
        PRESENT_CURRENT = 126  # 2 bytes
        PRESENT_VELOCITY = 128  # 4 bytes
        PRESENT_POSITION = 132  # 4 bytes
        VELOCITY_TRAJECTORY = 136  # 4 bytes
        POSITION_TRAJECTORY = 140  # 4 bytes
        PRESENT_INPUT_VOLTAGE = 144  # 2 bytes
        PRESENT_TEMPERATURE = 146  # 1 byte
        BACKUP_READY = 147  # 1 byte
        INDIRECT_ADDRESS_1 = 168  # 2 bytes
        INDIRECT_ADDRESS_2 = 170  # 2 bytes
        INDIRECT_ADDRESS_28 = 222  # 2 bytes
        INDIRECT_ADDRESS_29 = 224  # 2 bytes
        INDIRECT_DATA_1 = 224  # 1 byte
        INDIRECT_DATA_2 = 225  # 1 byte
        INDIRECT_DATA_28 = 251  # 1 byte
        INDIRECT_DATA_29 = 252  # 1 byte

    class BAUD_RATE:
        B9600 = 0  # 9,600 bps
        B57600 = 1  # 57,600 bps
        B115200 = 2  # 115,200 bps
        B1M = 3  # 1,000,000 bps
        B2M = 4  # 2,000,000 bps
        B3M = 5  # 3,000,000 bps
        B4M = 6  # 4,000,000 bps
        DEFAULT_BAUD_RATE = B57600

    class OPERATING_MODE:
        CURRENT_CONTROL = 0  # Current Control Mode (Torque Control)
        VELOCITY_CONTROL = 1  # Velocity Control Mode
        POSITION_CONTROL = 3  # Position Control Mode
        EXTENDED_POSITION = 4  # Extended Position Control Mode (multi-turn)
        CURRENT_POSITION = 5  # Current-based Position Control Mode
        PWM_CONTROL = 16  # PWM Control Mode (Voltage Control)
        DEFAULT_MODE = POSITION_CONTROL
# _________________________ CONFIGURATION ________________________
class Config:
    SIX_TENDON_HAND = {
        "BaudRate": 2_000_000,
        "ProtocolVersion": 2.0,
        "Model": "XC‑330-M288",
        "Voltage": 5.0,
        "MotorIDs": [0, 1, 2, 3, 4, 5],
        "MotorNames": [
            "PinkyFlexion",
            "RingFlexion",
            "MiddleFlexion",
            "IndexFlexion",
            "ThumbAdduction",
            "ThumbFlexion",
        ],
        "PulleyDiameters": [0.02, 0.02, 0.02, 0.02, 0.02, 0.02],  # in meters
    }

    HAND = SIX_TENDON_HAND
    BAUD_RATE = HAND["BaudRate"]
    PROTOCOL_VERSION = HAND["ProtocolVersion"]
    DXL_IDS = HAND["MotorIDs"]
    PORT_NAME = "/dev/ttyACM0"  # "/dev/cu.usbmodem59700729091" on Mac, "/dev/ttyACM0" or "/dev/ttyUSB0" on Linux
    VOLTAGE = HAND["Voltage"]
    MOTOR_NAMES = HAND["MotorNames"]
    PULLEY_DIAMETERS = HAND["PulleyDiameters"]

    ADDR_MODEL_NUMBER = DXL.ADDR.MODEL_NUMBER
    ADDR_FIRMWARE_VERSION = DXL.ADDR.FIRMWARE_VERSION
    ADDR_ID = DXL.ADDR.ID
    ADDR_BAUD_RATE = DXL.ADDR.BAUD_RATE
    ADDR_RETURN_DELAY_TIME = DXL.ADDR.RETURN_DELAY_TIME
    ADDR_DRIVE_MODE = DXL.ADDR.DRIVE_MODE
    ADDR_OPERATING_MODE = DXL.ADDR.OPERATING_MODE
    ADDR_TORQUE_ENABLE = DXL.ADDR.TORQUE_ENABLE
    ADDR_PROFILE_VELOCITY = DXL.ADDR.PROFILE_VELOCITY
    ADDR_GOAL_POSITION = DXL.ADDR.GOAL_POSITION
    ADDR_PRESENT_POSITION = DXL.ADDR.PRESENT_POSITION

    OPERATING_MODE = DXL.OPERATING_MODE.POSITION_CONTROL  # Position Control Mode
    TORQUE_ON = 1
    TORQUE_OFF = 0

    DEFAULT_PROFILE_VELOCITY = 100  # Lower value for slower movement (adjust as needed)

    TICKS_PER_REV = 4096
    REL_MIN = 10 / 360 * TICKS_PER_REV
    REL_MAX = 350 / 360 * TICKS_PER_REV
    
    CONFIG_PATH = os.path.join(BASE_PATH, Path("NIBIB.yaml"))  # Path to the YAML configuration file
# _________________________ UTILITIES ____________________________
class Utility:
    @staticmethod
    def ticks_to_degrees(ticks):
        return float(ticks * 360 / Config.TICKS_PER_REV)

    @staticmethod
    def degrees_to_ticks(degrees):
        return int(degrees * Config.TICKS_PER_REV / 360)

    @staticmethod
    def ticks_to_rel(ticks, home_offset):
        return ticks - home_offset

    @staticmethod
    def rel_to_ticks(rel, home_offset):
        return rel + home_offset

    @staticmethod
    def rel_to_degrees(rel, home_offset):
        ticks = Utility.rel_to_ticks(rel, home_offset)
        return Utility.ticks_to_degrees(ticks)

    @staticmethod
    def degrees_to_rel(degrees, home_offset):
        ticks = Utility.degrees_to_ticks(degrees)
        return Utility.ticks_to_rel(ticks, home_offset)

    @staticmethod
    def rel_to_abs(rel, home_offset):
        ticks = Utility.rel_to_ticks(rel, home_offset)
        return Utility.ticks_to_degrees(ticks)

    @staticmethod
    def abs_to_rel(abs_pos, home_offset):
        ticks = Utility.degrees_to_ticks(abs_pos)
        return Utility.ticks_to_rel(ticks, home_offset)

    @staticmethod
    def abs_to_ticks(abs_pos, home_offset):
        ticks = Utility.degrees_to_ticks(abs_pos)
        return Utility.rel_to_ticks(ticks, home_offset)

    @staticmethod
    def ticks_to_abs(ticks, home_offset):
        rel = Utility.ticks_to_rel(ticks, home_offset)
        return Utility.ticks_to_degrees(rel)

    @staticmethod
    def abs_to_degrees(abs_pos, home_offset):
        ticks = Utility.abs_to_ticks(abs_pos, home_offset)
        return Utility.ticks_to_degrees(ticks)

    @staticmethod
    def degrees_to_abs(degrees, home_offset):
        ticks = Utility.degrees_to_ticks(degrees)
        return Utility.ticks_to_abs(ticks, home_offset)
# _________________________ DXL_CONTROLLER _______________________
class DynamixelController:
    def __init__(self, port_name=Config.PORT_NAME, baud_rate=Config.BAUD_RATE):
        logger.info(
            f"Initializing DynamixelController on port {port_name} at {baud_rate} baud."
        )
        try:
            self.port_handler = PortHandler(port_name)
            if not self.port_handler.openPort():
                logger.error(f"Failed to open port {port_name}")
                raise RuntimeError(f"Failed to open port {port_name}")
            if not self.port_handler.setBaudRate(baud_rate):
                logger.error("Failed to set baud rate")
                raise RuntimeError("Failed to set baud rate")
            self.packet_handler = PacketHandler(Config.PROTOCOL_VERSION)
            for dxl_id in Config.DXL_IDS:
                logger.info(
                    f"Enabling torque and setting profile velocity for motor ID {dxl_id}"
                )
                self.packet_handler.write1ByteTxRx(
                    self.port_handler,
                    dxl_id,
                    Config.ADDR_TORQUE_ENABLE,
                    Config.TORQUE_ON,
                )
                # Set profile velocity for slower movement
                self.packet_handler.write4ByteTxRx(
                    self.port_handler,
                    dxl_id,
                    Config.ADDR_PROFILE_VELOCITY,
                    Config.DEFAULT_PROFILE_VELOCITY,
                )
        except Exception as e:
            logger.error(f"Exception during DynamixelController initialization: {e}")
            raise

    def set_position(self, dxl_id: int, abs_ticks: int):
        abs_ticks = max(-1_048_575, min(1_048_575, abs_ticks))
        logger.debug(f"Setting position for motor {dxl_id}: {abs_ticks} ticks")
        self.packet_handler.write4ByteTxRx(
            self.port_handler, dxl_id, Config.ADDR_GOAL_POSITION, abs_ticks & 0xFFFFFFFF
        )

    def set_all_positions(self, abs_positions):
        logger.info(f"Setting all motor positions: {abs_positions}")
        for dxl_id, pos in zip(Config.DXL_IDS, abs_positions):
            self.set_position(dxl_id, pos)

    def get_position(self, dxl_id: int) -> int:
        present, _, _ = self.packet_handler.read4ByteTxRx(
            self.port_handler, dxl_id, Config.ADDR_PRESENT_POSITION
        )
        pos = present - 0x100000000 if present & 0x80000000 else present
        logger.debug(f"Read position for motor {dxl_id}: {pos} ticks")
        return pos

    def get_all_positions(self):
        return [self.get_position(dxl_id) for dxl_id in Config.DXL_IDS]

    def close(self):
        logger.info("Disabling torque and closing port.")
        for dxl_id in Config.DXL_IDS:
            self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, Config.ADDR_TORQUE_ENABLE, Config.TORQUE_OFF
            )
        self.port_handler.closePort()
# _________________________ POSE_GUI _____________________________
class HandPoseControllerWindow(QWidget):
    def __init__(self, ctrl):
        super().__init__()
        self.setWindowTitle("Hand Pose Controller")

        # Set monospaced font for the window
        font = self.font()
        font.setFamily("Courier New")
        self.setFont(font)

        # Set a larger default window size
        self.resize(700, 400)

        self.ctrl = ctrl

        self.home_offsets = self.ctrl.get_all_positions()
        self.sliders, self.spinboxes = [], []

        main_layout = QVBoxLayout()

        for idx, (name, dxl_id) in enumerate(zip(Config.MOTOR_NAMES, Config.DXL_IDS)):
            hbox = QHBoxLayout()
            padded_name = f"{name:<15}"
            label = QLabel(
                f"{padded_name}: {0.0:7.2f}°"
            )  # Placeholder for initial value
            label.setFont(font)  # Ensure label uses monospaced font
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(
                int(Utility.ticks_to_degrees(Config.REL_MIN)),
                int(Utility.ticks_to_degrees(Config.REL_MAX)),
            )
            slider.setSingleStep(1)
            spin = QDoubleSpinBox()
            spin.setDecimals(2)
            spin.setRange(
                Utility.ticks_to_degrees(Config.REL_MIN),
                Utility.ticks_to_degrees(Config.REL_MAX),
            )
            for widget in (slider, spin):
                degrees = int(Utility.ticks_to_degrees(self.ctrl.get_position(dxl_id)))
                widget.setValue(degrees)

            slider.valueChanged.connect(lambda val, i=idx: self.slider_changed(i, val))
            spin.editingFinished.connect(
                lambda i=idx, s=spin: self.spin_editing_finished(i, s)
            )

            slider.label = spin.label = label
            self.sliders.append(slider)
            self.spinboxes.append(spin)

            hbox.addWidget(label)
            hbox.addWidget(slider)
            hbox.addWidget(spin)
            main_layout.addLayout(hbox)

        # Pose Name input and Save Pose button on the same row
        pose_name_box = QHBoxLayout()
        pose_name_box.addWidget(QLabel("Pose Name:"))
        self.pose_name_edit = QLineEdit()
        self.pose_name_edit.setPlaceholderText("Enter pose name…")
        pose_name_box.addWidget(self.pose_name_edit)
        save_btn = QPushButton("Save Pose")
        save_btn.clicked.connect(self.save_pose)
        pose_name_box.addWidget(save_btn)
        main_layout.addLayout(pose_name_box)

        # Load Poses button, dropdown, and Send Pose button
        load_box = QHBoxLayout()
        self.load_btn = QPushButton("Load Poses")
        self.load_btn.clicked.connect(self.load_poses)
        load_box.addWidget(self.load_btn)
        self.poses_dropdown = QComboBox()
        load_box.addWidget(self.poses_dropdown)
        self.send_pose_btn = QPushButton("Send Pose")
        self.send_pose_btn.clicked.connect(self.send_selected_pose)
        load_box.addWidget(self.send_pose_btn)
        main_layout.addLayout(load_box)

        self.setLayout(main_layout)

        # Update labels with actual values after GUI is shown
        self.update_labels_with_actual_values()

    def update_labels_with_actual_values(self):
        for idx, (slider, spin) in enumerate(zip(self.sliders, self.spinboxes)):
            degrees = spin.value()
            display_degrees = degrees
            slider.label.setText(
                f"{Config.MOTOR_NAMES[idx]:<15}: {display_degrees:7.2f}°"
            )

    # ------------ Helpers ------------ #
    def rel_to_abs(self, idx, rel):
        return self.home_offsets[idx] + rel

    def slider_changed(self, idx, val):
        spin = self.spinboxes[idx]
        if abs(spin.value() - val) > 1e-2:
            spin.blockSignals(True)
            spin.setValue(val)
            spin.blockSignals(False)
        self._update_motor(idx, val)

    def spin_editing_finished(self, idx, spin):
        val = spin.value()
        slider = self.sliders[idx]
        if abs(slider.value() - val) > 1e-2:
            slider.blockSignals(True)
            slider.setValue(int(val))
            slider.blockSignals(False)
        self._update_motor(idx, val)

    def _update_motor(self, idx, degrees):
        label = self.sliders[idx].label
        display_degrees = degrees
        label.setText(f"{Config.MOTOR_NAMES[idx]:<15}: {display_degrees:7.2f}°")
        ticks = Utility.degrees_to_ticks(degrees)
        self.ctrl.set_position(
            Config.DXL_IDS[idx], ticks
        )  # Relative: self.ctrl.set_position(DXL_IDS[idx], self.rel_to_abs(idx, ticks))

    # ------------ Pose I/O ------------ #
    def save_pose(self):
        pose_name = self.pose_name_edit.text().strip()
        if not pose_name:
            logger.warning("Attempted to save pose without a name.")
            QMessageBox.warning(self, "Missing Name", "Please enter a pose name.")
            return
        pose = {
            name: spin.value() for name, spin in zip(Config.MOTOR_NAMES, self.spinboxes)
        }
        pose_entry = {"name": pose_name}
        pose_entry.update(pose)
        # Load YAML, append pose, save
        try:
            with open(Config.CONFIG_PATH, "r") as f:
                data = yaml.safe_load(f) or {}
        except Exception:
            data = {}
        if "poses" not in data:
            data["poses"] = []
        # Overwrite if pose with same name exists
        data["poses"] = [p for p in data["poses"] if p.get("name") != pose_name]
        data["poses"].append(pose_entry)
        with open(Config.CONFIG_PATH, "w") as f:
            yaml.dump(data, f, sort_keys=False)
        logger.info(f"Pose '{pose_name}' saved: {pose}")
        QMessageBox.information(self, "Pose Saved", f"Pose saved to {Config.CONFIG_PATH}")
        self.pose_name_edit.clear()

    def load_poses(self):
        self.poses_dropdown.clear()
        try:
            with open(Config.CONFIG_PATH, "r") as f:
                data = yaml.safe_load(f) or {}
        except Exception:
            logger.warning(f"Tried to load poses but {Config.CONFIG_PATH} not found or invalid.")
            QMessageBox.warning(self, "No Poses", f"{Config.CONFIG_PATH} not found or invalid.")
            return
        self.poses = []
        for pose in data.get("poses", []):
            name = pose.get("name", "Unnamed")
            self.poses.append(pose)
            self.poses_dropdown.addItem(name)
        logger.info(f"Loaded {len(self.poses)} poses from YAML.")

    def send_selected_pose(self):
        idx = self.poses_dropdown.currentIndex()
        if not hasattr(self, "poses") or idx < 0 or idx >= len(self.poses):
            logger.warning("No pose selected or poses not loaded.")
            QMessageBox.warning(
                self, "No Pose Selected", "Please load and select a pose."
            )
            return
        pose_row = self.poses[idx]
        try:
            pose_values = [float(pose_row[name]) for name in Config.MOTOR_NAMES]
        except Exception as e:
            logger.error(f"Could not parse pose values: {e}")
            QMessageBox.warning(
                self, "Invalid Pose", f"Could not parse pose values: {e}"
            )
            return
        logger.info(
            f"Sending pose '{pose_row.get('name', 'Unnamed')}' to motors: {pose_values}"
        )
        for i, val in enumerate(pose_values):
            self.spinboxes[i].blockSignals(True)
            self.spinboxes[i].setValue(val)
            self.spinboxes[i].blockSignals(False)
            self._update_motor(i, val)
        QApplication.processEvents()
        time.sleep(2.0)
        current_positions = self.ctrl.get_all_positions()
        for i, ticks in enumerate(current_positions):
            degrees = Utility.ticks_to_degrees(ticks)
            self.sliders[i].blockSignals(True)
            self.spinboxes[i].blockSignals(True)
            self.sliders[i].setValue(int(degrees))
            self.spinboxes[i].setValue(degrees)
            self.sliders[i].blockSignals(False)
            self.spinboxes[i].blockSignals(False)
            self.sliders[i].label.setText(
                f"{Config.MOTOR_NAMES[i]:<15}: {degrees:7.2f}°"
            )

    # ------------ Home ------------ #
    def set_home(self):
        self.home_offsets = self.ctrl.get_all_positions()
        for idx, (slider, spin) in enumerate(zip(self.sliders, self.spinboxes)):
            for w in (slider, spin):
                w.blockSignals(True)
                w.setValue(0.0)
                w.blockSignals(False)
            slider.label.setText(f"{Config.MOTOR_NAMES[idx]:<15}: {0.00:7.2f}°")

    # ------------ Custom UI update event ------------ #
    def customEvent(self, event):
        if isinstance(event, _UpdateUIEvent):
            for idx, rel in enumerate(event.rel_positions):
                for widget in (self.sliders[idx], self.spinboxes[idx]):
                    degrees = Utility.ticks_to_degrees(rel)
                    widget.blockSignals(True)
                    widget.setValue(degrees)
                    widget.blockSignals(False)
                self.sliders[idx].label.setText(
                    f"{Config.MOTOR_NAMES[idx]:<15}: {degrees:7.2f}°"
                )

    def closeEvent(self, event):
        self.ctrl.close()
        event.accept()
# _________________________ DEMO_GUI _____________________________
class HandDemoWindow(QWidget):
    def __init__(self, ctrl, test_mode=False):
        from PyQt5.QtWidgets import QGridLayout, QSizePolicy, QPushButton

        super().__init__()
        self.test_mode = test_mode
        self.setWindowTitle("Hand Demo Window")
        self.resize(900, 600)
        self.showMaximized()

        font = self.font()
        font.setFamily("Courier New")
        font.setPointSize(32)
        self.setFont(font)

        self.ctrl = ctrl

        # Load poses and sequences from YAML
        try:
            with open(Config.CONFIG_PATH, "r") as f:
                data = yaml.safe_load(f) or {}
        except Exception:
            data = {}
        self.poses_dict = {p["name"]: p for p in data.get("poses", [])}
        self.sequences = data.get("sequences", [])
        self.home_pose = [
            Utility.degrees_to_ticks(self.poses_dict.get("Home", {}).get(m, 20.0))
            for m in Config.MOTOR_NAMES
        ]

        main_layout = QVBoxLayout()
        grid_layout = QGridLayout()
        columns = 3
        num_buttons = len(self.sequences)
        num_rows = (num_buttons + columns - 1) // columns
        self._demo_buttons = []
        self._button_map = {}
        self._sequence_threads = {}
        self._pause_flag = False
        self._stop_flag = False

        # --- Pause/Resume and Reset row --- #
        button_row = QHBoxLayout()
        self.pause_btn = QPushButton("Pause")
        self.pause_btn.setCheckable(True)
        self.pause_btn.setFont(font)
        self.pause_btn.setStyleSheet(
            "background-color: #D32F2F; color: white; border-radius: 20px; font-weight: bold;"
        )
        self.pause_btn.toggled.connect(self.toggle_pause)
        button_row.addWidget(self.pause_btn)

        # Find Reset button index
        reset_idx = next(
            (i for i, seq in enumerate(self.sequences) if seq.get("name") == "Reset"),
            None,
        )
        if reset_idx is not None:
            reset_btn = QPushButton("Reset")
            reset_btn.setFont(font)
            reset_btn.setMinimumWidth(200)
            reset_btn.setStyleSheet(
                "background-color: #D1B3FF; color: black; border-radius: 20px; font-weight: bold;"
            )
            reset_btn.setCheckable(True)
            reset_btn.toggled.connect(
                lambda checked, b=reset_btn: self.update_button_style(
                    b, checked, is_reset=True
                )
            )
            reset_btn.clicked.connect(
                lambda checked, i=reset_idx: self.handle_sequence_button(i)
            )
            button_row.addWidget(reset_btn)
            self._button_map["Reset"] = reset_btn
            self._demo_buttons.append(reset_btn)
        main_layout.addLayout(button_row)

        # --- Sequence buttons grid (excluding Reset) --- #
        grid_btn_idx = 0
        for idx, seq in enumerate(self.sequences):
            seq_name = seq.get("name", f"Sequence {idx + 1}")
            if seq_name == "Test" and not self.test_mode:
                continue  # Don't show Test sequence
            if seq_name == "Reset":
                continue  # Already added above
            btn = QPushButton(seq_name)
            btn.setMinimumWidth(200)
            btn.setFont(font)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            btn.setCheckable(True)
            btn.setStyleSheet(
                "background-color: #34AEEB; color: black; border-radius: 20px; font-weight: bold;"
            )
            btn.toggled.connect(
                lambda checked, b=btn: self.update_button_style(b, checked)
            )
            btn.clicked.connect(lambda checked, i=idx: self.handle_sequence_button(i))
            row = grid_btn_idx // columns
            col = grid_btn_idx % columns
            grid_layout.addWidget(btn, row, col)
            self._demo_buttons.append(btn)
            self._button_map[seq_name] = btn
            grid_btn_idx += 1
        for row in range(num_rows):
            grid_layout.setRowStretch(row, 1)
        for col in range(columns):
            grid_layout.setColumnStretch(col, 1)
        main_layout.addLayout(grid_layout, stretch=1)
        self.setLayout(main_layout)

    def toggle_pause(self, checked):
        self._pause_flag = checked
        if checked:
            self.pause_btn.setText("Resume")
            self.pause_btn.setStyleSheet(
                "background-color: #388E3C; color: white; border-radius: 20px; font-weight: bold;"
            )
        else:
            self.pause_btn.setText("Pause")
            self.pause_btn.setStyleSheet(
                "background-color: #D32F2F; color: white; border-radius: 20px; font-weight: bold;"
            )

    def handle_sequence_button(self, idx):
        seq = self.sequences[idx]
        seq_name = seq.get("name", f"Sequence {idx + 1}")
        # If Reset is pressed, interrupt any running sequence and go to Home
        if seq_name == "Reset":
            self._stop_flag = True
            # Set Pause/Resume to unchecked (not paused)
            if self.pause_btn.isChecked():
                self.pause_btn.setChecked(False)
            # Uncheck all other buttons except Reset
            for b in self._demo_buttons:
                if b != self._button_map["Reset"] and b.isChecked():
                    b.setChecked(False)

            def run_reset():
                # Wait for any running sequence to actually stop
                time.sleep(0.15)
                # Always go to Home pose after Reset
                home_pose = self.poses_dict.get("Home")
                if home_pose:
                    pose_values = [
                        float(home_pose.get(m, 0.0)) for m in Config.MOTOR_NAMES
                    ]
                    ticks = [Utility.degrees_to_ticks(val) for val in pose_values]
                    self.ctrl.set_all_positions(ticks)
                    QApplication.processEvents()

                    # Wait for hand to reach Home before unchecking Reset
                    def wait_for_home_reset():
                        tolerance = 50
                        stable_count = 0
                        for _ in range(40):  # up to 4 seconds
                            current = self.ctrl.get_all_positions()
                            if all(
                                abs(a - b) <= tolerance
                                for a, b in zip(current, self.home_pose)
                            ):
                                stable_count += 1
                                if (
                                    stable_count >= 5
                                ):  # must be within tolerance for 0.5s
                                    self._button_map[seq_name].setChecked(False)
                                    return
                            else:
                                stable_count = 0
                            time.sleep(0.1)
                        self._button_map[seq_name].setChecked(False)

                    threading.Thread(target=wait_for_home_reset, daemon=True).start()
                else:
                    self._button_map[seq_name].setChecked(False)

            threading.Thread(target=run_reset, daemon=True).start()
            return
        # If another sequence is running, stop it
        self._stop_flag = True
        for b in self._demo_buttons:
            if b != self._demo_buttons[idx] and b.isChecked():
                b.setChecked(False)
        # Wait a moment to ensure previous thread sees stop flag
        time.sleep(0.05)
        self._stop_flag = False

        def run_seq():
            self.run_sequence(seq, self._demo_buttons[idx])

        threading.Thread(target=run_seq, daemon=True).start()

    def run_sequence(self, seq, btn):
        btn.setChecked(True)
        for action in seq.get("actions", []):
            if self._stop_flag:
                break
            while self._pause_flag:
                time.sleep(0.1)
            if "pose" in action:
                pose_name = action["pose"]
                pose = self.poses_dict.get(pose_name)
                if not pose:
                    logger.warning(f"Pose '{pose_name}' not found in YAML.")
                    continue
                pose_values = [float(pose.get(m, 0.0)) for m in Config.MOTOR_NAMES]
                ticks = [Utility.degrees_to_ticks(val) for val in pose_values]
                self.ctrl.set_all_positions(ticks)
                QApplication.processEvents()
            elif "delay" in action:
                delay = float(action["delay"])
                t0 = time.time()
                while time.time() - t0 < delay:
                    if self._stop_flag:
                        break
                    while self._pause_flag:
                        time.sleep(0.1)
                    time.sleep(0.01)

        # Wait for hand to reach Home pose before unchecking button (debounced)
        def wait_for_home():
            tolerance = 50  # ticks
            stable_count = 0
            for _ in range(40):  # up to 4 seconds
                current = self.ctrl.get_all_positions()
                if all(
                    abs(a - b) <= tolerance for a, b in zip(current, self.home_pose)
                ):
                    stable_count += 1
                    if stable_count >= 5:  # must be within tolerance for 0.5s
                        btn.setChecked(False)
                        return
                else:
                    stable_count = 0
                time.sleep(0.1)
            btn.setChecked(False)

        threading.Thread(target=wait_for_home, daemon=True).start()

    def update_button_style(self, btn, checked, is_reset=False):
        if is_reset:
            # Reset button: light purple when not checked, darker purple when checked
            if checked:
                btn.setStyleSheet(
                    "background-color: #B39DDB; color: #333; border-radius: 20px; font-weight: bold;"
                )
            else:
                btn.setStyleSheet(
                    "background-color: #D1B3FF; color: black; border-radius: 20px; font-weight: bold;"
                )
        else:
            if checked:
                btn.setStyleSheet(
                    "background-color: #96948C; color: #66645F; border-radius: 20px; font-weight: bold;"
                )  # Light grey bg, dark grey bold text
            else:
                btn.setStyleSheet(
                    "background-color: #34AEEB; color: black; border-radius: 20px; font-weight: bold;"
                )  # Light blue bg, black bold text

    def return_to_home(self):
        logger.info("Returning to Home pose after demo.")
        self.ctrl.set_all_positions(self.home_pose)
        QApplication.processEvents()
        idx = getattr(self, "_active_demo_idx", None)
        if idx is not None:
            # Wait for the hand to reach home pose (within tolerance)
            try:
                import threading

                def check_and_uncheck():
                    tolerance = 50  # ticks, adjust as needed
                    for _ in range(20):  # check for up to 2 seconds
                        current = self.ctrl.get_all_positions()
                        if all(
                            abs(a - b) <= tolerance
                            for a, b in zip(current, self.home_pose)
                        ):
                            self._demo_buttons[idx].setChecked(False)
                            break
                        time.sleep(0.1)

                t = threading.Thread(target=check_and_uncheck, daemon=True)
                t.start()
            except Exception as e:
                logger.error(f"Error in home pose check: {e}")
# _________________________ MAIN_GUI _____________________________
class MainWindow(QMainWindow):
    def __init__(self, test_mode=False):
        super().__init__()
        self.setWindowTitle("NIBIB DexHand Control")
        self.resize(1100, 700)
        self.showMaximized()  # Show window in full screen (maximized) by default
        self.tab_widget = QTabWidget()
        self.test_mode = test_mode
        try:
            ctrl = DynamixelController(port_name=Config.PORT_NAME)
        except Exception as e:
            # Log available serial ports for debugging
            try:
                available_ports = [port.device for port in serial.tools.list_ports.comports()]
                logger.error(f"Available serial ports: {available_ports}")
            except Exception as port_exc:
                logger.error(f"Could not list serial ports: {port_exc}")
            QMessageBox.critical(self, "Connection Error", str(e))
            sys.exit(1)
        self.pose_tab = HandPoseControllerWindow(ctrl)
        self.demo_tab = HandDemoWindow(ctrl, test_mode=self.test_mode)
        self.tab_widget.addTab(self.pose_tab, "Pose Controller")
        self.tab_widget.addTab(self.demo_tab, "Demo")
        self.setCentralWidget(self.tab_widget)
        self.tab_widget.setCurrentIndex(1)  # Show Demo tab by default
# _________________________ UI_EVENTS ____________________________
class _UpdateUIEvent(QEvent):
    TYPE = QEvent.Type(QEvent.registerEventType())

    def __init__(self, rel_positions):
        super().__init__(self.TYPE)
        self.rel_positions = rel_positions
# _________________________ MAIN _________________________________
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="NIBIB Hand Control Launcher")
    parser.add_argument(
        "-t",
        "--test",
        action="store_true",
        help="Show Test Sequence in Demo Window for Debugging",
    )
    args = parser.parse_args()

    app = QApplication(sys.argv)
    win = MainWindow(test_mode=args.test)
    win.show()
    sys.exit(app.exec_())
