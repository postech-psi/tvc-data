"""ESC Console - Minimal Performance GUI"""

import math
import os
import sys
import threading
import time
import csv
import random
from datetime import datetime
from collections import deque

import serial
from PySide6 import QtCore, QtWidgets
import pyqtgraph as pg

BAUD = 115200
ESC_MIN, ESC_MAX = 1000, 2000
SEND_HZ = 50  # MCU timeout is 1000ms, 50Hz keeps signal responsive

# Where data_*.csv is written. Empty = the current folder (original behaviour).
# Override per-session without editing this file by setting TVC_CSV_DIR.
CSV_OUT_DIR = ""
PLOT_WIN = 10.0
AVG_WIN = 5.0  # seconds
PLOT_REFRESH_HZ = 30
PLOT_REFRESH_INTERVAL = 1.0 / PLOT_REFRESH_HZ
DEFAULT_PORT = os.environ.get(
    "ESC_CONSOLE_PORT",
    "/dev/cu.usbmodem011" if sys.platform == "darwin" else "",
)
DEFAULT_SUPPLY_V = 10.0
DEFAULT_AIR_DENSITY = 1.225  # kg/m^3
DEFAULT_PROP_DIAMETER_IN = 10.0
INCH_TO_M = 0.0254
RPM_OUTLIER_WINDOW = 5
RPM_OUTLIER_ABS = 5000
RPM_OUTLIER_REL = 0.5
ADC_ZERO_RAW = 807.0
ADC_THIRTY_RAW = 3290.0
ADC_FULLSCALE_A = 30.0
ADC_OFFSET_WINDOW_S = 10.0
ADC_OFFSET_MIN_SAMPLES = 500


def _serial_port_sort_key(port):
    if sys.platform.startswith("win") and port.upper().startswith("COM"):
        suffix = port[3:]
        if suffix.isdigit():
            return (0, int(suffix))
    return (1, port.casefold())


class SerialWorker(QtCore.QObject):
    line_rx = QtCore.Signal(str)
    stat_rx = QtCore.Signal(dict)
    conn_changed = QtCore.Signal(bool, str)
    tx = QtCore.Signal(str)

    def __init__(self):
        super().__init__()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ser = None
        self._last_tx = None

    def connect(self, port):
        try:
            ser = serial.Serial(port, BAUD, timeout=0.1)
            with self._lock:
                self._ser = ser
            self._stop.clear()
            threading.Thread(target=self._rx, daemon=True).start()
            self.conn_changed.emit(True, port)
        except Exception as e:
            self.conn_changed.emit(False, str(e))

    def disconnect(self):
        self._stop.set()
        with self._lock:
            if self._ser:
                self._ser.close()
            self._ser = None
        self.conn_changed.emit(False, "")

    def connected(self):
        with self._lock:
            return self._ser is not None

    def send(self, s, force_log=False):
        should_emit = force_log
        error_msg = None
        with self._lock:
            if s != self._last_tx:
                self._last_tx = s
                should_emit = True
            if self._ser:
                try:
                    self._ser.write((s + "\r\n").encode())
                except Exception as e:
                    error_msg = str(e)
            else:
                error_msg = "not connected"
        if should_emit:
            self.tx.emit(s)
        if error_msg:
            self.tx.emit(f"!! send failed: {error_msg}")

    def _rx(self):
        buf = bytearray()
        while not self._stop.is_set():
            with self._lock:
                ser = self._ser
            if not ser:
                time.sleep(0.05)
                continue
            try:
                data = ser.read(ser.in_waiting or 1)
                if not data:
                    continue
                buf += data
                while b"\n" in buf:
                    idx = buf.find(b"\n")
                    line = buf[:idx].decode(errors="replace").strip()
                    del buf[:idx + 1]
                    if line and line not in ("OK", "OK SET"):
                        self.line_rx.emit(line)
                        if line.startswith("t="):
                            self.stat_rx.emit(self._parse(line))
            except:
                time.sleep(0.1)

    @staticmethod
    def _parse(line):
        d = {}
        for p in line.split():
            if "=" in p:
                k, v = p.split("=", 1)
                if "(" in v:
                    v = v.split("(")[0]
                try:
                    d[k] = float(v) if "." in v else int(v)
                except:
                    d[k] = v
        return d


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESC Console")
        self.resize(1280, 780)

        self.worker = SerialWorker()
        self.worker.conn_changed.connect(self._on_conn)
        self.worker.line_rx.connect(self._on_line)
        self.worker.stat_rx.connect(self._on_stat)
        self.worker.tx.connect(self._on_tx)

        self._armed = False
        self._idle = 1000

        self.send_timer = QtCore.QTimer()
        self.send_timer.setInterval(1000 // SEND_HZ)
        self.send_timer.timeout.connect(self._send_tick)
        self.send_timer.start()
        self.seq_timer = QtCore.QTimer()
        self.seq_timer.setSingleShot(True)
        self.seq_timer.timeout.connect(self._advance_sequence_step)
        self._seq_running = False
        self._seq_values = []
        self._seq_index = 0
        self._seq_hold_ms = 0
        self._plot_dirty = False
        self.plot_refresh_hz = PLOT_REFRESH_HZ
        self._seq_status_text = ""
        self._seq_pre_hold_pending = False
        self._seq_pre_hold_ms = 0
        self._latest_rpm = 0
        self.metrics_t0 = None
        self.metrics_buf = {
            "t": deque(maxlen=2000),
            "pelec": deque(maxlen=2000),
            "pshaft": deque(maxlen=2000),
            "eff": deque(maxlen=2000),
            "loading": deque(maxlen=2000),
            "system_eff": deque(maxlen=2000),
            "ct": deque(maxlen=2000),
            "cp": deque(maxlen=2000),
            "pwm": deque(maxlen=2000),
            "rpm": deque(maxlen=2000),
        }
        self._metrics_dirty = False
        self._rpm_hist = deque(maxlen=RPM_OUTLIER_WINDOW)
        self._rpm_outlier_count = 0
        self._adc_offset_samples = deque(maxlen=3000)
        self._adc_zero_raw = ADC_ZERO_RAW
        self._adc_offset_ready = False
        self._adc_cal_active = False

        # Buffers
        self.t0 = None
        self.buf_t = deque(maxlen=2000)
        self.buf_pwm = deque(maxlen=2000)
        self.buf_rpm = deque(maxlen=2000)
        self.buf_fx = deque(maxlen=2000)
        self.buf_fy = deque(maxlen=2000)
        self.buf_fz = deque(maxlen=2000)
        self.buf_tx = deque(maxlen=2000)
        self.buf_ty = deque(maxlen=2000)
        self.buf_tz = deque(maxlen=2000)
        self.buf_current = deque(maxlen=2000)
        
        # Force/Torque values
        self._fx = self._fy = self._fz = 0.0
        self._tx = self._ty = self._tz = 0.0
        self._current_ma = 0  # telemetry current
        self._current_adc_ma = None  # mapped from ADC raw
        self._current_adc = None
        self._current_cc = None
        
        # Averaging buffer: (t_ms, fx, fy, fz, tx, ty, tz)
        self._avg_buf = deque(maxlen=1500)  # 30s at 50Hz
        self._avg_sum = [0.0] * 9

        # Sampling rate tracking
        self._sample_intervals = deque(maxlen=100)
        self._last_sample_ts = None
        
        # Zero offset
        self._zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # fx,fy,fz,tx,ty,tz
        self._zeroing = False
        self._zero_buf = []
        self._zero_start = 0
        
        # Snapshot for comparison
        self._snap = None  # (fx, fy, fz, tx, ty, tz, rpm, current)
        
        # CSV logging
        self._csv_file = None
        self._csv_writer = None
        self._csv_count = 0
        self._csv_columns = []
        
        # Console/Safe tracking
        self._mcu_state = "INIT"
        self._safe_lines = deque(maxlen=200)

        self._build_ui()
        self._plot_timer = QtCore.QTimer()
        self._plot_timer.timeout.connect(self._render_plots)
        self._apply_plot_timer_interval()
        self._plot_timer.start()
        self._port_refresh_timer = QtCore.QTimer()
        self._port_refresh_timer.setInterval(2000)
        self._port_refresh_timer.timeout.connect(self._refresh_ports_if_disconnected)
        self._port_refresh_timer.start()
        self._refresh_ports()
        self._update_state()

    def _build_ui(self):
        w = QtWidgets.QWidget()
        self.setCentralWidget(w)
        main = QtWidgets.QHBoxLayout(w)
        main.setSpacing(4)
        main.setContentsMargins(4, 4, 4, 4)

        # Left panel
        left = QtWidgets.QVBoxLayout()
        left.setSpacing(4)

        # Connection
        conn_port_row = QtWidgets.QHBoxLayout()
        conn_port_row.setSpacing(6)
        conn_port_row.addWidget(QtWidgets.QLabel("Port"))
        self.port_cb = QtWidgets.QComboBox()
        self.port_cb.setMinimumWidth(160)
        conn_port_row.addWidget(self.port_cb, 1)
        left.addLayout(conn_port_row)

        conn_btn_row = QtWidgets.QHBoxLayout()
        conn_btn_row.setSpacing(6)
        self.btn_refresh = QtWidgets.QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self._refresh_ports)
        conn_btn_row.addWidget(self.btn_refresh)
        self.btn_conn = QtWidgets.QPushButton("Connect")
        self.btn_conn.clicked.connect(self._do_connect)
        conn_btn_row.addWidget(self.btn_conn)
        self.btn_disc = QtWidgets.QPushButton("Disconnect")
        self.btn_disc.clicked.connect(self._do_disconnect)
        conn_btn_row.addWidget(self.btn_disc)
        left.addLayout(conn_btn_row)

        self.lbl_conn = QtWidgets.QLabel("--")
        self.lbl_conn.setStyleSheet("font-size:11px")
        left.addWidget(self.lbl_conn)

        # Safety/Arm
        self.chk_safe = QtWidgets.QCheckBox("Safety")
        self.chk_safe.toggled.connect(self._on_safety_toggled)
        left.addWidget(self.chk_safe)

        arm_row = QtWidgets.QHBoxLayout()
        self.btn_arm = QtWidgets.QPushButton("ARM")
        self.btn_arm.setStyleSheet("background:#4a4;color:white;font-weight:bold")
        self.btn_arm.clicked.connect(self._do_arm)
        arm_row.addWidget(self.btn_arm)
        self.btn_disarm = QtWidgets.QPushButton("DISARM")
        self.btn_disarm.setStyleSheet("background:#a44;color:white;font-weight:bold")
        self.btn_disarm.clicked.connect(self._do_disarm)
        arm_row.addWidget(self.btn_disarm)
        self.lbl_pwm_status = QtWidgets.QLabel(f"PWM: {self._idle}")
        self.lbl_pwm_status.setStyleSheet("font-size:11px;font-weight:bold")
        arm_row.addStretch(1)
        arm_row.addWidget(self.lbl_pwm_status)
        left.addLayout(arm_row)

        self.lbl_status = QtWidgets.QLabel("STATE: LOCKED")
        self.lbl_status.setStyleSheet("font-weight:bold")
        self.lbl_status.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        left.addWidget(self.lbl_status)

        left.addSpacing(4)

        # Live telemetry box
        self.lbl_mcu = QtWidgets.QLabel("--")
        self.lbl_mcu.setStyleSheet("font-size:11px")
        self.lbl_rate = QtWidgets.QLabel("Rate: -- Hz")
        self.lbl_rate.setStyleSheet("font-size:11px")
        self.lbl_rpm = QtWidgets.QLabel("RPM: --")
        self.lbl_rpm.setStyleSheet("font-size:11px")
        self.lbl_rpm_outliers = QtWidgets.QLabel("RPM outliers: 0")
        self.lbl_rpm_outliers.setStyleSheet("font-size:10px;color:#888")
        self.lbl_current = QtWidgets.QLabel("Current: -- mA")
        self.lbl_current.setStyleSheet("font-size:11px")
        self.lbl_current_adc = QtWidgets.QLabel("ADC current: -- mA")
        self.lbl_current_adc.setStyleSheet("font-size:11px;color:#8af")
        self.lbl_adc_offset = QtWidgets.QLabel("ADC offset: waiting")
        self.lbl_adc_offset.setStyleSheet("font-size:10px;color:#888")
        self.btn_adc_cal = QtWidgets.QPushButton("Calibrate ADC")
        self.btn_adc_cal.setFixedHeight(20)
        self.btn_adc_cal.setStyleSheet("font-size:10px")
        self.btn_adc_cal.clicked.connect(self._start_adc_calibration)
        self.lbl_live_force = QtWidgets.QLabel("Force: -- -- -- N")
        self.lbl_live_force.setStyleSheet("font-family:monospace;font-size:11px")
        self.lbl_live_torque = QtWidgets.QLabel("Torque: -- -- -- Nm")
        self.lbl_live_torque.setStyleSheet("font-family:monospace;font-size:11px")
        self.lbl_adc = QtWidgets.QLabel("CC: -- ADC: --")
        self.lbl_adc.setStyleSheet("font-size:11px")
        self.lbl_ft_counts = QtWidgets.QLabel("FT cnt: -- / --")
        self.lbl_ft_counts.setStyleSheet("font-size:11px")

        live_box = QtWidgets.QGroupBox("Live values")
        live_layout = QtWidgets.QGridLayout(live_box)
        live_layout.setContentsMargins(6, 4, 6, 4)
        live_layout.setHorizontalSpacing(8)
        live_layout.setVerticalSpacing(2)
        live_layout.addWidget(self.lbl_mcu, 0, 0, 1, 2)
        live_layout.addWidget(self.lbl_rate, 1, 0, 1, 2)
        live_layout.addWidget(self.lbl_rpm, 2, 0)
        live_layout.addWidget(self.lbl_current, 2, 1)
        live_layout.addWidget(self.lbl_current_adc, 3, 0, 1, 2)
        live_layout.addWidget(self.lbl_adc_offset, 4, 0, 1, 2)
        live_layout.addWidget(self.lbl_rpm_outliers, 5, 0, 1, 2)
        live_layout.addWidget(self.lbl_live_force, 6, 0, 1, 2)
        live_layout.addWidget(self.lbl_live_torque, 7, 0, 1, 2)
        live_layout.addWidget(self.lbl_adc, 8, 0)
        live_layout.addWidget(self.lbl_ft_counts, 8, 1)
        live_layout.addWidget(self.btn_adc_cal, 9, 0, 1, 2)
        left.addWidget(live_box)

        left.addSpacing(8)

        # Average values box
        self.spin_avg = QtWidgets.QDoubleSpinBox()
        self.spin_avg.setRange(0.5, 30.0)
        self.spin_avg.setValue(AVG_WIN)
        self.spin_avg.setSuffix("s")
        self.spin_avg.setSingleStep(1.0)
        self.lbl_avg_n = QtWidgets.QLabel("(0)")
        self.lbl_avg_n.setStyleSheet("font-size:10px;color:#888")
        self.lbl_avg_f = QtWidgets.QLabel("F: -- -- --")
        self.lbl_avg_f.setStyleSheet("font-size:11px;color:#8af")
        self.lbl_avg_t = QtWidgets.QLabel("T: -- -- --")
        self.lbl_avg_t.setStyleSheet("font-size:11px;color:#8af")
        self.lbl_avg_rpm = QtWidgets.QLabel("RPM: --")
        self.lbl_avg_rpm.setStyleSheet("font-size:11px;color:#8af")
        self.lbl_avg_current = QtWidgets.QLabel("Current: -- A")
        self.lbl_avg_current.setStyleSheet("font-size:11px;color:#8af")
        self.lbl_avg_current_adc = QtWidgets.QLabel("ADC: -- A")
        self.lbl_avg_current_adc.setStyleSheet("font-size:11px;color:#8af")

        avg_box = QtWidgets.QGroupBox("Averages")
        avg_layout = QtWidgets.QVBoxLayout(avg_box)
        avg_layout.setContentsMargins(6, 4, 6, 4)
        avg_layout.setSpacing(2)
        spin_row = QtWidgets.QHBoxLayout()
        spin_row.addWidget(QtWidgets.QLabel("Window"))
        spin_row.addWidget(self.spin_avg)
        spin_row.addWidget(self.lbl_avg_n)
        spin_row.addStretch(1)
        avg_layout.addLayout(spin_row)
        avg_layout.addWidget(self.lbl_avg_f)
        avg_layout.addWidget(self.lbl_avg_t)
        avg_layout.addWidget(self.lbl_avg_rpm)
        avg_current_row = QtWidgets.QHBoxLayout()
        avg_current_row.setSpacing(8)
        avg_current_row.addWidget(self.lbl_avg_current)
        avg_current_row.addWidget(self.lbl_avg_current_adc)
        avg_current_row.addStretch(1)
        avg_layout.addLayout(avg_current_row)
        left.addWidget(avg_box)

        left.addSpacing(8)

        # Snapshot box
        snap_box = QtWidgets.QGroupBox("Snapshot")
        snap_layout = QtWidgets.QVBoxLayout(snap_box)
        snap_layout.setContentsMargins(6, 4, 6, 4)
        snap_btn_row = QtWidgets.QHBoxLayout()
        self.btn_snap = QtWidgets.QPushButton("Snap")
        self.btn_snap.clicked.connect(self._do_snap)
        snap_btn_row.addWidget(self.btn_snap)
        self.btn_snap_clear = QtWidgets.QPushButton("Clear")
        self.btn_snap_clear.clicked.connect(self._clear_snap)
        snap_btn_row.addWidget(self.btn_snap_clear)
        snap_btn_row.addStretch(1)
        snap_layout.addLayout(snap_btn_row)
        self.lbl_snap = QtWidgets.QLabel("Snap: F -- | T -- | RPM -- | mA --")
        self.lbl_snap.setStyleSheet("font-size:10px;color:#fa0")
        snap_layout.addWidget(self.lbl_snap)
        self.lbl_delta = QtWidgets.QLabel("Δ: F -- | T -- | RPM -- | mA --")
        self.lbl_delta.setStyleSheet("font-size:10px;color:#0f0")
        snap_layout.addWidget(self.lbl_delta)
        left.addWidget(snap_box)

        # Zero/Tare
        left.addSpacing(8)
        zero_row = QtWidgets.QHBoxLayout()
        self.btn_zero = QtWidgets.QPushButton("Zero (10s)")
        self.btn_zero.clicked.connect(self._start_zero)
        zero_row.addWidget(self.btn_zero)
        self.btn_clear_zero = QtWidgets.QPushButton("Clear")
        self.btn_clear_zero.clicked.connect(self._clear_zero)
        zero_row.addWidget(self.btn_clear_zero)
        left.addLayout(zero_row)
        self.lbl_zero = QtWidgets.QLabel("Zero: OFF")
        self.lbl_zero.setStyleSheet("font-size:10px;color:#888")
        left.addWidget(self.lbl_zero)

        # CSV logging
        left.addSpacing(8)
        csv_row = QtWidgets.QHBoxLayout()
        self.btn_csv = QtWidgets.QPushButton("▶ CSV")
        self.btn_csv.clicked.connect(self._toggle_csv)
        csv_row.addWidget(self.btn_csv)
        left.addLayout(csv_row)
        self.lbl_csv = QtWidgets.QLabel("CSV: OFF")
        self.lbl_csv.setStyleSheet("font-size:10px;color:#888")
        left.addWidget(self.lbl_csv)
        csv_opt_row = QtWidgets.QHBoxLayout()
        csv_opt_row.addWidget(QtWidgets.QLabel("Mode"))
        self.csv_mode_cb = QtWidgets.QComboBox()
        self.csv_mode_cb.addItems(["Standard", "Forces only", "Signals only"])
        csv_opt_row.addWidget(self.csv_mode_cb, 1)
        left.addLayout(csv_opt_row)

        left.addSpacing(6)

        # Console (bottom left, hidden when armed)
        self.console_box = QtWidgets.QWidget()
        console_lay = QtWidgets.QVBoxLayout(self.console_box)
        console_lay.setContentsMargins(0, 0, 0, 0)
        console_lay.setSpacing(0)
        console_lbl = QtWidgets.QLabel("Console:")
        console_lbl.setStyleSheet("font-size:11px")
        console_lay.addWidget(console_lbl)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(200)
        self.log.setStyleSheet("font-family:monospace;font-size:10px")
        self.log.setMinimumHeight(80)
        console_lay.addWidget(self.log)
        self.console_safe_msg = QtWidgets.QPlainTextEdit()
        self.console_safe_msg.setReadOnly(True)
        self.console_safe_msg.setMaximumBlockCount(500)
        self.console_safe_msg.setStyleSheet("font-family:monospace;font-size:10px;color:#333;background:#eee")
        self.console_safe_msg.setVisible(False)
        self.console_safe_msg.setMinimumHeight(80)
        self.console_safe_msg.setPlainText("Console hidden while ARMED\nDisarm (SAFE) to view logs.")
        console_lay.addWidget(self.console_safe_msg)
        left.addWidget(self.console_box, 1)
        self.console_box.setVisible(False)

        left_w = QtWidgets.QWidget()
        left_w.setLayout(left)
        left_w.setFixedWidth(250)
        main.addWidget(left_w)

        # Right: graphs
        right = QtWidgets.QVBoxLayout()
        right.setSpacing(2)

        # Control tabs (Manual / Sequence)
        self.control_tabs = QtWidgets.QTabWidget()
        self.control_tabs.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Maximum)
        manual_tab = QtWidgets.QWidget()
        manual_layout = QtWidgets.QHBoxLayout(manual_tab)
        manual_layout.setContentsMargins(6, 4, 6, 4)
        manual_layout.setSpacing(6)
        self.lbl_pwm_slider = QtWidgets.QLabel(f"PWM: {self._idle}")
        self.lbl_pwm_slider.setStyleSheet("font-size:14px;font-weight:bold")
        self.lbl_pwm_slider.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self.lbl_pwm_slider.setFixedWidth(80)
        manual_layout.addWidget(self.lbl_pwm_slider)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(ESC_MIN, ESC_MAX)
        self.slider.setValue(self._idle)
        self.slider.setFixedHeight(18)
        self.slider.valueChanged.connect(self._on_pwm_changed)
        manual_layout.addWidget(self.slider, 1)
        self.control_tabs.addTab(manual_tab, "Manual")

        seq_tab = QtWidgets.QWidget()
        seq_layout = QtWidgets.QVBoxLayout(seq_tab)
        seq_layout.setContentsMargins(12, 10, 12, 10)
        seq_layout.setSpacing(10)

        top_row = QtWidgets.QHBoxLayout()
        top_row.setSpacing(10)

        sweep_box = QtWidgets.QGroupBox("Step Sweep")
        sweep_layout = QtWidgets.QGridLayout(sweep_box)
        sweep_layout.setContentsMargins(12, 10, 12, 10)
        sweep_layout.setHorizontalSpacing(12)
        sweep_layout.setVerticalSpacing(8)
        self.spin_seq_start = QtWidgets.QSpinBox()
        self.spin_seq_start.setRange(ESC_MIN, ESC_MAX)
        self.spin_seq_start.setValue(self._idle)
        self.spin_seq_start.setSingleStep(10)
        self.spin_seq_step = QtWidgets.QSpinBox()
        self.spin_seq_step.setRange(-500, 500)
        self.spin_seq_step.setSingleStep(10)
        self.spin_seq_step.setValue(10)
        self.spin_seq_steps = QtWidgets.QSpinBox()
        self.spin_seq_steps.setRange(1, 500)
        default_steps = int(abs(ESC_MAX - ESC_MIN) / max(1, abs(self.spin_seq_step.value()))) + 1
        self.spin_seq_steps.setValue(default_steps)
        self.spin_seq_hold = QtWidgets.QDoubleSpinBox()
        self.spin_seq_hold.setRange(0.5, 60.0)
        self.spin_seq_hold.setSingleStep(0.5)
        self.spin_seq_hold.setValue(10.0)
        self.spin_seq_hold.setSuffix(" s")
        sweep_pairs = [
            ("Start PWM", self.spin_seq_start),
            ("Step ΔPWM", self.spin_seq_step),
            ("Steps", self.spin_seq_steps),
            ("Hold per step", self.spin_seq_hold),
        ]
        for idx, (label, widget) in enumerate(sweep_pairs):
            row = idx // 2
            col = (idx % 2) * 2
            sweep_layout.addWidget(QtWidgets.QLabel(label), row, col, QtCore.Qt.AlignRight)
            sweep_layout.addWidget(widget, row, col + 1)
        sweep_note = QtWidgets.QLabel("Use this to create linear step sweeps.")
        sweep_note.setStyleSheet("font-size:10px;color:#aaa")
        sweep_layout.addWidget(sweep_note, 4, 0, 1, 2)
        top_row.addWidget(sweep_box, 1)

        preset_box = QtWidgets.QGroupBox("Preset Tests")
        preset_layout = QtWidgets.QGridLayout(preset_box)
        preset_layout.setContentsMargins(12, 10, 12, 10)
        preset_layout.setHorizontalSpacing(12)
        preset_layout.setVerticalSpacing(8)
        self.preset_mode_cb = QtWidgets.QComboBox()
        self.preset_mode_cb.addItems(["Motor Check", "Prop Ramp", "Endurance"])
        self.spin_preset_peak = QtWidgets.QSpinBox()
        self.spin_preset_peak.setRange(10, 500)
        self.spin_preset_peak.setValue(100)
        self.spin_preset_peak.setSingleStep(10)
        self.spin_preset_hold = QtWidgets.QDoubleSpinBox()
        self.spin_preset_hold.setRange(0.2, 20.0)
        self.spin_preset_hold.setSingleStep(0.2)
        self.spin_preset_hold.setValue(1.5)
        self.spin_preset_loops = QtWidgets.QSpinBox()
        self.spin_preset_loops.setRange(1, 10)
        self.spin_preset_loops.setValue(1)
        preset_pairs = [
            ("Profile", self.preset_mode_cb),
            ("Peak ΔPWM", self.spin_preset_peak),
            ("Hold (s)", self.spin_preset_hold),
            ("Loops", self.spin_preset_loops),
        ]
        for idx, (label, widget) in enumerate(preset_pairs):
            row = idx // 2
            col = (idx % 2) * 2
            preset_layout.addWidget(QtWidgets.QLabel(label), row, col, QtCore.Qt.AlignRight)
            preset_layout.addWidget(widget, row, col + 1)
        preset_btns = QtWidgets.QHBoxLayout()
        self.btn_preset_run = QtWidgets.QPushButton("Run Preset")
        self.btn_preset_run.clicked.connect(self._run_selected_preset)
        preset_btns.addWidget(self.btn_preset_run)
        preset_btns.addStretch(1)
        preset_layout.addLayout(preset_btns, 4, 0, 1, 2)
        top_row.addWidget(preset_box, 1)

        seq_layout.addLayout(top_row)

        options_box = QtWidgets.QGroupBox("Options")
        options_layout = QtWidgets.QGridLayout(options_box)
        options_layout.setContentsMargins(12, 8, 12, 8)
        options_layout.setHorizontalSpacing(12)
        options_layout.setVerticalSpacing(4)
        self.spin_seq_loops = QtWidgets.QSpinBox()
        self.spin_seq_loops.setRange(1, 10)
        self.spin_seq_loops.setValue(1)
        self.spin_seq_idle_hold = QtWidgets.QDoubleSpinBox()
        self.spin_seq_idle_hold.setRange(0.0, 10.0)
        self.spin_seq_idle_hold.setSingleStep(0.5)
        self.spin_seq_idle_hold.setValue(0.0)
        self.spin_seq_jitter = QtWidgets.QDoubleSpinBox()
        self.spin_seq_jitter.setRange(0.0, 100.0)
        self.spin_seq_jitter.setSingleStep(1.0)
        self.spin_seq_jitter.setValue(0.0)
        option_pairs = [
            ("Repeat", self.spin_seq_loops),
            ("Idle hold (s)", self.spin_seq_idle_hold),
            ("Jitter ±PWM", self.spin_seq_jitter),
        ]
        for idx, (label, widget) in enumerate(option_pairs):
            row = idx // 2
            col = (idx % 2) * 2
            options_layout.addWidget(QtWidgets.QLabel(label), row, col, QtCore.Qt.AlignRight)
            options_layout.addWidget(widget, row, col + 1)
        self.chk_seq_reverse = QtWidgets.QCheckBox("Return sweep")
        options_layout.addWidget(self.chk_seq_reverse, 2, 0, 1, 4, QtCore.Qt.AlignRight)
        seq_layout.addWidget(options_box)

        btn_row = QtWidgets.QHBoxLayout()
        btn_row.setSpacing(8)
        self.btn_seq_run = QtWidgets.QPushButton("Run Sequence")
        self.btn_seq_run.clicked.connect(self._start_sequence)
        btn_row.addWidget(self.btn_seq_run)
        self.btn_seq_stop = QtWidgets.QPushButton("Stop")
        self.btn_seq_stop.setEnabled(False)
        self.btn_seq_stop.clicked.connect(lambda: self._stop_sequence("stopped"))
        btn_row.addWidget(self.btn_seq_stop)
        seq_layout.addLayout(btn_row)
        self.lbl_seq_status = QtWidgets.QLabel("Sequence idle")
        self.lbl_seq_status.setStyleSheet("font-size:10px;color:#888")
        seq_layout.addWidget(self.lbl_seq_status)
        self.control_tabs.addTab(seq_tab, "Sequence")

        self.metrics_live_labels = {}
        self.metrics_avg_labels = {}

        metrics_tab = QtWidgets.QWidget()
        metrics_layout = QtWidgets.QVBoxLayout(metrics_tab)
        metrics_layout.setContentsMargins(12, 10, 12, 10)
        metrics_layout.setSpacing(8)

        metrics_pwm_row = QtWidgets.QHBoxLayout()
        metrics_pwm_row.setSpacing(8)
        self.lbl_metrics_pwm = QtWidgets.QLabel(f"PWM: {self._idle}")
        self.lbl_metrics_pwm.setStyleSheet("font-size:12px;font-weight:bold")
        metrics_pwm_row.addWidget(self.lbl_metrics_pwm)
        self.metrics_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.metrics_slider.setRange(ESC_MIN, ESC_MAX)
        self.metrics_slider.setValue(self._idle)
        self.metrics_slider.setFixedHeight(18)
        self.metrics_slider.valueChanged.connect(self._on_pwm_changed)
        metrics_pwm_row.addWidget(self.metrics_slider, 1)
        metrics_layout.addLayout(metrics_pwm_row)

        metrics_upper = QtWidgets.QHBoxLayout()
        metrics_upper.setSpacing(10)

        metrics_input_box = QtWidgets.QGroupBox("Inputs")
        metrics_input_box.setMaximumWidth(320)
        metrics_input_layout = QtWidgets.QGridLayout(metrics_input_box)
        metrics_input_layout.setContentsMargins(10, 8, 10, 8)
        metrics_input_layout.setHorizontalSpacing(10)
        metrics_input_layout.setVerticalSpacing(6)
        self.spin_metrics_voltage = QtWidgets.QDoubleSpinBox()
        self.spin_metrics_voltage.setRange(0.1, 100.0)
        self.spin_metrics_voltage.setValue(DEFAULT_SUPPLY_V)
        self.spin_metrics_voltage.setSuffix(" V")
        metrics_input_layout.addWidget(QtWidgets.QLabel("Supply voltage"), 0, 0, QtCore.Qt.AlignRight)
        metrics_input_layout.addWidget(self.spin_metrics_voltage, 0, 1)
        self.spin_metrics_density = QtWidgets.QDoubleSpinBox()
        self.spin_metrics_density.setRange(0.5, 2.5)
        self.spin_metrics_density.setValue(DEFAULT_AIR_DENSITY)
        self.spin_metrics_density.setDecimals(4)
        self.spin_metrics_density.setSuffix(" kg/m³")
        metrics_input_layout.addWidget(QtWidgets.QLabel("Air density"), 0, 2, QtCore.Qt.AlignRight)
        metrics_input_layout.addWidget(self.spin_metrics_density, 0, 3)
        self.spin_metrics_diameter = QtWidgets.QDoubleSpinBox()
        self.spin_metrics_diameter.setRange(2.0, 40.0)
        self.spin_metrics_diameter.setValue(DEFAULT_PROP_DIAMETER_IN)
        self.spin_metrics_diameter.setDecimals(4)
        self.spin_metrics_diameter.setSuffix(" in")
        metrics_input_layout.addWidget(QtWidgets.QLabel("Prop diameter"), 1, 0, QtCore.Qt.AlignRight)
        metrics_input_layout.addWidget(self.spin_metrics_diameter, 1, 1)
        self.combo_metrics_torque = QtWidgets.QComboBox()
        self.combo_metrics_torque.addItems(["Tx", "Ty", "Tz"])
        metrics_input_layout.addWidget(QtWidgets.QLabel("Torque axis"), 1, 2, QtCore.Qt.AlignRight)
        metrics_input_layout.addWidget(self.combo_metrics_torque, 1, 3)
        self.combo_metrics_force = QtWidgets.QComboBox()
        self.combo_metrics_force.addItems(["Fx", "Fy", "Fz"])
        metrics_input_layout.addWidget(QtWidgets.QLabel("Thrust axis"), 2, 0, QtCore.Qt.AlignRight)
        metrics_input_layout.addWidget(self.combo_metrics_force, 2, 1)

        metrics_values_box = QtWidgets.QGroupBox("Calculated metrics")
        values_grid = QtWidgets.QGridLayout(metrics_values_box)
        values_grid.setContentsMargins(12, 8, 12, 8)
        values_grid.setHorizontalSpacing(12)
        values_grid.setVerticalSpacing(6)

        values_grid.addWidget(QtWidgets.QLabel("Avg window"), 0, 0, QtCore.Qt.AlignRight)
        self.spin_metrics_avg = QtWidgets.QDoubleSpinBox()
        self.spin_metrics_avg.setRange(0.5, 30.0)
        self.spin_metrics_avg.setValue(AVG_WIN)
        self.spin_metrics_avg.setSuffix("s")
        self.spin_metrics_avg.setSingleStep(1.0)
        values_grid.addWidget(self.spin_metrics_avg, 0, 1)
        values_grid.addWidget(QtWidgets.QLabel("Live"), 1, 1, QtCore.Qt.AlignCenter)
        values_grid.addWidget(QtWidgets.QLabel("Avg"), 1, 2, QtCore.Qt.AlignCenter)
        values_grid.addWidget(QtWidgets.QLabel("Live"), 1, 4, QtCore.Qt.AlignCenter)
        values_grid.addWidget(QtWidgets.QLabel("Avg"), 1, 5, QtCore.Qt.AlignCenter)

        metric_items = [
            ("Electrical input power (W)", "pelec"),
            ("Mechanical shaft power (W)", "pshaft"),
            ("ESC + Motor efficiency (%)", "eff"),
            ("System efficiency (g/W)", "system"),
            ("Prop power loading (g/W)", "loading"),
            ("Thrust coefficient CT", "ct"),
            ("Power coefficient CP", "cp"),
        ]
        self.metric_labels = {}
        self.metrics_avg_labels = {}
        for idx, (label, key) in enumerate(metric_items):
            row = 2 + (idx // 2)
            col = 0 if idx % 2 == 0 else 3
            values_grid.addWidget(QtWidgets.QLabel(label), row, col, QtCore.Qt.AlignRight)
            lbl = QtWidgets.QLabel("--")
            lbl.setStyleSheet("font-family:monospace")
            values_grid.addWidget(lbl, row, col + 1)
            self.metric_labels[key] = lbl
            avg_lbl = QtWidgets.QLabel("--")
            avg_lbl.setStyleSheet("font-family:monospace;color:#8af")
            values_grid.addWidget(avg_lbl, row, col + 2)
            self.metrics_avg_labels[key] = avg_lbl

        metrics_upper.addWidget(metrics_input_box, 0)
        metrics_upper.addWidget(metrics_values_box, 1)
        metrics_layout.addLayout(metrics_upper, 0)

        self.metric_toggle_defs = [
            ("pelec", "Pelec"),
            ("pshaft", "Pshaft"),
            ("eff", "Efficiency"),
            ("system_eff", "System"),
            ("loading", "Loading"),
            ("ct", "CT"),
            ("cp", "CP"),
            ("pwm", "PWM"),
            ("rpm", "RPM"),
        ]
        self.metric_plot_groups = [
            ("power", "Power (W)", [("pelec", "Pelec", "#fdd835"), ("pshaft", "Pshaft", "#26c6da")]),
            ("eff", "ESC + Motor Efficiency (%)", [("eff", "Efficiency", "#66bb6a")]),
            ("system", "System Efficiency (g/W)", [("system_eff", "System", "#ff7043")]),
            ("loading", "Power Loading (g/W)", [("loading", "Loading", "#ab47bc")]),
            ("coeff", "Thrust / Power Coefficients", [("ct", "CT", "#42a5f5"), ("cp", "CP", "#ef5350")]),
            ("signals", "PWM / RPM", [("pwm", "PWM", "#9ccc65"), ("rpm", "RPM", "#ffd54f")]),
        ]
        self.metric_plot_checks = {}
        self.metric_group_plots = {}
        self.metric_curve_plots = {}
        self.metric_curves = {}

        metrics_toggle_row = QtWidgets.QHBoxLayout()
        metrics_toggle_row.setSpacing(6)
        metrics_toggle_row.addWidget(QtWidgets.QLabel("Plot Hz"))
        self.spin_metrics_plot_hz = QtWidgets.QSpinBox()
        self.spin_metrics_plot_hz.setRange(5, 120)
        self.spin_metrics_plot_hz.setValue(self.plot_refresh_hz)
        self.spin_metrics_plot_hz.valueChanged.connect(self._on_plot_rate_changed)
        metrics_toggle_row.addWidget(self.spin_metrics_plot_hz)
        metrics_toggle_row.addSpacing(8)
        metrics_toggle_row.addWidget(QtWidgets.QLabel("Metric plots:"))
        for key, label in self.metric_toggle_defs:
            chk = QtWidgets.QCheckBox(label)
            chk.setChecked(True)
            metrics_toggle_row.addWidget(chk)
            self.metric_plot_checks[key] = chk
        metrics_toggle_row.addStretch(1)
        metrics_layout.addLayout(metrics_toggle_row)

        def _setup_metrics_plot(widget, title):
            widget.setTitle(title)
            widget.showGrid(x=True, y=True, alpha=0.2)
            widget.enableAutoRange(axis='x', enable=False)
            widget.enableAutoRange(axis='y', enable=True)
            widget.addLegend(offset=(60, 10))
            widget.setMenuEnabled(False)
            widget.setMouseEnabled(x=False, y=False)

        metrics_plot_box = QtWidgets.QGroupBox("Metric plots")
        metrics_plot_layout = QtWidgets.QGridLayout(metrics_plot_box)
        metrics_plot_layout.setContentsMargins(6, 6, 6, 6)
        metrics_plot_layout.setHorizontalSpacing(8)
        metrics_plot_layout.setVerticalSpacing(8)

        for idx, (group_key, title, curves) in enumerate(self.metric_plot_groups):
            plot = pg.PlotWidget()
            _setup_metrics_plot(plot, title)
            self.metric_group_plots[group_key] = plot
            for metric_key, name, color in curves:
                curve = plot.plot(pen=pg.mkPen(color, width=1), name=name)
                self.metric_curves[metric_key] = curve
                self.metric_curve_plots[metric_key] = plot
                self._bind_curve_toggle(self.metric_plot_checks.get(metric_key), curve)
            row = idx // 2
            col = idx % 2
            metrics_plot_layout.addWidget(plot, row, col)

        metrics_plot_layout.setColumnStretch(0, 1)
        metrics_plot_layout.setColumnStretch(1, 1)
        for row in range((len(self.metric_plot_groups) + 1) // 2):
            metrics_plot_layout.setRowStretch(row, 1)

        metrics_layout.addWidget(metrics_plot_box, 1)

        self.control_tabs.addTab(metrics_tab, "Metrics")
        self.manual_tab_height = 105
        self.sequence_tab_min_height = 400
        self.control_tabs.currentChanged.connect(self._on_control_tab_changed)
        self._on_control_tab_changed(self.control_tabs.currentIndex())

        right.addWidget(self.control_tabs, 0)

        # Panel visibility toggles
        self.chk_plot_vis_pwm = QtWidgets.QCheckBox("PWM plot")
        self.chk_plot_vis_pwm.setChecked(True)
        self.chk_plot_vis_force = QtWidgets.QCheckBox("Force plot")
        self.chk_plot_vis_force.setChecked(True)
        self.chk_plot_vis_torque = QtWidgets.QCheckBox("Torque plot")
        self.chk_plot_vis_torque.setChecked(True)
        self.chk_plot_vis_current = QtWidgets.QCheckBox("Current plot")
        self.chk_plot_vis_current.setChecked(True)

        # Trace toggles
        self.chk_pwm = QtWidgets.QCheckBox("PWM")
        self.chk_pwm.setChecked(True)
        self.chk_rpm = QtWidgets.QCheckBox("RPM")
        self.chk_rpm.setChecked(True)
        self.chk_current = QtWidgets.QCheckBox("Current")
        self.chk_current.setChecked(True)
        self.chk_fx = QtWidgets.QCheckBox("Fx")
        self.chk_fx.setChecked(True)
        self.chk_fy = QtWidgets.QCheckBox("Fy")
        self.chk_fy.setChecked(True)
        self.chk_fz = QtWidgets.QCheckBox("Fz")
        self.chk_fz.setChecked(True)
        self.chk_tx = QtWidgets.QCheckBox("Tx")
        self.chk_tx.setChecked(True)
        self.chk_ty = QtWidgets.QCheckBox("Ty")
        self.chk_ty.setChecked(True)
        self.chk_tz = QtWidgets.QCheckBox("Tz")
        self.chk_tz.setChecked(True)

        # Plot refresh + toggles row
        self.plot_panel = QtWidgets.QWidget()
        plot_panel_layout = QtWidgets.QVBoxLayout(self.plot_panel)
        plot_panel_layout.setContentsMargins(0, 0, 0, 0)
        plot_panel_layout.setSpacing(6)
        plot_ctrl = QtWidgets.QHBoxLayout()
        plot_ctrl.setSpacing(6)
        plot_ctrl.addWidget(QtWidgets.QLabel("Plot Hz"))
        self.spin_plot_hz = QtWidgets.QSpinBox()
        self.spin_plot_hz.setRange(5, 120)
        self.spin_plot_hz.setValue(self.plot_refresh_hz)
        self.spin_plot_hz.valueChanged.connect(self._on_plot_rate_changed)
        plot_ctrl.addWidget(self.spin_plot_hz)
        plot_ctrl.addSpacing(8)
        plot_ctrl.addWidget(QtWidgets.QLabel("Panels:"))
        for chk in (self.chk_plot_vis_pwm, self.chk_plot_vis_force, self.chk_plot_vis_torque, self.chk_plot_vis_current):
            plot_ctrl.addWidget(chk)
        plot_ctrl.addSpacing(8)
        plot_ctrl.addWidget(QtWidgets.QLabel("Traces:"))
        for chk in (self.chk_pwm, self.chk_rpm, self.chk_current, self.chk_fx, self.chk_fy, self.chk_fz, self.chk_tx, self.chk_ty, self.chk_tz):
            plot_ctrl.addWidget(chk)
        plot_ctrl.addStretch(1)
        plot_panel_layout.addLayout(plot_ctrl)

        # Plots
        pg.setConfigOptions(antialias=False)

        def _setup_plot(widget, title):
            widget.setTitle(title)
            widget.showGrid(x=True, y=True, alpha=0.2)
            widget.enableAutoRange(axis='x', enable=False)
            widget.addLegend(offset=(60, 10))
            widget.setMenuEnabled(False)
            widget.setMouseEnabled(x=False, y=False)

        # PWM plot
        self.plot_pwm = pg.PlotWidget()
        _setup_plot(self.plot_pwm, "PWM / RPM")
        self.curve_pwm = self.plot_pwm.plot(pen=pg.mkPen('g', width=1), name='PWM')
        self.curve_rpm = self.plot_pwm.plot(pen=pg.mkPen('y', width=1), name='RPM')
        self._bind_curve_toggle(self.chk_pwm, self.curve_pwm)
        self._bind_curve_toggle(self.chk_rpm, self.curve_rpm)

        # Force plot (Fx, Fy, Fz)
        self.plot_f = pg.PlotWidget()
        _setup_plot(self.plot_f, "Force (N)")
        self.curve_fx = self.plot_f.plot(pen=pg.mkPen('r', width=1), name='Fx')
        self.curve_fy = self.plot_f.plot(pen=pg.mkPen('g', width=1), name='Fy')
        self.curve_fz = self.plot_f.plot(pen=pg.mkPen('b', width=1), name='Fz')
        self._bind_curve_toggle(self.chk_fx, self.curve_fx)
        self._bind_curve_toggle(self.chk_fy, self.curve_fy)
        self._bind_curve_toggle(self.chk_fz, self.curve_fz)

        # Torque plot (Tx, Ty, Tz)
        self.plot_t = pg.PlotWidget()
        _setup_plot(self.plot_t, "Torque (Nm)")
        self.curve_tx = self.plot_t.plot(pen=pg.mkPen('r', width=1), name='Tx')
        self.curve_ty = self.plot_t.plot(pen=pg.mkPen('g', width=1), name='Ty')
        self.curve_tz = self.plot_t.plot(pen=pg.mkPen('b', width=1), name='Tz')
        self._bind_curve_toggle(self.chk_tx, self.curve_tx)
        self._bind_curve_toggle(self.chk_ty, self.curve_ty)
        self._bind_curve_toggle(self.chk_tz, self.curve_tz)

        # Current plot
        self.plot_current = pg.PlotWidget()
        _setup_plot(self.plot_current, "Current (mA)")
        self.curve_current = self.plot_current.plot(pen=pg.mkPen('c', width=1), name='mA')
        self._bind_curve_toggle(self.chk_current, self.curve_current)

        self._bind_plot_toggle(self.chk_plot_vis_pwm, self.plot_pwm)
        self._bind_plot_toggle(self.chk_plot_vis_force, self.plot_f)
        self._bind_plot_toggle(self.chk_plot_vis_torque, self.plot_t)
        self._bind_plot_toggle(self.chk_plot_vis_current, self.plot_current)

        left_split = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        left_split.setChildrenCollapsible(False)
        left_split.addWidget(self.plot_pwm)
        left_split.addWidget(self.plot_t)
        right_split = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        right_split.setChildrenCollapsible(False)
        right_split.addWidget(self.plot_f)
        right_split.addWidget(self.plot_current)
        self.plot_pwm.setMinimumHeight(120)
        self.plot_f.setMinimumHeight(120)
        self.plot_t.setMinimumHeight(120)
        self.plot_current.setMinimumHeight(120)
        plots_split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        plots_split.setChildrenCollapsible(False)
        plots_split.addWidget(left_split)
        plots_split.addWidget(right_split)
        plots_split.setStretchFactor(0, 1)
        plots_split.setStretchFactor(1, 1)

        plot_panel_layout.addWidget(plots_split, 1)

        right.addWidget(self.plot_panel, 1)

        main.addLayout(right, 2)

    def _bind_curve_toggle(self, checkbox, curve):
        if not checkbox:
            return
        if curve:
            checkbox.toggled.connect(curve.setVisible)
            curve.setVisible(checkbox.isChecked())
        else:
            checkbox.setEnabled(False)

    def _bind_plot_toggle(self, checkbox, widget):
        if not checkbox or not widget:
            return
        checkbox.toggled.connect(widget.setVisible)
        widget.setVisible(checkbox.isChecked())

    def _on_control_tab_changed(self, index):
        if not hasattr(self, "control_tabs"):
            return
        if index in (1, 2):
            tab = self.control_tabs.widget(index)
            if tab and tab.layout():
                tab.layout().activate()
                tab.adjustSize()
            hint = tab.sizeHint().height() if tab else 0
            max_h = max(hint, 200)
            self.control_tabs.setMinimumHeight(0)
            self.control_tabs.setMaximumHeight(max_h)
        else:
            max_h = getattr(self, "manual_tab_height", 130)
            self.control_tabs.setMinimumHeight(0)
            self.control_tabs.setMaximumHeight(max_h)
        if hasattr(self, "plot_panel"):
            self.plot_panel.setVisible(index != 2)

    def _update_live_readouts(self, has_data=True):
        if not hasattr(self, "lbl_live_force"):
            return
        if not has_data:
            self.lbl_live_force.setText("Force: -- -- -- N")
            self.lbl_live_torque.setText("Torque: -- -- -- Nm")
            if hasattr(self, "lbl_rpm_outliers"):
                self.lbl_rpm_outliers.setText(f"RPM outliers: {self._rpm_outlier_count}")
            if hasattr(self, "lbl_current_adc"):
                self.lbl_current_adc.setText("ADC current: -- mA")
            if hasattr(self, "lbl_adc_offset") and not self._adc_offset_samples:
                self.lbl_adc_offset.setText("ADC offset: waiting")
            self._set_metrics_live_label("force", "-- -- --")
            self._set_metrics_live_label("torque", "-- -- --")
            for key in ("pwm", "rpm", "current", "pelec", "pshaft", "eff", "loading", "system", "ct", "cp"):
                self._set_metrics_live_label(key, "--")
            return
        self.lbl_live_force.setText(f"Force: {self._fx:+.2f} {self._fy:+.2f} {self._fz:+.2f} N")
        self.lbl_live_torque.setText(f"Torque: {self._tx:+.3f} {self._ty:+.3f} {self._tz:+.3f} Nm")
        self._set_metrics_live_label("force", f"{self._fx:+.2f} {self._fy:+.2f} {self._fz:+.2f}")
        self._set_metrics_live_label("torque", f"{self._tx:+.3f} {self._ty:+.3f} {self._tz:+.3f}")
        self._set_metrics_live_label("pwm", f"{self.slider.value()}")
        rpm_text = f"{self._latest_rpm}" if self._latest_rpm else "--"
        self._set_metrics_live_label("rpm", rpm_text)
        current_a = (self._current_ma or 0) / 1000.0
        self._set_metrics_live_label("current", f"{current_a:.2f}")

    def _start_sequence(self):
        if self._seq_running:
            return
        steps = self.spin_seq_steps.value()
        if steps <= 0:
            self.lbl_seq_status.setText("Sequence: no steps")
            return
        value = self.spin_seq_start.value()
        delta = self.spin_seq_step.value()
        values = []
        for _ in range(steps):
            values.append(self._clamp_pwm(value))
            value += delta
        hold_ms = max(100, int(self.spin_seq_hold.value() * 1000))
        values = self._build_sequence_values(values)
        pre_hold = int(max(0.0, self.spin_seq_idle_hold.value()) * 1000) if hasattr(self, "spin_seq_idle_hold") else 0
        self._begin_sequence(values, hold_ms, f"Sweep: {len(values)} steps @ {self.spin_seq_hold.value():.1f}s", pre_hold_ms=pre_hold)

    def _clamp_pwm(self, value):
        return max(ESC_MIN, min(ESC_MAX, int(round(value))))

    def _begin_sequence(self, values, hold_ms, status_text, pre_hold_ms=0):
        if not values:
            self.lbl_seq_status.setText("Sequence: no steps")
            return
        if self._seq_running:
            return
        self._seq_status_text = status_text
        self._seq_pre_hold_ms = max(0, pre_hold_ms)
        self._seq_pre_hold_pending = self._seq_pre_hold_ms > 0
        self._seq_values = values
        self._seq_index = 0
        self._seq_hold_ms = max(50, hold_ms)
        self._seq_running = True
        self.btn_seq_run.setEnabled(False)
        self.btn_seq_stop.setEnabled(True)
        self.lbl_seq_status.setText(status_text)
        self._advance_sequence_step()
        self._update_state()

    def _build_sequence_values(self, base_values, loops=None, reverse=None, jitter=None):
        if loops is None:
            loops = max(1, self.spin_seq_loops.value()) if hasattr(self, "spin_seq_loops") else 1
        if reverse is None:
            reverse = self.chk_seq_reverse.isChecked() if hasattr(self, "chk_seq_reverse") else False
        if jitter is None:
            jitter = self.spin_seq_jitter.value() if hasattr(self, "spin_seq_jitter") else 0.0
        seq = []
        for _ in range(loops):
            if jitter > 0:
                jittered = [self._clamp_pwm(v + random.uniform(-jitter, jitter)) for v in base_values]
            else:
                jittered = list(base_values)
            seq.extend(jittered)
            if reverse:
                seq.extend(reversed(jittered))
        return seq

    def _run_selected_preset(self):
        if self._seq_running:
            return
        mode = self.preset_mode_cb.currentText() if hasattr(self, "preset_mode_cb") else "Motor Check"
        peak = self.spin_preset_peak.value() if hasattr(self, "spin_preset_peak") else 200
        hold_s = self.spin_preset_hold.value() if hasattr(self, "spin_preset_hold") else 1.5
        loops = self.spin_preset_loops.value() if hasattr(self, "spin_preset_loops") else 1
        offsets = self._preset_offsets(mode, peak)
        base = self.slider.value()
        values = [self._clamp_pwm(base + off) for off in offsets]
        values = self._build_sequence_values(
            values,
            loops=loops,
            reverse=self.chk_seq_reverse.isChecked() if hasattr(self, "chk_seq_reverse") else False,
            jitter=self.spin_seq_jitter.value() if hasattr(self, "spin_seq_jitter") else 0.0,
        )
        pre_hold = int(max(0.0, self.spin_seq_idle_hold.value()) * 1000) if hasattr(self, "spin_seq_idle_hold") else 0
        self._begin_sequence(values, int(max(0.2, hold_s) * 1000), f"{mode} preset", pre_hold_ms=pre_hold)

    def _preset_offsets(self, mode, peak):
        peak = max(10, peak)
        half = peak // 2
        presets = {
            "Motor Check": [0, peak, 0, -half, 0],
            "Prop Ramp": [0, half, peak, half, 0],
            "Endurance": [0, peak, peak, peak, half, 0],
        }
        return presets.get(mode, presets["Motor Check"])

    def _advance_sequence_step(self):
        if not self._seq_running:
            return
        if getattr(self, "_seq_pre_hold_pending", False):
            self._seq_pre_hold_pending = False
            self.lbl_seq_status.setText(f"{self._seq_status_text} (hold)")
            self.seq_timer.start(self._seq_pre_hold_ms)
            return
        if self._seq_index >= len(self._seq_values):
            self._stop_sequence("complete")
            return
        total = len(self._seq_values)
        pwm = self._seq_values[self._seq_index]
        self.slider.setValue(pwm)
        self.lbl_seq_status.setText(f"Step {self._seq_index + 1}/{total}: PWM {pwm}")
        self._seq_index += 1
        self.seq_timer.start(self._seq_hold_ms)

    def _stop_sequence(self, reason="stopped"):
        if self.seq_timer.isActive():
            self.seq_timer.stop()
        if self._seq_running:
            self._seq_running = False
        self._seq_pre_hold_pending = False
        self.btn_seq_run.setEnabled(True)
        self.btn_seq_stop.setEnabled(False)
        if reason:
            self.lbl_seq_status.setText(f"Sequence {reason}")
        else:
            self.lbl_seq_status.setText("Sequence idle")
        self._update_state()

    def _refresh_ports(self):
        current = self.port_cb.currentText()
        devices = []
        try:
            from serial.tools import list_ports
            devices = sorted(
                [p.device for p in list_ports.comports()],
                key=_serial_port_sort_key,
            )
        except Exception:
            devices = []

        if DEFAULT_PORT and DEFAULT_PORT not in devices:
            devices.insert(0, DEFAULT_PORT)

        self.port_cb.blockSignals(True)
        self.port_cb.clear()
        if devices:
            for dev in devices:
                self.port_cb.addItem(dev)
        else:
            self.port_cb.addItem("No serial ports found")
        self.port_cb.blockSignals(False)

        preferred = current if current in devices else DEFAULT_PORT
        if preferred in devices:
            self.port_cb.setCurrentText(preferred)

        self._update_state()

    def _refresh_ports_if_disconnected(self):
        if not self.worker.connected():
            self._refresh_ports()

    def _update_state(self):
        conn = self.worker.connected()
        safe = self.chk_safe.isChecked()
        has_port = self.port_cb.currentText() != "No serial ports found"
        self.btn_conn.setEnabled(not conn and has_port)
        self.btn_disc.setEnabled(conn)
        self.chk_safe.setEnabled(conn)
        self.btn_arm.setEnabled(conn and safe and not self._armed)
        self.btn_disarm.setEnabled(conn)  # Always allow DISARM when connected
        self.slider.setEnabled(self._armed and not self._seq_running)
        if hasattr(self, "metrics_slider"):
            self.metrics_slider.setEnabled(self._armed and not self._seq_running)
        
        state = getattr(self, "_mcu_state", "")
        state_text = ""
        if not conn:
            state_text = "OFFLINE"
            self.lbl_status.setStyleSheet("font-weight:bold;color:gray")
        elif not safe:
            state_text = "LOCKED"
            self.lbl_status.setStyleSheet("font-weight:bold;color:green")
        elif self._armed:
            if state == "ARMED":
                state_text = "ARMED"
            else:
                state_text = "ARMING..."
            self.lbl_status.setStyleSheet("font-weight:bold;color:red")
        else:
            state_text = "UNLOCKED"
            self.lbl_status.setStyleSheet("font-weight:bold;color:orange")
        if state_text:
            self.lbl_status.setText(f"STATE: {state_text}")
        self._update_console_visibility()

    def _on_pwm_changed(self, value):
        text = f"PWM: {value}"
        if hasattr(self, "lbl_pwm_slider"):
            self.lbl_pwm_slider.setText(text)
        if hasattr(self, "lbl_pwm_status"):
            self.lbl_pwm_status.setText(text)
        if hasattr(self, "lbl_metrics_pwm"):
            self.lbl_metrics_pwm.setText(text)
        if hasattr(self, "slider"):
            self._sync_slider(self.slider, value)
        if hasattr(self, "metrics_slider"):
            self._sync_slider(self.metrics_slider, value)
        if self._armed:
            self._send_pwm(value)

    def _sync_slider(self, slider, value):
        if slider is None:
            return
        if slider.value() == value:
            return
        slider.blockSignals(True)
        slider.setValue(value)
        slider.blockSignals(False)

    def _update_console_visibility(self):
        if not hasattr(self, "console_box"):
            return
        self.console_box.setVisible(True)
        if not self._armed:
            self.log.setVisible(True)
            self.console_safe_msg.setVisible(False)
            return
        state = getattr(self, "_mcu_state", "")
        self.log.setVisible(False)
        self.console_safe_msg.setVisible(True)
        if state == "SAFE":
            if self._safe_lines:
                text = "\n".join(self._safe_lines)
            else:
                text = "SAFE state reported - no telemetry captured."
            self.console_safe_msg.setPlainText(text)
        else:
            self.console_safe_msg.setPlainText("Console paused while ARMED\nDisarm to resume logging.")

    def _do_connect(self):
        port = self.port_cb.currentText()
        if port == "No serial ports found":
            self._refresh_ports()
            return
        if port:
            self.worker.connect(port)

    def _do_disconnect(self):
        if self._armed:
            self.worker.send("DISARM", force_log=True)
        self._armed = False
        if self._seq_running:
            self._stop_sequence("disconnected")
        self.chk_safe.setChecked(False)
        self.worker.disconnect()
        self._reset_buffers()
        self._update_state()

    def _reset_buffers(self):
        self.t0 = None
        self.buf_t.clear()
        self.buf_pwm.clear()
        self.buf_rpm.clear()
        self.buf_fx.clear()
        self.buf_fy.clear()
        self.buf_fz.clear()
        self.buf_tx.clear()
        self.buf_ty.clear()
        self.buf_tz.clear()
        self.buf_current.clear()
        self._avg_buf.clear()
        self._avg_sum = [0.0] * 9
        self._sample_intervals.clear()
        self._last_sample_ts = None
        self._safe_lines.clear()
        self._mcu_state = "INIT"
        self._plot_dirty = True
        self._update_live_readouts(has_data=False)
        self._current_adc = None
        self._current_cc = None
        self._current_adc_ma = None
        self._adc_offset_samples.clear()
        self._adc_zero_raw = ADC_ZERO_RAW
        self._adc_offset_ready = False
        self._rpm_hist.clear()
        self._rpm_outlier_count = 0
        self.metrics_t0 = None
        for buf in self.metrics_buf.values():
            buf.clear()
        self._metrics_dirty = True
        if hasattr(self, "lbl_current"):
            self.lbl_current.setText("Current: -- mA")
        if hasattr(self, "lbl_current_adc"):
            self.lbl_current_adc.setText("ADC current: -- mA")
        if hasattr(self, "lbl_adc_offset"):
            self.lbl_adc_offset.setText("ADC offset: waiting")
        if hasattr(self, "lbl_adc"):
            self.lbl_adc.setText("CC: -- ADC: --")
        if hasattr(self, "lbl_ft_counts"):
            self.lbl_ft_counts.setText("FT cnt: -- / --")
        if hasattr(self, "lbl_rpm_outliers"):
            self.lbl_rpm_outliers.setText("RPM outliers: 0")
        if hasattr(self, "lbl_rate"):
            self.lbl_rate.setText("Rate: -- Hz")
        if hasattr(self, "lbl_avg_current"):
            self.lbl_avg_current.setText("Current: -- A")
        if hasattr(self, "lbl_avg_current_adc"):
            self.lbl_avg_current_adc.setText("ADC: -- A")
        if hasattr(self, "lbl_metrics_pwm"):
            self.lbl_metrics_pwm.setText(f"PWM: {self._idle}")
        if hasattr(self, "metrics_slider"):
            self.metrics_slider.blockSignals(True)
            self.metrics_slider.setValue(self._idle)
            self.metrics_slider.blockSignals(False)
        if hasattr(self, "metric_labels"):
            for lbl in self.metric_labels.values():
                lbl.setText("--")
        if hasattr(self, "metrics_live_labels"):
            for lbl in self.metrics_live_labels.values():
                lbl.setText("--")
        if hasattr(self, "metrics_avg_labels"):
            for lbl in self.metrics_avg_labels.values():
                lbl.setText("--")

    def _on_conn(self, ok, msg):
        self.lbl_conn.setText(msg if msg else "--")
        if ok:
            self._reset_buffers()
            self.log.appendPlainText(f"Connected: {msg}")
        self._update_state()

    def _do_arm(self):
        if self.worker.connected() and self.chk_safe.isChecked():
            self._safe_lines.clear()
            self.worker.send(f"ARM {self._idle}", force_log=True)
            self._armed = True
            self._update_state()

    def _do_disarm(self):
        self.worker.send("DISARM", force_log=True)
        self._armed = False
        if self._seq_running:
            self._stop_sequence("disarmed")
        self.slider.setValue(self._idle)
        if self.chk_safe.isChecked():
            self.chk_safe.blockSignals(True)
            self.chk_safe.setChecked(False)
            self.chk_safe.blockSignals(False)
        self._update_state()

    def _on_safety_toggled(self):
        # If safety turns OFF, immediately send DISARM
        if not self.chk_safe.isChecked():
            self._do_disarm()
        else:
            self._update_state()

    def _send_tick(self):
        self._send_pwm()

    def _send_pwm(self, value=None):
        if value is None:
            value = self.slider.value()
        if self._armed:
            self.worker.send(f"ARM {value}")

    def _do_snap(self):
        n = len(self._avg_buf)
        if n > 0:
            sums = self._avg_sum
            avg_fx = sums[0] / n
            avg_fy = sums[1] / n
            avg_fz = sums[2] / n
            avg_tx = sums[3] / n
            avg_ty = sums[4] / n
            avg_tz = sums[5] / n
            avg_rpm = sums[7] / n
            avg_current = sums[8] / n
            self._snap = (avg_fx, avg_fy, avg_fz, avg_tx, avg_ty, avg_tz, avg_rpm, avg_current)
            self.lbl_snap.setText(
                f"Snap: F {avg_fx:.2f} {avg_fy:.2f} {avg_fz:.2f} | "
                f"T {avg_tx:.3f} {avg_ty:.3f} {avg_tz:.3f} | "
                f"RPM {avg_rpm:.0f} | mA {avg_current:.0f}"
            )
            self.lbl_delta.setText("Δ: F +0.00 +0.00 +0.00 | T +0.000 +0.000 +0.000 | RPM +0 | mA +0")

    def _clear_snap(self):
        self._snap = None
        self.lbl_snap.setText("Snap: F -- | T -- | RPM -- | mA --")
        self.lbl_delta.setText("Δ: F -- | T -- | RPM -- | mA --")

    def _toggle_csv(self):
        if self._csv_file:
            # Stop CSV
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None
            self._csv_columns = []
            self.btn_csv.setText("▶ CSV")
            self.lbl_csv.setText(f"CSV: saved ({self._csv_count})")
            self.lbl_csv.setStyleSheet("font-size:9px;color:#0f0")
        else:
            # Start CSV
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"data_{ts}.csv"
            # CSV_OUT_DIR keeps runs out of whatever folder the GUI was launched
            # from; empty means "here", which is the historical behaviour.
            out_dir = os.environ.get("TVC_CSV_DIR", CSV_OUT_DIR).strip()
            if out_dir:
                os.makedirs(out_dir, exist_ok=True)
                filename = os.path.join(out_dir, filename)
            self._csv_file = open(filename, "w", newline="", encoding="utf-8")
            self._csv_writer = csv.writer(self._csv_file)
            mode = self.csv_mode_cb.currentText() if hasattr(self, "csv_mode_cb") else "Standard"
            self._csv_columns = self._csv_columns_for_mode(mode)
            self._csv_writer.writerow(self._csv_columns)
            self._csv_count = 0
            self.btn_csv.setText("⏹ CSV")
            self.lbl_csv.setText(f"CSV: {filename}")
            self.lbl_csv.setStyleSheet("font-size:9px;color:#f00")

    def _write_csv(self, st):
        if not self._csv_writer or not self._csv_columns:
            return
        current_val = self._current_ma if self._current_ma is not None else st.get("current")
        if current_val is None:
            current_val = st.get("mA", "")
        cc_val = self._current_cc if self._current_cc is not None else st.get("cc", "")
        adc_val = self._current_adc if self._current_adc is not None else st.get("adc", "")
        fc = st.get("fc", "")
        tc = st.get("tc", "")
        data_map = {
            "t_ms": st.get("t", ""),
            "pwm": st.get("pwm", ""),
            "rpm": st.get("rpm", ""),
            "Fx": f"{self._fx:.4f}",
            "Fy": f"{self._fy:.4f}",
            "Fz": f"{self._fz:.4f}",
            "Tx": f"{self._tx:.5f}",
            "Ty": f"{self._ty:.5f}",
            "Tz": f"{self._tz:.5f}",
            "Current_mA": current_val if current_val != "" else "",
            "ADC_Current_mA": f"{self._current_adc_ma:.2f}" if isinstance(self._current_adc_ma, (int, float)) else "",
            "CC_raw": cc_val if cc_val != "" else "",
            "ADC_raw": adc_val if adc_val != "" else "",
            "ForceCount": fc,
            "TorqueCount": tc,
        }
        row = [data_map.get(col, "") for col in self._csv_columns]
        self._csv_writer.writerow(row)
        self._csv_count += 1
        if self._csv_count % 100 == 0:
            self.lbl_csv.setText(f"CSV: {self._csv_count} rows")

    def _csv_columns_for_mode(self, mode):
        base = {
            "Standard": [
                "t_ms",
                "pwm",
                "rpm",
                "Fx",
                "Fy",
                "Fz",
                "Tx",
                "Ty",
                "Tz",
                "Current_mA",
                "ADC_Current_mA",
            ],
            "Forces only": ["t_ms", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz"],
            "Signals only": ["t_ms", "pwm", "rpm", "Current_mA", "ADC_Current_mA", "CC_raw", "ADC_raw", "ForceCount", "TorqueCount"],
        }
        return list(base.get(mode, base["Standard"]))

    def _start_zero(self):
        self._zeroing = True
        self._zero_buf = []
        self._zero_start = time.time()
        self.btn_zero.setEnabled(False)
        self.lbl_zero.setText("Zeroing... 0s")
        self.lbl_zero.setStyleSheet("font-size:9px;color:#fa0")

    def _clear_zero(self):
        self._zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._zeroing = False
        self.btn_zero.setEnabled(True)
        self.lbl_zero.setText("Zero: OFF")
        self.lbl_zero.setStyleSheet("font-size:9px;color:#888")

    def _update_zero(self):
        if not self._zeroing:
            return
        
        elapsed = time.time() - self._zero_start
        self.lbl_zero.setText(f"Zeroing... {elapsed:.1f}s")
        
        # Collect raw values (before zero offset)
        raw_fx = self._fx + self._zero[0]
        raw_fy = self._fy + self._zero[1]
        raw_fz = self._fz + self._zero[2]
        raw_tx = self._tx + self._zero[3]
        raw_ty = self._ty + self._zero[4]
        raw_tz = self._tz + self._zero[5]
        self._zero_buf.append((raw_fx, raw_fy, raw_fz, raw_tx, raw_ty, raw_tz))
        
        if elapsed >= 10.0:
            # Calculate average and set as zero
            n = len(self._zero_buf)
            if n > 0:
                self._zero[0] = sum(s[0] for s in self._zero_buf) / n
                self._zero[1] = sum(s[1] for s in self._zero_buf) / n
                self._zero[2] = sum(s[2] for s in self._zero_buf) / n
                self._zero[3] = sum(s[3] for s in self._zero_buf) / n
                self._zero[4] = sum(s[4] for s in self._zero_buf) / n
                self._zero[5] = sum(s[5] for s in self._zero_buf) / n
            
            self._zeroing = False
            self._zero_buf = []
            self.btn_zero.setEnabled(True)
            self.lbl_zero.setText(f"Zero: ON ({n})")
            self.lbl_zero.setStyleSheet("font-size:9px;color:#0f0")

    def _on_line(self, line):
        capture_for_safe = self._armed or self._mcu_state == "SAFE"
        if capture_for_safe:
            self._safe_lines.append(line)
            if self._mcu_state == "SAFE":
                self._update_console_visibility()
        if not self._armed:
            self.log.appendPlainText(line)

    def _on_tx(self, msg):
        entry = msg if msg.startswith("!!") else f">> {msg}"
        if self._armed or self._mcu_state == "SAFE":
            self._safe_lines.append(entry)
            if self._mcu_state == "SAFE":
                self._update_console_visibility()
        self.log.appendPlainText(entry)

    def _on_stat(self, st):
        # MCU label - show state, pwm, and data indicator
        state = st.get("st", "?")
        self._mcu_state = state
        pwm_val = st.get("pwm")
        if pwm_val is not None and hasattr(self, "lbl_pwm_status"):
            self.lbl_pwm_status.setText(f"PWM: {pwm_val}")
        if hasattr(self, "lbl_mcu"):
            pwm_display = pwm_val if pwm_val is not None else st.get("pwm", "?")
            self.lbl_mcu.setText(f"{state} pwm={pwm_display}")
        if "rpm" in st:
            rpm_filtered = self._filter_rpm(st.get("rpm"))
            if rpm_filtered is not None:
                st["rpm"] = rpm_filtered
                self._latest_rpm = rpm_filtered
                self.lbl_rpm.setText(f"RPM: {self._latest_rpm}")
        current_val = st.get("mA")
        if current_val is None:
            current_val = st.get("current")
        if isinstance(current_val, (int, float)):
            self._current_ma = float(current_val)
            self.lbl_current.setText(f"Current: {self._current_ma:.0f} mA")
        if "cc" in st:
            self._current_cc = st["cc"]
        if "adc" in st:
            try:
                self._current_adc = float(st["adc"])
            except (TypeError, ValueError):
                self._current_adc = None
            if self._current_adc is not None:
                self._update_adc_offset(self._current_adc)
                mapped_ma = self._current_from_adc_ma(self._current_adc)
                if mapped_ma is not None:
                    self._current_adc_ma = mapped_ma
                    if hasattr(self, "lbl_current_adc"):
                        self.lbl_current_adc.setText(f"ADC current: {self._current_adc_ma:.0f} mA")
                else:
                    self._current_adc_ma = None
                    if hasattr(self, "lbl_current_adc"):
                        self.lbl_current_adc.setText("ADC current: -- mA")
        elif hasattr(self, "lbl_current_adc"):
            self._current_adc_ma = None
            self.lbl_current_adc.setText("ADC current: -- mA")
        if "cc" in st or "adc" in st:
            if self._current_cc is not None and self._current_adc is not None:
                self.lbl_adc.setText(
                    f"CC: {self._current_cc} ADC: {self._current_adc:.0f}"
                )
            elif self._current_cc is not None:
                self.lbl_adc.setText(f"CC: {self._current_cc}")
            elif self._current_adc is not None:
                self.lbl_adc.setText(f"ADC: {self._current_adc:.0f}")
        fc = st.get("fc")
        tc = st.get("tc")
        if fc is not None or tc is not None:
            if fc is None:
                fc = "--"
            if tc is None:
                tc = "--"
            self.lbl_ft_counts.setText(f"FT cnt: {fc} / {tc}")
        self._update_sample_rate(time.monotonic())

        # Auto-DISARM when SAFE detected (timeout recovery)
        if state == "SAFE":
            self.worker.send("DISARM", force_log=True)
            self._armed = False
            self.slider.setValue(self._idle)
            if self._seq_running:
                self._stop_sequence("cancelled")
            self._update_state()
        else:
            self._update_state()

        # F/T (mN -> N, mNm -> Nm) with zero offset
        if "Fx" in st:
            self._fx = st["Fx"] * 0.001 - self._zero[0]
        if "Fy" in st:
            self._fy = st["Fy"] * 0.001 - self._zero[1]
        if "Fz" in st:
            self._fz = st["Fz"] * 0.001 - self._zero[2]
        if "Tx" in st:
            self._tx = st["Tx"] * 0.001 - self._zero[3]
        if "Ty" in st:
            self._ty = st["Ty"] * 0.001 - self._zero[4]
        if "Tz" in st:
            self._tz = st["Tz"] * 0.001 - self._zero[5]
        
        self._update_live_readouts(has_data="Fx" in st or "Tx" in st)
        
        # Update zeroing process
        self._update_zero()
        
        # CSV logging
        self._write_csv(st)

        # Update average
        if "t" in st:
            self._update_avg(st)

        # Plot
        self._ingest_plot_sample(st)
        self._update_metrics(st)

    def _update_sample_rate(self, ts):
        if self._last_sample_ts is None:
            self._last_sample_ts = ts
            return
        dt = ts - self._last_sample_ts
        self._last_sample_ts = ts
        if dt <= 0:
            self._sample_intervals.clear()
            self.lbl_rate.setText("Rate: -- Hz")
            return
        self._sample_intervals.append(dt)
        avg_dt = sum(self._sample_intervals) / len(self._sample_intervals)
        rate_hz = 0.0 if avg_dt <= 0 else 1.0 / avg_dt
        self.lbl_rate.setText(f"Rate: {rate_hz:.1f} Hz")

    def _filter_rpm(self, rpm):
        try:
            rpm_val = float(rpm)
        except (TypeError, ValueError):
            return None
        if rpm_val < 0:
            rpm_val = abs(rpm_val)
        if not self._rpm_hist:
            self._rpm_hist.append(rpm_val)
            return rpm_val
        hist = sorted(self._rpm_hist)
        median = hist[len(hist) // 2]
        threshold = max(RPM_OUTLIER_ABS, abs(median) * RPM_OUTLIER_REL)
        if abs(rpm_val - median) > threshold:
            self._rpm_outlier_count += 1
            if hasattr(self, "lbl_rpm_outliers"):
                self.lbl_rpm_outliers.setText(f"RPM outliers: {self._rpm_outlier_count}")
            return median
        self._rpm_hist.append(rpm_val)
        return rpm_val

    def _update_adc_offset(self, adc_raw):
        now = time.monotonic()
        cutoff = now - ADC_OFFSET_WINDOW_S
        while self._adc_offset_samples and self._adc_offset_samples[0][0] < cutoff:
            self._adc_offset_samples.popleft()
        if not self._adc_cal_active:
            if self._armed or self._seq_running:
                if hasattr(self, "lbl_adc_offset"):
                    self.lbl_adc_offset.setText("ADC offset: hold (armed)")
                return
        if not self._adc_cal_active and hasattr(self, "slider") and self.slider.value() > self._idle + 20:
            if hasattr(self, "lbl_adc_offset"):
                self.lbl_adc_offset.setText("ADC offset: hold (throttle)")
            return
        self._adc_offset_samples.append((now, adc_raw))
        sample_count = len(self._adc_offset_samples)
        if sample_count >= ADC_OFFSET_MIN_SAMPLES:
            self._adc_zero_raw = (
                sum(v for _, v in self._adc_offset_samples) / len(self._adc_offset_samples)
            )
            self._adc_offset_ready = True
            if self._adc_cal_active:
                self._adc_cal_active = False
                if hasattr(self, "btn_adc_cal"):
                    self.btn_adc_cal.setEnabled(True)
                    self.btn_adc_cal.setText("Calibrate ADC")
            if hasattr(self, "lbl_adc_offset"):
                self.lbl_adc_offset.setText(f"ADC offset: ready ({self._adc_zero_raw:.1f})")
        else:
            self._adc_offset_ready = False
            if not self._adc_cal_active and hasattr(self, "lbl_adc_offset"):
                self.lbl_adc_offset.setText(
                    f"ADC offset: sampling {sample_count}/{ADC_OFFSET_MIN_SAMPLES}"
                )

    def _start_adc_calibration(self):
        self._adc_offset_samples.clear()
        self._adc_offset_ready = False
        self._adc_cal_active = True
        if hasattr(self, "lbl_adc_offset"):
            self.lbl_adc_offset.setText("ADC offset: calibrating")
        if hasattr(self, "btn_adc_cal"):
            self.btn_adc_cal.setEnabled(False)
            self.btn_adc_cal.setText("Calibrating…")
        self._adc_zero_raw = ADC_ZERO_RAW
        self._adc_offset_samples.clear()

    def _current_from_adc_ma(self, adc_raw):
        if not self._adc_offset_ready:
            return None
        span_raw = ADC_THIRTY_RAW - ADC_ZERO_RAW
        if span_raw <= 0:
            return None
        current_a = (adc_raw - self._adc_zero_raw) * (ADC_FULLSCALE_A / span_raw)
        if current_a < 0:
            current_a = 0.0
        return current_a * 1000.0

    def _update_avg(self, st):
        t_ms = st.get("t")
        if t_ms is None:
            return
        try:
            t_ms = float(t_ms)
        except (TypeError, ValueError):
            return

        if self._avg_buf and not isinstance(self._avg_buf[-1][0], (int, float)):
            self._avg_buf.clear()
            self._avg_sum = [0.0] * 9

        # Handle MCU time reset
        if self._avg_buf and t_ms < self._avg_buf[-1][0] - 1000:
            self._avg_buf.clear()
            self._avg_sum = [0.0] * 9

        pwm_val = st.get("pwm")
        if not isinstance(pwm_val, (int, float)):
            pwm_val = float(self.slider.value())
        rpm_val = st.get("rpm")
        if not isinstance(rpm_val, (int, float)):
            rpm_val = float(self.buf_rpm[-1]) if self.buf_rpm else 0.0
        current_val = self._current_ma if isinstance(self._current_ma, (int, float)) else 0.0
        adc_current_val = (
            float(self._current_adc_ma)
            if isinstance(self._current_adc_ma, (int, float))
            else float("nan")
        )

        sample = (
            t_ms,
            self._fx,
            self._fy,
            self._fz,
            self._tx,
            self._ty,
            self._tz,
            pwm_val,
            rpm_val,
            current_val,
            adc_current_val,
        )
        self._avg_buf.append(sample)
        sums = self._avg_sum
        sums[0] += self._fx
        sums[1] += self._fy
        sums[2] += self._fz
        sums[3] += self._tx
        sums[4] += self._ty
        sums[5] += self._tz
        sums[6] += pwm_val
        sums[7] += rpm_val
        sums[8] += current_val

        # Remove old
        avg_win = self.spin_avg.value()
        cutoff = t_ms - avg_win * 1000
        while self._avg_buf and self._avg_buf[0][0] < cutoff:
            old = self._avg_buf.popleft()
            sums[0] -= old[1]
            sums[1] -= old[2]
            sums[2] -= old[3]
            sums[3] -= old[4]
            sums[4] -= old[5]
            sums[5] -= old[6]
            sums[6] -= old[7]
            sums[7] -= old[8]
            sums[8] -= old[9]

        n = len(self._avg_buf)
        
        if n > 0:
            avg_fx = sums[0] / n
            avg_fy = sums[1] / n
            avg_fz = sums[2] / n
            avg_tx = sums[3] / n
            avg_ty = sums[4] / n
            avg_tz = sums[5] / n
            avg_pwm = sums[6] / n
            avg_rpm = sums[7] / n
            avg_current = sums[8] / n
            adc_vals = [s[10] for s in self._avg_buf if isinstance(s[10], (int, float)) and math.isfinite(s[10])]
            avg_adc_current = (sum(adc_vals) / len(adc_vals)) if adc_vals else None
            if hasattr(self, "lbl_avg_n"):
                self.lbl_avg_n.setText(f"({n} samples @ {avg_pwm:.0f} PWM)")

            self.lbl_avg_f.setText(f"F: {avg_fx:.2f} {avg_fy:.2f} {avg_fz:.2f}")
            self.lbl_avg_t.setText(f"T: {avg_tx:.3f} {avg_ty:.3f} {avg_tz:.3f}")
            self.lbl_avg_rpm.setText(f"RPM: {avg_rpm:.0f}")
            self.lbl_avg_current.setText(f"Current: {(avg_current/1000.0):.2f} A")
            if avg_adc_current is None:
                self.lbl_avg_current_adc.setText("ADC: -- A")
            else:
                self.lbl_avg_current_adc.setText(f"ADC: {(avg_adc_current/1000.0):.2f} A")
            self._set_metrics_avg_label("force", f"{avg_fx:.2f} {avg_fy:.2f} {avg_fz:.2f}")
            self._set_metrics_avg_label("torque", f"{avg_tx:.3f} {avg_ty:.3f} {avg_tz:.3f}")
            self._set_metrics_avg_label("rpm", f"{avg_rpm:.0f}")
            self._set_metrics_avg_label("current", f"{(avg_current/1000.0):.2f}")

            # Update delta if snapshot exists
            if self._snap:
                df = (avg_fx - self._snap[0], avg_fy - self._snap[1], avg_fz - self._snap[2])
                dt = (avg_tx - self._snap[3], avg_ty - self._snap[4], avg_tz - self._snap[5])
                drpm = avg_rpm - self._snap[6]
                dcurrent = avg_current - self._snap[7]
                self.lbl_delta.setText(
                    "Δ: F "
                    f"{df[0]:+.2f} {df[1]:+.2f} {df[2]:+.2f} | "
                    f"T {dt[0]:+.3f} {dt[1]:+.3f} {dt[2]:+.3f} | "
                    f"RPM {drpm:+.0f} | mA {dcurrent:+.0f}"
                )
        else:
            if hasattr(self, "lbl_avg_n"):
                self.lbl_avg_n.setText("(0)")
            self.lbl_avg_f.setText("F: -- -- --")
            self.lbl_avg_t.setText("T: -- -- --")
            self.lbl_avg_rpm.setText("RPM: --")
            self.lbl_avg_current.setText("Current: -- A")
            self.lbl_avg_current_adc.setText("ADC: -- A")
            self._set_metrics_avg_label("rpm", "--")
            self._set_metrics_avg_label("current", "--")
            self._set_metrics_avg_label("force", "-- -- --")
            self._set_metrics_avg_label("torque", "-- -- --")
            self._set_metrics_avg_label("rpm", "--")
            self._set_metrics_avg_label("current", "--")

    def _update_metrics(self, st):
        t_ms = st.get("t")
        rpm = st.get("rpm")
        pwm_val = st.get("pwm")
        if t_ms is None or rpm is None:
            return
        if pwm_val is None:
            pwm_val = self.slider.value()
        try:
            t_ms = float(t_ms)
            rpm = float(rpm)
            pwm_val = float(pwm_val)
        except (TypeError, ValueError):
            return
        current_a = (self._current_ma or 0) / 1000.0
        voltage = self.spin_metrics_voltage.value() if hasattr(self, "spin_metrics_voltage") else DEFAULT_SUPPLY_V
        pelec = voltage * current_a

        torque_axis = self._get_axis_value(self.combo_metrics_torque.currentText(), torque=True) if hasattr(self, "combo_metrics_torque") else self._tz
        omega = 2 * math.pi * rpm / 60.0
        pshaft = abs(torque_axis * omega)

        thrust_axis = self._get_axis_value(self.combo_metrics_force.currentText(), torque=False) if hasattr(self, "combo_metrics_force") else self._fz
        thrust_n = abs(thrust_axis)
        thrust_kgf = thrust_n / 9.80665

        eff = 0.0
        if pelec > 0:
            eff = max(0.0, (pshaft / pelec) * 100.0)
        loading = (thrust_kgf * 1000.0) / pshaft if pshaft > 0 else 0.0  # g per watt
        system_eff = (thrust_kgf * 1000.0) / pelec if pelec > 0 else 0.0  # g per watt

        rho = self.spin_metrics_density.value() if hasattr(self, "spin_metrics_density") else DEFAULT_AIR_DENSITY
        diameter_in = self.spin_metrics_diameter.value() if hasattr(self, "spin_metrics_diameter") else DEFAULT_PROP_DIAMETER_IN
        diameter = diameter_in * INCH_TO_M
        n = rpm / 60.0  # rev per second
        denom_ct = rho * (n ** 2) * (diameter ** 4)
        denom_cp = rho * (n ** 3) * (diameter ** 5)
        ct = thrust_n / denom_ct if denom_ct > 0 else 0.0
        cp = pshaft / denom_cp if denom_cp > 0 else 0.0

        self._set_metric_label("pelec", f"{pelec:8.2f}")
        self._set_metric_label("pshaft", f"{pshaft:8.2f}")
        self._set_metric_label("eff", f"{eff:6.2f}")
        self._set_metric_label("loading", f"{loading:7.2f}")
        self._set_metric_label("system", f"{system_eff:7.2f}")
        self._set_metric_label("ct", f"{ct:7.4f}")
        self._set_metric_label("cp", f"{cp:7.4f}")
        self._set_metrics_live_label("pelec", f"{pelec:.2f}")
        self._set_metrics_live_label("pshaft", f"{pshaft:.2f}")
        self._set_metrics_live_label("eff", f"{eff:.2f}")
        self._set_metrics_live_label("loading", f"{loading:.2f}")
        self._set_metrics_live_label("system", f"{system_eff:.2f}")
        self._set_metrics_live_label("ct", f"{ct:.4f}")
        self._set_metrics_live_label("cp", f"{cp:.4f}")

        if self.metrics_t0 is None:
            self.metrics_t0 = t_ms
        t_s = (t_ms - self.metrics_t0) / 1000.0
        self.metrics_buf["t"].append(t_s)
        self.metrics_buf["pelec"].append(pelec)
        self.metrics_buf["pshaft"].append(pshaft)
        self.metrics_buf["eff"].append(eff)
        self.metrics_buf["loading"].append(loading)
        self.metrics_buf["system_eff"].append(system_eff)
        self.metrics_buf["ct"].append(ct)
        self.metrics_buf["cp"].append(cp)
        self.metrics_buf["pwm"].append(pwm_val)
        self.metrics_buf["rpm"].append(rpm)
        self._metrics_dirty = True
        self._update_metrics_average_labels()

    def _set_metric_label(self, key, text):
        if hasattr(self, "metric_labels") and key in self.metric_labels:
            self.metric_labels[key].setText(text)

    def _get_axis_value(self, axis_name, torque=False):
        if torque:
            mapping = {"Tx": self._tx, "Ty": self._ty, "Tz": self._tz}
        else:
            mapping = {"Fx": self._fx, "Fy": self._fy, "Fz": self._fz}
        return mapping.get(axis_name, list(mapping.values())[-1])

    def _set_metrics_live_label(self, key, text):
        if hasattr(self, "metrics_live_labels"):
            lbl = self.metrics_live_labels.get(key)
            if lbl:
                lbl.setText(text)

    def _set_metrics_avg_label(self, key, text):
        if hasattr(self, "metrics_avg_labels"):
            lbl = self.metrics_avg_labels.get(key)
            if lbl:
                lbl.setText(text)

    def _update_metrics_average_labels(self):
        mapping = {
            "pelec": "pelec",
            "pshaft": "pshaft",
            "eff": "eff",
            "loading": "loading",
            "system": "system_eff",
            "ct": "ct",
            "cp": "cp",
        }
        t_vals = list(self.metrics_buf.get("t", []))
        if not t_vals:
            for label_key in mapping.keys():
                self._set_metrics_avg_label(label_key, "--")
            return
        window = self.spin_metrics_avg.value() if hasattr(self, "spin_metrics_avg") else AVG_WIN
        cutoff = t_vals[-1] - window
        start_idx = 0
        for i, t in enumerate(t_vals):
            if t >= cutoff:
                start_idx = i
                break
        for label_key, buf_key in mapping.items():
            buf = self.metrics_buf.get(buf_key)
            if not buf:
                self._set_metrics_avg_label(label_key, "--")
                continue
            values = list(buf)[start_idx:]
            avg = sum(values) / len(values) if values else 0.0
            if label_key in ("eff", "system"):
                text = f"{avg:.2f}"
            elif label_key in ("ct", "cp"):
                text = f"{avg:.4f}"
            elif label_key == "loading":
                text = f"{avg:.2f}"
            else:
                text = f"{avg:.2f}"
            self._set_metrics_avg_label(label_key, text)

    def _ingest_plot_sample(self, st):
        if "t" not in st:
            return

        t_ms = st["t"]
        try:
            t_ms = float(t_ms)
        except (TypeError, ValueError):
            return
        
        # Handle MCU time reset (time went backward)
        if self.buf_t and t_ms < (self.t0 + self.buf_t[-1] * 1000) - 2000:
            self._reset_buffers()
            # Clear plot curves immediately
            self.curve_pwm.setData([], [])
            self.curve_fx.setData([], [])
            self.curve_fy.setData([], [])
            self.curve_fz.setData([], [])
            self.curve_tx.setData([], [])
            self.curve_ty.setData([], [])
            self.curve_tz.setData([], [])
            self.curve_current.setData([], [])
        
        if self.t0 is None:
            self.t0 = t_ms
        t_s = (t_ms - self.t0) / 1000.0

        self.buf_t.append(t_s)
        self.buf_pwm.append(st.get("pwm", 0))
        self.buf_rpm.append(st.get("rpm", 0))
        self.buf_fx.append(self._fx)
        self.buf_fy.append(self._fy)
        self.buf_fz.append(self._fz)
        self.buf_tx.append(self._tx)
        self.buf_ty.append(self._ty)
        self.buf_tz.append(self._tz)
        self.buf_current.append(self._current_ma)

        # Trim
        while self.buf_t and self.buf_t[-1] - self.buf_t[0] > PLOT_WIN:
            self.buf_t.popleft()
            self.buf_pwm.popleft()
            self.buf_rpm.popleft()
            self.buf_fx.popleft()
            self.buf_fy.popleft()
            self.buf_fz.popleft()
            self.buf_tx.popleft()
            self.buf_ty.popleft()
            self.buf_tz.popleft()
            self.buf_current.popleft()

        if self.buf_t:
            self._plot_dirty = True

    def _render_plots(self):
        if not self._plot_dirty or not self.buf_t:
            return
        self._plot_dirty = False

        t = list(self.buf_t)
        pwm = list(self.buf_pwm)
        rpm = list(self.buf_rpm)
        fx = list(self.buf_fx)
        fy = list(self.buf_fy)
        fz = list(self.buf_fz)
        tx = list(self.buf_tx)
        ty = list(self.buf_ty)
        tz = list(self.buf_tz)
        current = list(self.buf_current)
        xmin, xmax = max(0, t[-1] - PLOT_WIN), t[-1]
        plot_kwargs = {"skipFiniteCheck": True}

        self.curve_pwm.setData(t, pwm, **plot_kwargs)
        self.curve_rpm.setData(t, rpm, **plot_kwargs)
        self.plot_pwm.setXRange(xmin, xmax, padding=0)

        self.curve_fx.setData(t, fx, **plot_kwargs)
        self.curve_fy.setData(t, fy, **plot_kwargs)
        self.curve_fz.setData(t, fz, **plot_kwargs)
        self.plot_f.setXRange(xmin, xmax, padding=0)

        self.curve_tx.setData(t, tx, **plot_kwargs)
        self.curve_ty.setData(t, ty, **plot_kwargs)
        self.curve_tz.setData(t, tz, **plot_kwargs)
        self.plot_t.setXRange(xmin, xmax, padding=0)

        self.curve_current.setData(t, current, **plot_kwargs)
        self.plot_current.setXRange(xmin, xmax, padding=0)
        self._render_metrics_plots()

    def _apply_plot_timer_interval(self):
        hz = max(1, int(round(self.plot_refresh_hz)))
        interval = max(1, int(1000 / hz))
        self._plot_timer.setInterval(interval)

    def _on_plot_rate_changed(self, value):
        self.plot_refresh_hz = max(1, value)
        self._apply_plot_timer_interval()
        if hasattr(self, "spin_plot_hz"):
            self._sync_spinbox(self.spin_plot_hz, self.plot_refresh_hz)
        if hasattr(self, "spin_metrics_plot_hz"):
            self._sync_spinbox(self.spin_metrics_plot_hz, self.plot_refresh_hz)

    def _sync_spinbox(self, spinbox, value):
        if spinbox is None:
            return
        if spinbox.value() == value:
            return
        spinbox.blockSignals(True)
        spinbox.setValue(value)
        spinbox.blockSignals(False)

    def _render_metrics_plots(self):
        if not self._metrics_dirty or not self.metrics_buf["t"]:
            return
        self._metrics_dirty = False
        t = list(self.metrics_buf["t"])
        xmin, xmax = max(0, t[-1] - PLOT_WIN), t[-1]
        plot_kwargs = {"skipFiniteCheck": True}
        def _set_curve(widget, curve, data):
            if widget and widget.isVisible():
                curve.setData(t, data, **plot_kwargs)
        plots_touched = set()
        plot_ranges = {}
        for key, curve in getattr(self, "metric_curves", {}).items():
            plot = self.metric_curve_plots.get(key) if hasattr(self, "metric_curve_plots") else None
            data = list(self.metrics_buf.get(key, []))
            if plot and curve:
                _set_curve(plot, curve, data)
                plots_touched.add(plot)
                if curve.isVisible() and data:
                    lo, hi = min(data), max(data)
                    if plot not in plot_ranges:
                        plot_ranges[plot] = [lo, hi]
                    else:
                        plot_ranges[plot][0] = min(plot_ranges[plot][0], lo)
                        plot_ranges[plot][1] = max(plot_ranges[plot][1], hi)
        for plot in plots_touched:
            plot.setXRange(xmin, xmax, padding=0)
            if plot in plot_ranges:
                lo, hi = plot_ranges[plot]
                if lo == hi:
                    pad = 1.0 if lo == 0 else abs(lo) * 0.05
                    lo -= pad
                    hi += pad
                plot.setYRange(lo, hi, padding=0.1)


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
