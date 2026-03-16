#!/usr/bin/env python3
"""
Mecanum Robot Controller - Serial Test GUI
Direct USB serial interface for controlling and testing the robot.
"""

import os
import re
import serial
import serial.tools.list_ports
import time
import json
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from typing import Optional, Callable


# =============================================================================
# Color Palette
# =============================================================================

_COLORS = {
    'bg':       '#f5f5f5',
    'card':     '#ffffff',
    'accent':   '#2563eb',
    'accent2':  '#059669',
    'warn':     '#d97706',
    'err':      '#dc2626',
    'text':     '#1e293b',
    'muted':    '#64748b',
    'border':   '#e2e8f0',
    'console_bg': '#1e1e2e',
    'console_fg': '#cdd6f4',
}


# =============================================================================
# Serial Connection
# =============================================================================

class SerialConnection:
    """Thread-safe serial connection to Arduino."""

    def __init__(self):
        self.port: Optional[serial.Serial] = None
        self.connected = False
        self.running = False
        self._thread: Optional[threading.Thread] = None
        self.on_data: Optional[Callable[[str], None]] = None
        self.on_disconnect: Optional[Callable[[str], None]] = None

    @staticmethod
    def list_ports() -> list:
        return [(p.device, p.description) for p in serial.tools.list_ports.comports()]

    @staticmethod
    def detect_arduino() -> Optional[str]:
        keywords = ['arduino', 'ch340', 'cp210', 'ftdi', 'usb serial', 'usb-serial']
        vids = {0x2341, 0x1A86, 0x0403, 0x10C4}
        for p in serial.tools.list_ports.comports():
            if any(k in p.description.lower() for k in keywords) or p.vid in vids:
                return p.device
        return None

    def connect(self, device: str, baud: int = 115200) -> tuple:
        try:
            self.port = serial.Serial(port=device, baudrate=baud, timeout=0.1)
            self.connected = True
            self.running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            return True, f"Connected to {device} @ {baud}"
        except serial.SerialException as e:
            return False, str(e)

    def disconnect(self):
        self.running = False
        self.connected = False
        if self._thread:
            self._thread.join(timeout=1)
            self._thread = None
        if self.port:
            try:
                self.port.close()
            except Exception:
                pass
            self.port = None

    def send(self, cmd: str) -> bool:
        if not self.connected or not self.port:
            return False
        try:
            self.port.write(f"{cmd}\n".encode())
            self.port.flush()
            return True
        except Exception:
            return False

    def _read_loop(self):
        errors = 0
        while self.running:
            try:
                if self.port and self.port.is_open and self.port.in_waiting:
                    line = self.port.readline().decode('utf-8', errors='ignore').strip()
                    if line and self.on_data:
                        self.on_data(line)
                    errors = 0
                else:
                    time.sleep(0.02)
            except serial.SerialException as e:
                self._lost(f"Serial error: {e}")
                break
            except Exception:
                errors += 1
                if errors > 10:
                    self._lost("Too many read errors")
                    break
                time.sleep(0.05)

    def _lost(self, reason: str):
        self.running = False
        self.connected = False
        if self.port:
            try:
                self.port.close()
            except Exception:
                pass
            self.port = None
        if self.on_disconnect:
            self.on_disconnect(reason)


# =============================================================================
# Telemetry line patterns
# =============================================================================

_ENC_RE = re.compile(r'ENC,FL:(-?\d+),FR:(-?\d+),RL:(-?\d+),RR:(-?\d+)')
_ODOM_RE = re.compile(r'ODOM,(-?\d+),(-?\d+),(-?\d+)')
_TEST_RE = re.compile(r'TEST,\w+,pwm:-?\d+,FL:(-?\d+),FR:(-?\d+),RL:(-?\d+),RR:(-?\d+)')
_CALIB_RE = re.compile(r'CALIB,(\w+)')
_TELEMETRY_PREFIXES = ('ENC,', 'ODOM,', 'TEST,', 'Moving:')


# =============================================================================
# GUI Application
# =============================================================================

class RobotGUI:
    """Professional serial GUI for Mecanum robot controller."""

    def __init__(self):
        self.serial = SerialConnection()
        self.serial.on_data = self._on_rx
        self.serial.on_disconnect = self._on_disconnect

        self._vel_timer_id = None
        self._vel_active = False

        self._calib_data = {
            'dz': {}, 'fwd': {}, 'rev': {},
            'fwd_done': False, 'rev_done': False,
            'running': False, 'phase': ''
        }

        self.root = tk.Tk()
        self.root.title("Mecanum Robot Controller")
        self.root.geometry("960x900")
        self.root.resizable(True, True)
        self.root.minsize(800, 700)
        self.root.configure(bg=_COLORS['bg'])

        self._setup_style()
        self._build_ui()
        self._bind_keys()
        self._refresh_ports()

    # ─────────────────────────────────────────────────────────────────────────
    # Style
    # ─────────────────────────────────────────────────────────────────────────

    def _setup_style(self):
        s = ttk.Style()
        for theme in ['clam', 'alt', 'default']:
            if theme in s.theme_names():
                s.theme_use(theme)
                break

        # General
        s.configure('.', font=('Segoe UI', 10))
        s.configure('TFrame', background=_COLORS['bg'])
        s.configure('TLabel', background=_COLORS['bg'], foreground=_COLORS['text'])
        s.configure('TLabelframe', background=_COLORS['bg'])
        s.configure('TLabelframe.Label', background=_COLORS['bg'], foreground=_COLORS['accent'],
                     font=('Segoe UI', 10, 'bold'))
        s.configure('TNotebook', background=_COLORS['bg'])
        s.configure('TNotebook.Tab', font=('Segoe UI', 10, 'bold'), padding=[12, 4])

        # Semantic label styles
        s.configure('Green.TLabel', foreground=_COLORS['accent2'], font=('Segoe UI', 10, 'bold'),
                     background=_COLORS['bg'])
        s.configure('Red.TLabel', foreground=_COLORS['err'], font=('Segoe UI', 10, 'bold'),
                     background=_COLORS['bg'])
        s.configure('Gray.TLabel', foreground=_COLORS['muted'], background=_COLORS['bg'])
        s.configure('Yellow.TLabel', foreground=_COLORS['warn'], font=('Segoe UI', 10, 'bold'),
                     background=_COLORS['bg'])
        s.configure('Mono.TLabel', font=('Consolas', 10), background=_COLORS['bg'])
        s.configure('MonoBold.TLabel', font=('Consolas', 10, 'bold'), background=_COLORS['bg'])
        s.configure('Header.TLabel', font=('Segoe UI', 11, 'bold'), foreground=_COLORS['text'],
                     background=_COLORS['bg'])

        # Card-like frames
        s.configure('Card.TFrame', background=_COLORS['card'], relief='solid', borderwidth=1)

        # Table heading
        s.configure('Treeview', font=('Consolas', 10), rowheight=24)
        s.configure('Treeview.Heading', font=('Segoe UI', 9, 'bold'))

    # ─────────────────────────────────────────────────────────────────────────
    # UI Construction
    # ─────────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=8)
        main.pack(fill='both', expand=True)

        self._build_connection(main)

        # Always-visible telemetry bar
        self._build_telemetry(main)

        # Notebook — each section is its own tab
        self.notebook = ttk.Notebook(main)
        self.notebook.pack(fill='both', expand=True, pady=(4, 0))

        tab_pos = ttk.Frame(self.notebook)
        self.notebook.add(tab_pos, text="  Position  ")
        self._build_position_control(tab_pos)

        tab_vel = ttk.Frame(self.notebook)
        self.notebook.add(tab_vel, text="  Velocity  ")
        self._build_velocity_control(tab_vel)

        tab_test = ttk.Frame(self.notebook)
        self.notebook.add(tab_test, text="  Motor Test  ")
        self._build_diagnostics(tab_test)

        tab_calib = ttk.Frame(self.notebook)
        self.notebook.add(tab_calib, text="  Smart Calibration  ")
        self._build_tab_calibration(tab_calib)

        # Monitor always visible at bottom
        self._build_monitor(main)

    # --- Connection bar ---

    def _build_connection(self, parent):
        frame = ttk.Frame(parent)
        frame.pack(fill='x', pady=(0, 2))

        inner = ttk.Frame(frame)
        inner.pack(fill='x')

        ttk.Label(inner, text="Port:").pack(side='left')
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(inner, textvariable=self.port_var, width=22, state='readonly')
        self.port_combo.pack(side='left', padx=(4, 6))

        ttk.Button(inner, text="Refresh", command=self._refresh_ports, width=7).pack(side='left', padx=2)

        self.connect_btn = ttk.Button(inner, text="Connect", command=self._toggle_connect, width=10)
        self.connect_btn.pack(side='left', padx=(6, 0))

        self.status_label = ttk.Label(inner, text="  Disconnected", style='Red.TLabel')
        self.status_label.pack(side='left', padx=(10, 0))

        self.state_label = ttk.Label(inner, text="", style='Gray.TLabel')
        self.state_label.pack(side='left', padx=(12, 0))

    def _build_position_control(self, parent):
        frame = ttk.LabelFrame(parent, text="Position Control", padding=10)
        frame.pack(fill='x', pady=(4, 2), padx=2)

        # Parameters — labeled number entries
        params = ttk.Frame(frame)
        params.pack(fill='x', pady=(0, 8))

        ttk.Label(params, text="Speed (20-255):", style='Gray.TLabel').grid(row=0, column=0, sticky='w')
        self.speed_var = tk.IntVar(value=100)
        ttk.Entry(params, textvariable=self.speed_var, width=6, font=('Consolas', 11),
                  justify='center').grid(row=0, column=1, padx=(4, 20))

        ttk.Label(params, text="Ticks:", style='Gray.TLabel').grid(row=0, column=2, sticky='w')
        self.ticks_var = tk.IntVar(value=1000)
        ttk.Entry(params, textvariable=self.ticks_var, width=8, font=('Consolas', 11),
                  justify='center').grid(row=0, column=3, padx=(4, 20))

        ttk.Label(params, text="Presets:", style='Gray.TLabel').grid(row=0, column=4, sticky='w', padx=(0, 4))
        for i, val in enumerate([500, 1000, 2000, 5000]):
            ttk.Button(params, text=str(val), width=5,
                       command=lambda v=val: self.ticks_var.set(v)).grid(row=0, column=5+i, padx=1)

        # Direction grid + rotation
        bottom = ttk.Frame(frame)
        bottom.pack(fill='x')

        grid = ttk.Frame(bottom)
        grid.pack(side='left')

        self._move_buttons = []
        dirs = [
            ("FL", "DIAGFL", 0, 0), ("FWD", "FWD", 0, 1), ("FR", "DIAGFR", 0, 2),
            ("LEFT", "LEFT", 1, 0), ("STOP", "STOP", 1, 1), ("RIGHT", "RIGHT", 1, 2),
            ("BL", "DIAGBL", 2, 0), ("BWD", "BWD", 2, 1), ("BR", "DIAGBR", 2, 2),
        ]
        for label, cmd, r, c in dirs:
            if cmd == "STOP":
                btn = ttk.Button(grid, text=label, width=7, command=self._stop)
            else:
                btn = ttk.Button(grid, text=label, width=7, command=lambda d=cmd: self._move(d))
                self._move_buttons.append(btn)
            btn.grid(row=r, column=c, padx=2, pady=2)

        side_panel = ttk.Frame(bottom)
        side_panel.pack(side='left', padx=(20, 0))

        rot = ttk.Frame(side_panel)
        rot.pack(pady=(0, 8))
        btn_ccw = ttk.Button(rot, text="CCW", width=8, command=lambda: self._move("TLEFT"))
        btn_ccw.pack(side='left', padx=2)
        self._move_buttons.append(btn_ccw)
        btn_cw = ttk.Button(rot, text="CW", width=8, command=lambda: self._move("TRIGHT"))
        btn_cw.pack(side='left', padx=2)
        self._move_buttons.append(btn_cw)

        ttk.Button(side_panel, text="Read Encoders", width=14,
                   command=lambda: self._send("READ")).pack(anchor='w')

    def _build_velocity_control(self, parent):
        frame = ttk.LabelFrame(parent, text="Velocity Control (VEL)", padding=10)
        frame.pack(fill='x', pady=2, padx=2)

        # Value entries
        vals = ttk.Frame(frame)
        vals.pack(fill='x', pady=(0, 8))

        self.vel_vx_var = tk.IntVar(value=0)
        self.vel_vy_var = tk.IntVar(value=0)
        self.vel_wz_var = tk.IntVar(value=0)

        for i, (name, var, desc) in enumerate([
            ("vx", self.vel_vx_var, "Forward/Back"),
            ("vy", self.vel_vy_var, "Left/Right"),
            ("wz", self.vel_wz_var, "CCW/CW"),
        ]):
            col = i * 4
            ttk.Label(vals, text=f"{name}:", style='MonoBold.TLabel').grid(row=0, column=col, sticky='e')
            ttk.Entry(vals, textvariable=var, width=6, font=('Consolas', 11),
                      justify='center').grid(row=0, column=col+1, padx=(4, 2))
            ttk.Label(vals, text=f"(-255..255)", style='Gray.TLabel',
                      font=('Segoe UI', 8)).grid(row=1, column=col+1, sticky='n')
            ttk.Label(vals, text=desc, style='Gray.TLabel').grid(row=0, column=col+2, padx=(2, 20))

        # Controls
        ctrls = ttk.Frame(frame)
        ctrls.pack(fill='x')

        self.vel_start_btn = ttk.Button(ctrls, text="Start", width=8, command=self._vel_start)
        self.vel_start_btn.grid(row=0, column=0, padx=(0, 4))

        self.vel_stop_btn = ttk.Button(ctrls, text="Stop", width=8, command=self._vel_stop)
        self.vel_stop_btn.grid(row=0, column=1, padx=(0, 4))

        ttk.Button(ctrls, text="Zero All", width=8, command=self._vel_zero).grid(row=0, column=2, padx=(0, 12))

        self.vel_status_label = ttk.Label(ctrls, text="Stopped", style='Gray.TLabel')
        self.vel_status_label.grid(row=0, column=3)

    def _build_diagnostics(self, parent):
        frame = ttk.LabelFrame(parent, text="Motor Test", padding=10)
        frame.pack(fill='x', pady=2, padx=2)

        # Motor test controls
        controls = ttk.Frame(frame)
        controls.pack(fill='x', pady=(0, 8))

        ttk.Label(controls, text="Motor:").grid(row=0, column=0, sticky='w')
        self.motor_var = tk.StringVar(value="FL")
        ttk.Combobox(controls, textvariable=self.motor_var, values=["FL", "FR", "RL", "RR"],
                     width=5, state='readonly').grid(row=0, column=1, padx=(4, 20))

        ttk.Label(controls, text="PWM (-255..255):").grid(row=0, column=2, sticky='w')
        self.pwm_var = tk.IntVar(value=0)
        ttk.Entry(controls, textvariable=self.pwm_var, width=6, font=('Consolas', 11),
                  justify='center').grid(row=0, column=3, padx=(4, 20))

        ttk.Button(controls, text="Run Motor", width=12,
                   command=self._test_motor).grid(row=0, column=4, padx=(0, 4))
        ttk.Button(controls, text="Stop", width=8,
                   command=self._stop).grid(row=0, column=5, padx=(0, 4))

        # Other diagnostics
        btns = ttk.Frame(frame)
        btns.pack(fill='x')

        ttk.Button(btns, text="Test Encoders", width=14,
                   command=self._test_encoders).grid(row=0, column=0, padx=(0, 4))
        ttk.Button(btns, text="Quick Calibrate", width=14,
                   command=self._calibrate).grid(row=0, column=1, padx=(0, 4))
        ttk.Button(btns, text="Smart Calibrate", width=14,
                   command=lambda: self.notebook.select(3)).grid(row=0, column=2, padx=(0, 4))

    def _build_telemetry(self, parent):
        frame = ttk.LabelFrame(parent, text="Live Telemetry", padding=6)
        frame.pack(fill='x', pady=(4, 0))

        row = ttk.Frame(frame)
        row.pack(fill='x')

        # Encoders
        self._enc_labels = {}
        ttk.Label(row, text="ENC", style='Gray.TLabel', font=('Segoe UI', 8, 'bold')).grid(row=0, column=0, padx=(0, 6))
        for i, name in enumerate(['FL', 'FR', 'RL', 'RR']):
            col = 1 + i * 2
            ttk.Label(row, text=f"{name}:", style='Mono.TLabel').grid(row=0, column=col, sticky='e')
            lbl = ttk.Label(row, text="---", width=8, style='MonoBold.TLabel', anchor='e')
            lbl.grid(row=0, column=col+1, padx=(1, 8))
            self._enc_labels[name] = lbl

        # Separator
        sep = ttk.Separator(row, orient='vertical')
        sep.grid(row=0, column=9, sticky='ns', padx=8)

        # Odometry
        self._odom_labels = {}
        ttk.Label(row, text="ODOM", style='Gray.TLabel', font=('Segoe UI', 8, 'bold')).grid(row=0, column=10, padx=(0, 6))
        for i, (name, unit) in enumerate([('vx', 'mm/s'), ('vy', 'mm/s'), ('wz', 'mrad/s')]):
            col = 11 + i * 3
            ttk.Label(row, text=f"{name}:", style='Mono.TLabel').grid(row=0, column=col, sticky='e')
            lbl = ttk.Label(row, text="---", width=6, style='MonoBold.TLabel', anchor='e')
            lbl.grid(row=0, column=col+1, padx=(1, 1))
            ttk.Label(row, text=unit, style='Gray.TLabel', font=('Segoe UI', 7)).grid(row=0, column=col+2, padx=(0, 6))
            self._odom_labels[name] = lbl

    # --- Tab 2: Smart Calibration ---

    def _build_tab_calibration(self, parent):
        canvas = tk.Canvas(parent, bg=_COLORS['bg'], highlightthickness=0)
        scrollbar = ttk.Scrollbar(parent, orient='vertical', command=canvas.yview)
        content = ttk.Frame(canvas)

        content.bind('<Configure>', lambda e: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.create_window((0, 0), window=content, anchor='nw')
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')

        self._build_calib_motor(content)
        self._build_calib_measurement(content)
        self._build_calib_export(content)

    def _build_calib_motor(self, parent):
        """Motor calibration: dead-zone + fwd/rev speed."""
        frame = ttk.LabelFrame(parent, text="Motor Calibration (Dead-Zone + Forward + Reverse)", padding=10)
        frame.pack(fill='x', pady=(4, 2), padx=2)

        # Controls row
        top = ttk.Frame(frame)
        top.pack(fill='x', pady=(0, 8))

        self._calib_start_btn = ttk.Button(top, text="Run Full Calibration", width=20, command=self._calib_run)
        self._calib_start_btn.pack(side='left', padx=(0, 6))

        self._calib_stop_btn = ttk.Button(top, text="Abort", width=8, command=self._stop, state='disabled')
        self._calib_stop_btn.pack(side='left', padx=(0, 10))

        self._calib_status_var = tk.StringVar(value="Ready")
        ttk.Label(top, textvariable=self._calib_status_var, style='Gray.TLabel').pack(side='left')

        # Results table using grid
        table = ttk.Frame(frame)
        table.pack(fill='x')

        headers = ["", "Dead-Zone", "Forward Max", "Reverse Max", "Fwd/Rev Diff"]
        for c, h in enumerate(headers):
            anchor = 'w' if c == 0 else 'center'
            lbl = ttk.Label(table, text=h, font=('Segoe UI', 9, 'bold'), anchor=anchor)
            lbl.grid(row=0, column=c, padx=(0, 4), pady=(0, 4), sticky='ew')

        table.columnconfigure(0, weight=0, minsize=40)
        for c in range(1, 5):
            table.columnconfigure(c, weight=1, minsize=120)

        self._calib_labels = {}
        motors = ['FL', 'RL', 'RR', 'FR']
        for r, m in enumerate(motors, start=1):
            # Motor name with colored background
            name_lbl = ttk.Label(table, text=f"  {m}  ", style='MonoBold.TLabel')
            name_lbl.grid(row=r, column=0, padx=(0, 4), pady=2, sticky='w')

            self._calib_labels[m] = {}
            for c, key in enumerate(['dz', 'fwd', 'rev', 'asym'], start=1):
                lbl = ttk.Label(table, text="---", style='Mono.TLabel', width=14, anchor='center')
                lbl.grid(row=r, column=c, padx=(0, 4), pady=2, sticky='ew')
                self._calib_labels[m][key] = lbl

        # Separator + legend
        ttk.Separator(frame, orient='horizontal').pack(fill='x', pady=(8, 4))

        legend = ttk.Frame(frame)
        legend.pack(fill='x')

        legends = [
            ("Dead-Zone:", "Min PWM to start moving (0-255)"),
            ("Forward/Reverse Max:", "Max encoder ticks per 20ms at full PWM"),
            ("Fwd/Rev Diff:", "Speed difference between directions (lower = better)"),
        ]
        for i, (title, desc) in enumerate(legends):
            ttk.Label(legend, text=title, font=('Segoe UI', 8, 'bold')).grid(row=i, column=0, sticky='w', padx=(0, 6))
            ttk.Label(legend, text=desc, style='Gray.TLabel', font=('Segoe UI', 8)).grid(row=i, column=1, sticky='w')

    def _build_calib_measurement(self, parent):
        """Ticks-per-mm measurement section."""
        frame = ttk.LabelFrame(parent, text="Ticks-per-mm Measurement", padding=10)
        frame.pack(fill='x', pady=2, padx=2)

        ttk.Label(frame, text=(
            "Drive the robot, measure real distance with a ruler, and record the result. "
            "Tests if ticks-to-distance is consistent across speeds and directions."
        ), style='Gray.TLabel', wraplength=800).pack(anchor='w', pady=(0, 8))

        # Drive controls — two rows
        row1 = ttk.Frame(frame)
        row1.pack(fill='x')

        ttk.Label(row1, text="Direction:").grid(row=0, column=0, sticky='w')
        self._meas_dir_var = tk.StringVar(value="FWD")
        ttk.Combobox(row1, textvariable=self._meas_dir_var,
                     values=["FWD", "BWD", "LEFT", "RIGHT"], width=7,
                     state='readonly').grid(row=0, column=1, padx=(4, 12))

        ttk.Label(row1, text="Speed:").grid(row=0, column=2, sticky='w')
        self._meas_speed_var = tk.IntVar(value=100)
        ttk.Spinbox(row1, from_=40, to=200, textvariable=self._meas_speed_var,
                    width=5).grid(row=0, column=3, padx=(4, 12))

        ttk.Label(row1, text="Ticks:").grid(row=0, column=4, sticky='w')
        self._meas_ticks_var = tk.IntVar(value=5000)
        ttk.Spinbox(row1, from_=500, to=50000, increment=500,
                    textvariable=self._meas_ticks_var, width=7).grid(row=0, column=5, padx=(4, 12))

        ttk.Button(row1, text="Drive", width=8,
                   command=self._meas_drive).grid(row=0, column=6, padx=(4, 0))

        # Input row
        row2 = ttk.Frame(frame)
        row2.pack(fill='x', pady=(8, 0))

        ttk.Label(row2, text="Measured distance (mm):").grid(row=0, column=0, sticky='w')
        self._meas_dist_var = tk.DoubleVar(value=0.0)
        ttk.Entry(row2, textvariable=self._meas_dist_var, width=10,
                  font=('Consolas', 11)).grid(row=0, column=1, padx=(4, 8))

        ttk.Button(row2, text="Add Result", width=10,
                   command=self._meas_add).grid(row=0, column=2, padx=(0, 4))
        ttk.Button(row2, text="Clear All", width=8,
                   command=self._meas_clear).grid(row=0, column=3)

        # Preset buttons
        preset_frame = ttk.LabelFrame(frame, text="Quick Tests", padding=6)
        preset_frame.pack(fill='x', pady=(10, 0))

        presets = [
            ("FWD slow", "FWD", 60, 3000), ("FWD med", "FWD", 100, 5000),
            ("FWD fast", "FWD", 180, 5000), ("BWD med", "BWD", 100, 5000),
            ("LEFT med", "LEFT", 100, 5000), ("RIGHT med", "RIGHT", 100, 5000),
        ]
        for i, (label, d, spd, tck) in enumerate(presets):
            ttk.Button(preset_frame, text=label, width=10,
                       command=lambda d=d, s=spd, t=tck: self._meas_preset(d, s, t)
                       ).grid(row=0, column=i, padx=2, pady=2)
        for i in range(len(presets)):
            preset_frame.columnconfigure(i, weight=1)

        # Results table
        table_frame = ttk.Frame(frame)
        table_frame.pack(fill='x', pady=(10, 0))

        cols = ('dir', 'speed', 'ticks', 'measured_mm', 'ticks_per_mm', 'vs_theory')
        self._meas_tree = ttk.Treeview(table_frame, columns=cols, show='headings', height=5)
        for c, h, w in [
            ('dir', 'Direction', 80), ('speed', 'Speed', 60), ('ticks', 'Ticks', 80),
            ('measured_mm', 'Distance (mm)', 110), ('ticks_per_mm', 'Ticks/mm', 90),
            ('vs_theory', 'vs Theory', 90)
        ]:
            self._meas_tree.heading(c, text=h)
            self._meas_tree.column(c, width=w, anchor='center', stretch=True)
        self._meas_tree.pack(side='left', fill='x', expand=True)

        sb = ttk.Scrollbar(table_frame, orient='vertical', command=self._meas_tree.yview)
        sb.pack(side='right', fill='y')
        self._meas_tree.configure(yscrollcommand=sb.set)

        # Summary
        summary_frame = ttk.Frame(frame)
        summary_frame.pack(fill='x', pady=(8, 0))

        self._meas_summary_var = tk.StringVar(value="No measurements yet. Drive, measure, and add results.")
        ttk.Label(summary_frame, textvariable=self._meas_summary_var,
                  style='Mono.TLabel', justify='left').pack(anchor='w')

        # Internal state
        self._meas_last_dir = "FWD"
        self._meas_last_speed = 100
        self._meas_last_ticks = 5000
        self._meas_results = []

    def _build_calib_export(self, parent):
        """Export / update config section."""
        frame = ttk.LabelFrame(parent, text="Export & Update", padding=10)
        frame.pack(fill='x', pady=2, padx=2)

        row = ttk.Frame(frame)
        row.pack(fill='x')

        ttk.Button(row, text="Update config.h", width=14,
                   command=self._update_config).pack(side='left', padx=(0, 6))
        ttk.Button(row, text="Export JSON", width=12,
                   command=self._export_json).pack(side='left', padx=(0, 10))

        self._config_status_var = tk.StringVar(value="Run calibration first, then export or update config.")
        ttk.Label(row, textvariable=self._config_status_var, style='Gray.TLabel').pack(side='left')

    # --- Serial Monitor ---

    def _build_monitor(self, parent):
        frame = ttk.LabelFrame(parent, text="Serial Monitor", padding=6)
        frame.pack(fill='both', expand=True, pady=(6, 0))

        self.console = scrolledtext.ScrolledText(
            frame, height=10, state='disabled',
            font=('Consolas', 9), wrap='word',
            bg=_COLORS['console_bg'], fg=_COLORS['console_fg'],
            insertbackground='white', selectbackground='#264f78',
            relief='flat', borderwidth=0)
        self.console.pack(fill='both', expand=True)

        self.console.tag_config('tx', foreground='#89b4fa')
        self.console.tag_config('rx', foreground='#a6e3a1')
        self.console.tag_config('ok', foreground='#94e2d5', font=('Consolas', 9, 'bold'))
        self.console.tag_config('err', foreground='#f38ba8', font=('Consolas', 9, 'bold'))
        self.console.tag_config('info', foreground='#6c7086')
        self.console.tag_config('ts', foreground='#45475a')

        input_row = ttk.Frame(frame)
        input_row.pack(fill='x', pady=(4, 0))

        self.cmd_var = tk.StringVar()
        cmd_entry = ttk.Entry(input_row, textvariable=self.cmd_var, font=('Consolas', 10))
        cmd_entry.pack(side='left', fill='x', expand=True, padx=(0, 4), ipady=3)
        cmd_entry.bind('<Return>', lambda e: self._send_manual())

        ttk.Button(input_row, text="Send", width=6, command=self._send_manual).pack(side='left', padx=1)
        ttk.Button(input_row, text="Clear", width=6, command=self._clear_console).pack(side='left', padx=1)

        self.autoscroll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(input_row, text="Auto", variable=self.autoscroll_var).pack(side='left', padx=(6, 0))

        self.filter_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(input_row, text="Filter", variable=self.filter_var).pack(side='left', padx=(2, 0))

    # ─────────────────────────────────────────────────────────────────────────
    # Key Bindings
    # ─────────────────────────────────────────────────────────────────────────

    def _bind_keys(self):
        binds = {
            'w': "FWD", 'W': "FWD", 's': "BWD", 'S': "BWD",
            'a': "LEFT", 'A': "LEFT", 'd': "RIGHT", 'D': "RIGHT",
            'q': "TLEFT", 'Q': "TLEFT", 'e': "TRIGHT", 'E': "TRIGHT",
        }
        for key, cmd in binds.items():
            self.root.bind(f'<{key}>', lambda e, c=cmd: self._move(c))

        diag_binds = {'7': "DIAGFL", '9': "DIAGFR", '1': "DIAGBL", '3': "DIAGBR"}
        for key, cmd in diag_binds.items():
            self.root.bind(f'<Key-{key}>', lambda e, c=cmd: self._move(c))
            self.root.bind(f'<KP_{key}>', lambda e, c=cmd: self._move(c))

        self.root.bind('<space>', lambda e: self._stop())
        self.root.bind('<Escape>', lambda e: self._stop())
        self.root.bind('<r>', lambda e: self._send("READ"))
        self.root.bind('<R>', lambda e: self._send("READ"))

    # ─────────────────────────────────────────────────────────────────────────
    # Connection
    # ─────────────────────────────────────────────────────────────────────────

    def _refresh_ports(self):
        ports = self.serial.list_ports()
        devices = [p[0] for p in ports]
        self.port_combo['values'] = devices
        auto = self.serial.detect_arduino()
        if auto:
            self.port_var.set(auto)
            self._log_info(f"Auto-detected: {auto}")
        elif devices:
            self.port_var.set(devices[0])

    def _toggle_connect(self):
        if self.serial.connected:
            self._vel_stop()
            self.serial.disconnect()
            self.connect_btn.config(text="Connect")
            self.status_label.config(text="  Disconnected", style='Red.TLabel')
            self._set_state("")
            self._log_info("Disconnected")
        else:
            port = self.port_var.get()
            if not port:
                messagebox.showerror("Error", "Select a serial port")
                return
            ok, msg = self.serial.connect(port)
            if ok:
                self.connect_btn.config(text="Disconnect")
                self.status_label.config(text=f"  {port} (115200)", style='Green.TLabel')
                self._log_info(msg)
            else:
                messagebox.showerror("Connection Error", msg)
                self._log_err(msg)

    def _on_disconnect(self, reason: str):
        def update():
            self._vel_stop()
            self.connect_btn.config(text="Connect")
            self.status_label.config(text="  Connection Lost", style='Red.TLabel')
            self._set_state("")
            self._log_err(f"DISCONNECTED: {reason}")
        self.root.after(0, update)

    # ─────────────────────────────────────────────────────────────────────────
    # Robot State
    # ─────────────────────────────────────────────────────────────────────────

    def _set_state(self, state: str):
        styles = {
            'IDLE': 'Gray.TLabel', 'MOVING': 'Green.TLabel',
            'CALIBRATING': 'Yellow.TLabel',
            'ERROR': 'Red.TLabel', 'BUSY': 'Yellow.TLabel', '': 'Gray.TLabel',
        }
        self.state_label.config(
            text=f"  State: {state}" if state else "",
            style=styles.get(state, 'Gray.TLabel'))

    # ─────────────────────────────────────────────────────────────────────────
    # Serial I/O
    # ─────────────────────────────────────────────────────────────────────────

    def _send(self, cmd: str):
        if self.serial.send(cmd):
            self._log_tx(cmd)
        else:
            self._log_err("Not connected")

    def _send_manual(self):
        cmd = self.cmd_var.get().strip().upper()
        if cmd:
            self._send(cmd)
            self.cmd_var.set("")

    def _on_rx(self, data: str):
        self.root.after(0, lambda: self._process_rx(data))

    def _process_rx(self, data: str):
        # Smart calibration parsing
        if self._calib_process(data):
            self._log_rx(data, 'ok')
            return

        is_telemetry = data.startswith(_TELEMETRY_PREFIXES)

        m = _ENC_RE.match(data)
        if m:
            self._enc_labels['FL'].config(text=m.group(1))
            self._enc_labels['FR'].config(text=m.group(2))
            self._enc_labels['RL'].config(text=m.group(3))
            self._enc_labels['RR'].config(text=m.group(4))

        m = _ODOM_RE.match(data)
        if m:
            self._odom_labels['vx'].config(text=m.group(1))
            self._odom_labels['vy'].config(text=m.group(2))
            self._odom_labels['wz'].config(text=m.group(3))

        m = _TEST_RE.match(data)
        if m:
            self._enc_labels['FL'].config(text=m.group(1))
            self._enc_labels['FR'].config(text=m.group(2))
            self._enc_labels['RL'].config(text=m.group(3))
            self._enc_labels['RR'].config(text=m.group(4))

        if data.startswith(('READY', 'DONE')):
            self._set_state('IDLE')
        elif data == 'OK':
            self._set_state('MOVING')
        elif data.startswith('BUSY'):
            self._set_state('BUSY')
        elif data.startswith(('ERROR', 'STALL')):
            self._set_state('ERROR')

        m = _CALIB_RE.match(data)
        if m:
            phase = m.group(1)
            if phase == 'start':
                self._set_state('CALIBRATING')
            elif phase == 'aborted':
                self._set_state('IDLE')

        if is_telemetry and self.filter_var.get():
            return

        tag = 'rx'
        if data.startswith(('DONE', 'READY')):
            tag = 'ok'
        elif data.startswith(('ERROR', 'BUSY', 'STALL')):
            tag = 'err'
        elif data.startswith('CALIB,'):
            tag = 'ok'
        elif data.startswith('Moving:'):
            tag = 'info'

        self._log_rx(data, tag)

    # ─────────────────────────────────────────────────────────────────────────
    # Commands
    # ─────────────────────────────────────────────────────────────────────────

    def _move(self, direction: str):
        if self._vel_active:
            return
        speed = self.speed_var.get()
        ticks = self.ticks_var.get()
        if speed < 20 or speed > 255:
            self._log_err(f"Speed out of range: {speed}")
            return
        cmd_map = {
            "FWD": f"FWD,{speed},{ticks}", "BWD": f"BWD,{speed},{ticks}",
            "LEFT": f"LEFT,{speed},{ticks}", "RIGHT": f"RIGHT,{speed},{ticks}",
            "TLEFT": f"TURN,{speed},{ticks}", "TRIGHT": f"TURN,{speed},{-ticks}",
            "DIAGFL": f"DIAGFL,{speed},{ticks}", "DIAGFR": f"DIAGFR,{speed},{ticks}",
            "DIAGBL": f"DIAGBL,{speed},{ticks}", "DIAGBR": f"DIAGBR,{speed},{ticks}",
        }
        if direction in cmd_map:
            self._send(cmd_map[direction])

    def _stop(self):
        if self._vel_active:
            self._vel_stop()
        else:
            self._send("STOP")

    def _test_motor(self):
        self._send(f"TMOTOR,{self.motor_var.get()},{self.pwm_var.get()}")

    def _test_encoders(self):
        self._send("TENC")

    def _calibrate(self):
        self._send("CALIB")

    # ─────────────────────────────────────────────────────────────────────────
    # VEL Mode
    # ─────────────────────────────────────────────────────────────────────────

    def _vel_start(self):
        if self._vel_active:
            return
        self._vel_active = True
        self.vel_start_btn.config(state='disabled')
        self.vel_status_label.config(text="Running at 10Hz", style='Green.TLabel')
        for btn in self._move_buttons:
            btn.config(state='disabled')
        self._vel_send_tick()

    def _vel_stop(self):
        self._vel_active = False
        if self._vel_timer_id is not None:
            self.root.after_cancel(self._vel_timer_id)
            self._vel_timer_id = None
        self.vel_start_btn.config(state='normal')
        self.vel_status_label.config(text="Stopped", style='Gray.TLabel')
        for btn in self._move_buttons:
            btn.config(state='normal')
        self._send("STOP")

    def _vel_send_tick(self):
        if not self._vel_active:
            return
        try:
            vx = self.vel_vx_var.get()
            vy = self.vel_vy_var.get()
            wz = self.vel_wz_var.get()
        except tk.TclError:
            vx = vy = wz = 0
        self.serial.send(f"VEL,{vx},{vy},{wz}")
        self._vel_timer_id = self.root.after(100, self._vel_send_tick)

    def _vel_zero(self):
        self.vel_vx_var.set(0)
        self.vel_vy_var.set(0)
        self.vel_wz_var.set(0)

    # ─────────────────────────────────────────────────────────────────────────
    # Smart Calibration
    # ─────────────────────────────────────────────────────────────────────────

    def _calib_run(self):
        if not self.serial.connected:
            self._log_err("Not connected")
            return
        self._calib_data = {
            'dz': {}, 'fwd': {}, 'rev': {},
            'fwd_done': False, 'rev_done': False,
            'running': True, 'phase': 'starting'
        }
        for m in ['FL', 'RL', 'RR', 'FR']:
            for key in ['dz', 'fwd', 'rev', 'asym']:
                self._calib_labels[m][key].config(text="...", foreground='')
        self._calib_start_btn.config(state='disabled')
        self._calib_stop_btn.config(state='normal')
        self._calib_status_var.set("Running calibration...")
        self._send("CALIB")

    def _calib_process(self, data: str):
        if not self._calib_data.get('running'):
            return False

        if data.startswith('CALIB,start'):
            self._calib_data['phase'] = 'deadzone'
            self._calib_status_var.set("Phase 1/3: Detecting dead-zones...")
            return True

        m = re.match(r'CALIB,dz,(\w+):(\d+|MAX!)', data)
        if m:
            motor, val = m.group(1), m.group(2)
            is_max = val == 'MAX!'
            val_int = 120 if is_max else int(val)
            self._calib_data['dz'][motor] = val_int
            lbl = self._calib_labels.get(motor, {}).get('dz')
            if lbl:
                txt = "FAIL" if is_max else f"PWM {val_int}"
                lbl.config(text=txt, foreground=_COLORS['err'] if is_max else '')
            return True

        if data.startswith('CALIB,phase:'):
            phase = data.split(':')[1]
            self._calib_data['phase'] = phase
            n = '2/3' if phase == 'forward' else '3/3'
            self._calib_status_var.set(f"Phase {n}: Measuring {phase} speed...")
            return True

        m = re.match(r'CALIB,(fwd|rev),(\d+)/(\d+)', data)
        if m:
            direction, session, total = m.group(1), m.group(2), m.group(3)
            name = 'Forward' if direction == 'fwd' else 'Reverse'
            self._calib_status_var.set(f"{name} session {session}/{total}...")
            return True

        m = re.match(r'CALIB,(fwd|rev),done,(.+)', data)
        if m:
            direction = m.group(1)
            pairs = re.findall(r'(\w+):(\d+)', m.group(2))
            for motor, val in pairs:
                val_int = int(val)
                self._calib_data[direction][motor] = val_int
                lbl = self._calib_labels.get(motor, {}).get(direction)
                if lbl:
                    lbl.config(text=f"{val_int} ticks")
            if direction == 'fwd':
                self._calib_data['fwd_done'] = True
            else:
                self._calib_data['rev_done'] = True
            if self._calib_data['fwd_done'] and self._calib_data['rev_done']:
                self._update_asymmetry()
            return True

        if data == 'CALIB,saved':
            self._calib_status_var.set("Calibration saved to EEPROM!")
            return True

        if data == 'CALIB,aborted':
            self._calib_finish("Calibration aborted")
            return True

        if data == 'DONE' and self._calib_data.get('running'):
            self._calib_finish("Calibration complete!")
            return True

        return False

    def _update_asymmetry(self):
        for motor in ['FL', 'RL', 'RR', 'FR']:
            fwd = self._calib_data['fwd'].get(motor)
            rev = self._calib_data['rev'].get(motor)
            if fwd and rev:
                avg = (fwd + rev) / 2
                diff_pct = abs(fwd - rev) / avg * 100 if avg > 0 else 0
                diff_abs = abs(fwd - rev)
                lbl = self._calib_labels[motor]['asym']
                txt = f"{diff_abs} ({diff_pct:.1f}%)"
                color = _COLORS['err'] if diff_pct > 10 else (_COLORS['warn'] if diff_pct > 5 else '')
                lbl.config(text=txt, foreground=color)

    def _calib_finish(self, msg: str):
        self._calib_data['running'] = False
        self._calib_start_btn.config(state='normal')
        self._calib_stop_btn.config(state='disabled')
        self._calib_status_var.set(msg)

    # ─────────────────────────────────────────────────────────────────────────
    # Ticks-per-mm Measurement
    # ─────────────────────────────────────────────────────────────────────────

    def _meas_drive(self):
        direction = self._meas_dir_var.get()
        speed = self._meas_speed_var.get()
        ticks = self._meas_ticks_var.get()
        self._meas_last_dir = direction
        self._meas_last_speed = speed
        self._meas_last_ticks = ticks
        self._meas_summary_var.set(f"Driving {direction} at speed {speed} for {ticks} ticks... measure when done.")
        self._send(f"{direction},{speed},{ticks}")

    def _meas_preset(self, direction: str, speed: int, ticks: int):
        self._meas_dir_var.set(direction)
        self._meas_speed_var.set(speed)
        self._meas_ticks_var.set(ticks)
        self._meas_drive()

    def _meas_add(self):
        dist = self._meas_dist_var.get()
        if dist <= 0:
            self._meas_summary_var.set("Enter a positive measured distance first!")
            return

        ticks = self._meas_last_ticks
        tpm = ticks / dist
        theoretical = 4320 / (3.14159 * 80)
        error = (tpm - theoretical) / theoretical * 100

        result = {
            'dir': self._meas_last_dir, 'speed': self._meas_last_speed,
            'ticks': ticks, 'measured_mm': dist,
            'ticks_per_mm': tpm, 'error_pct': error,
        }
        self._meas_results.append(result)

        self._meas_tree.insert('', 'end', values=(
            result['dir'], result['speed'], result['ticks'],
            f"{dist:.1f}", f"{tpm:.2f}", f"{error:+.1f}%"))

        self._meas_update_summary()
        self._meas_dist_var.set(0.0)

    def _meas_clear(self):
        self._meas_results.clear()
        for item in self._meas_tree.get_children():
            self._meas_tree.delete(item)
        self._meas_summary_var.set("Cleared. Drive, measure, and add results.")

    def _meas_update_summary(self):
        if not self._meas_results:
            self._meas_summary_var.set("No measurements yet.")
            return

        all_tpm = [r['ticks_per_mm'] for r in self._meas_results]
        avg = sum(all_tpm) / len(all_tpm)
        spread = (max(all_tpm) - min(all_tpm)) / avg * 100 if avg else 0
        theory = 4320 / (3.14159 * 80)

        lines = [f"Avg: {avg:.2f} ticks/mm   Theory: {theory:.2f}   Spread: {spread:.1f}%"]

        by_dir = {}
        for r in self._meas_results:
            by_dir.setdefault(r['dir'], []).append(r['ticks_per_mm'])
        dir_avgs = {d: sum(v)/len(v) for d, v in by_dir.items()}

        if len(dir_avgs) > 1:
            lines.append("Per-direction:  " + "   ".join(f"{d}: {v:.2f}" for d, v in sorted(dir_avgs.items())))
            fwd = dir_avgs.get('FWD') or dir_avgs.get('BWD')
            strafe = dir_avgs.get('LEFT') or dir_avgs.get('RIGHT')
            if fwd and strafe:
                slip = (1 - strafe / fwd) * 100
                if abs(slip) > 5:
                    lines.append(f"Strafe roller slip: {slip:.0f}% — use per-direction ticks/mm!")

        if spread < 5:
            lines.append("GOOD: <5% spread — single ticks/mm value is fine.")
        elif spread < 15:
            lines.append("OK: 5-15% spread — consider per-direction ticks/mm.")
        else:
            lines.append("HIGH: >15% spread — check wheels/surface/weight.")

        self._meas_summary_var.set("\n".join(lines))

    # ─────────────────────────────────────────────────────────────────────────
    # Export / Update Config
    # ─────────────────────────────────────────────────────────────────────────

    def _update_config(self):
        if not self._calib_data.get('fwd_done'):
            self._config_status_var.set("Run calibration first!")
            return

        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.normpath(os.path.join(script_dir, '..', 'src', 'core', 'config.h'))

        if not os.path.exists(config_path):
            self._config_status_var.set(f"Not found: {config_path}")
            return

        fwd_vals = [self._calib_data['fwd'].get(m, 150) for m in ['FL', 'RL', 'RR', 'FR']]
        new_default = round(sum(fwd_vals) / len(fwd_vals))

        try:
            with open(config_path, 'r') as f:
                content = f.read()
            new_content = re.sub(
                r'#define\s+MAX_MOTOR_TICKRATE\s+\d+',
                f'#define MAX_MOTOR_TICKRATE      {new_default}',
                content)
            if new_content == content:
                self._config_status_var.set("MAX_MOTOR_TICKRATE not found in config.h")
                return
            with open(config_path, 'w') as f:
                f.write(new_content)
            self._config_status_var.set(f"Updated MAX_MOTOR_TICKRATE to {new_default}")
            self._log_info(f"config.h updated: MAX_MOTOR_TICKRATE = {new_default}")
        except Exception as e:
            self._config_status_var.set(f"Error: {e}")

    def _export_json(self):
        if not self._calib_data.get('fwd_done'):
            self._config_status_var.set("Run calibration first!")
            return

        script_dir = os.path.dirname(os.path.abspath(__file__))
        json_path = os.path.join(script_dir, 'calibration.json')

        export = {
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'dead_zone': self._calib_data.get('dz', {}),
            'forward_tickrate': self._calib_data.get('fwd', {}),
            'reverse_tickrate': self._calib_data.get('rev', {}),
        }

        asym = {}
        for m in ['FL', 'RL', 'RR', 'FR']:
            fwd = self._calib_data['fwd'].get(m)
            rev = self._calib_data['rev'].get(m)
            if fwd and rev:
                avg = (fwd + rev) / 2
                asym[m] = round(abs(fwd - rev) / avg * 100, 1) if avg > 0 else 0.0
        export['asymmetry_pct'] = asym

        if self._meas_results:
            export['ticks_per_mm_measurements'] = self._meas_results
            all_tpm = [r['ticks_per_mm'] for r in self._meas_results]
            export['ticks_per_mm_avg'] = round(sum(all_tpm) / len(all_tpm), 2)
            by_dir = {}
            for r in self._meas_results:
                by_dir.setdefault(r['dir'], []).append(r['ticks_per_mm'])
            export['ticks_per_mm_by_direction'] = {
                d: round(sum(v)/len(v), 2) for d, v in by_dir.items()
            }

        try:
            with open(json_path, 'w') as f:
                json.dump(export, f, indent=2)
            self._config_status_var.set(f"Exported to {json_path}")
            self._log_info(f"Calibration exported to {json_path}")
        except Exception as e:
            self._config_status_var.set(f"Error: {e}")

    # ─────────────────────────────────────────────────────────────────────────
    # Console Logging
    # ─────────────────────────────────────────────────────────────────────────

    def _log(self, text: str, tag: str):
        self.console.config(state='normal')
        ts = time.strftime("%H:%M:%S")
        self.console.insert('end', f"{ts} ", 'ts')
        self.console.insert('end', f"{text}\n", tag)
        if self.autoscroll_var.get():
            self.console.see('end')
        self.console.config(state='disabled')

    def _log_tx(self, cmd: str):
        self._log(f"-> {cmd}", 'tx')

    def _log_rx(self, data: str, tag: str = 'rx'):
        self._log(f"<- {data}", tag)

    def _log_info(self, msg: str):
        self._log(f"-- {msg}", 'info')

    def _log_err(self, msg: str):
        self._log(f"!! {msg}", 'err')

    def _clear_console(self):
        self.console.config(state='normal')
        self.console.delete('1.0', 'end')
        self.console.config(state='disabled')

    # ─────────────────────────────────────────────────────────────────────────
    # Run
    # ─────────────────────────────────────────────────────────────────────────

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.mainloop()

    def _on_close(self):
        self._vel_stop()
        self.serial.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    RobotGUI().run()
