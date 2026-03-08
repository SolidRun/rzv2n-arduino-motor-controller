#!/usr/bin/env python3
"""
Mecanum Robot Controller - Serial Test GUI
Direct USB serial interface for controlling and testing the robot.
"""

import re
import serial
import serial.tools.list_ports
import time
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from typing import Optional, Callable


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
        """List all serial ports as (device, description) tuples."""
        return [(p.device, p.description) for p in serial.tools.list_ports.comports()]

    @staticmethod
    def detect_arduino() -> Optional[str]:
        """Auto-detect first Arduino port. Returns device path or None."""
        keywords = ['arduino', 'ch340', 'cp210', 'ftdi', 'usb serial', 'usb-serial']
        vids = {0x2341, 0x1A86, 0x0403, 0x10C4}  # Arduino, CH340, FTDI, CP210x
        for p in serial.tools.list_ports.comports():
            if any(k in p.description.lower() for k in keywords) or p.vid in vids:
                return p.device
        return None

    def connect(self, device: str, baud: int = 115200) -> tuple:
        """Connect to serial port. Returns (success, message)."""
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
        """Disconnect and clean up."""
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
        """Send command string (appends newline). Returns success."""
        if not self.connected or not self.port:
            return False
        try:
            self.port.write(f"{cmd}\n".encode())
            self.port.flush()
            return True
        except Exception:
            return False

    def _read_loop(self):
        """Background reader thread."""
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
        """Handle unexpected disconnect."""
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

# ENC,FL:123,FR:456,RL:789,RR:012
_ENC_RE = re.compile(r'ENC,FL:(-?\d+),FR:(-?\d+),RL:(-?\d+),RR:(-?\d+)')
# ODOM,vx,vy,wz
_ODOM_RE = re.compile(r'ODOM,(-?\d+),(-?\d+),(-?\d+)')
# TEST,FL,pwm:200,FL:123,FR:456,RL:789,RR:012
_TEST_RE = re.compile(r'TEST,\w+,pwm:-?\d+,FL:(-?\d+),FR:(-?\d+),RL:(-?\d+),RR:(-?\d+)')

# Lines that are high-frequency telemetry (filterable)
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

        # VEL mode state
        self._vel_timer_id = None
        self._vel_active = False

        # Main window
        self.root = tk.Tk()
        self.root.title("Mecanum Robot Controller")
        self.root.geometry("900x980")
        self.root.resizable(True, True)
        self.root.minsize(750, 750)

        self._setup_style()
        self._build_ui()
        self._bind_keys()

        # Auto-detect Arduino port on startup
        self._refresh_ports()

    # ----- Style -----

    def _setup_style(self):
        style = ttk.Style()
        for theme in ['clam', 'alt', 'default']:
            if theme in style.theme_names():
                style.theme_use(theme)
                break
        style.configure('Green.TLabel', foreground='#009933', font=('Arial', 10, 'bold'))
        style.configure('Red.TLabel', foreground='#cc0000', font=('Arial', 10, 'bold'))
        style.configure('Gray.TLabel', foreground='#888888', font=('Arial', 10))
        style.configure('Yellow.TLabel', foreground='#cc8800', font=('Arial', 10, 'bold'))
        style.configure('Mono.TLabel', font=('Consolas', 10))
        style.configure('MonoBold.TLabel', font=('Consolas', 10, 'bold'))

    # ----- UI Construction -----

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=6)
        main.pack(fill='both', expand=True)

        self._build_connection(main)
        self._build_controls(main)
        self._build_velocity(main)
        self._build_diagnostics(main)
        self._build_telemetry(main)
        self._build_monitor(main)
        self._build_statusbar(main)

    def _build_connection(self, parent):
        frame = ttk.LabelFrame(parent, text="Connection", padding=8)
        frame.pack(fill='x', pady=(0, 4))

        row = ttk.Frame(frame)
        row.pack(fill='x')

        ttk.Label(row, text="Port:").pack(side='left')
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(row, textvariable=self.port_var, width=25, state='readonly')
        self.port_combo.pack(side='left', padx=(4, 8))

        ttk.Button(row, text="Refresh", command=self._refresh_ports, width=8).pack(side='left', padx=2)

        self.connect_btn = ttk.Button(row, text="Connect", command=self._toggle_connect, width=10)
        self.connect_btn.pack(side='left', padx=(8, 0))

        self.status_label = ttk.Label(row, text="  Disconnected", style='Red.TLabel')
        self.status_label.pack(side='left', padx=(12, 0))

        self.state_label = ttk.Label(row, text="", style='Gray.TLabel')
        self.state_label.pack(side='left', padx=(16, 0))

    def _build_controls(self, parent):
        frame = ttk.LabelFrame(parent, text="Position Control", padding=8)
        frame.pack(fill='x', pady=4)

        # Parameters row
        params = ttk.Frame(frame)
        params.pack(fill='x', pady=(0, 6))

        ttk.Label(params, text="Speed:").pack(side='left')
        self.speed_var = tk.IntVar(value=100)
        ttk.Spinbox(params, from_=20, to=255, textvariable=self.speed_var, width=5).pack(side='left', padx=(2, 12))

        ttk.Label(params, text="Ticks:").pack(side='left')
        self.ticks_var = tk.IntVar(value=1000)
        ttk.Spinbox(params, from_=50, to=50000, increment=100, textvariable=self.ticks_var, width=7).pack(side='left', padx=(2, 12))

        for val in [500, 1000, 2000, 5000]:
            ttk.Button(params, text=str(val), width=5,
                       command=lambda v=val: self.ticks_var.set(v)).pack(side='left', padx=1)

        # Direction grid + rotation + utilities
        bottom = ttk.Frame(frame)
        bottom.pack(fill='x')

        # 3x3 direction grid
        grid = ttk.Frame(bottom)
        grid.pack(side='left')

        self._move_buttons = []
        dirs = [
            ("FL",  "DIAGFL", 0, 0), ("FWD", "FWD",    0, 1), ("FR",  "DIAGFR", 0, 2),
            ("LEFT","LEFT",   1, 0), ("STOP","STOP",    1, 1), ("RIGHT","RIGHT", 1, 2),
            ("BL",  "DIAGBL", 2, 0), ("BWD", "BWD",    2, 1), ("BR",  "DIAGBR", 2, 2),
        ]
        for label, cmd, r, c in dirs:
            if cmd == "STOP":
                btn = ttk.Button(grid, text=label, width=7, command=self._stop)
            else:
                btn = ttk.Button(grid, text=label, width=7, command=lambda d=cmd: self._move(d))
                self._move_buttons.append(btn)
            btn.grid(row=r, column=c, padx=2, pady=2)

        # Rotation + utilities
        side_panel = ttk.Frame(bottom)
        side_panel.pack(side='left', padx=(16, 0))

        rot = ttk.Frame(side_panel)
        rot.pack(pady=(0, 8))
        btn_ccw = ttk.Button(rot, text="CCW", width=8, command=lambda: self._move("TLEFT"))
        btn_ccw.pack(side='left', padx=2)
        self._move_buttons.append(btn_ccw)
        btn_cw = ttk.Button(rot, text="CW", width=8, command=lambda: self._move("TRIGHT"))
        btn_cw.pack(side='left', padx=2)
        self._move_buttons.append(btn_cw)

        util = ttk.Frame(side_panel)
        util.pack()
        ttk.Button(util, text="Read Enc", width=8, command=lambda: self._send("READ")).pack(side='left', padx=2)

    def _build_velocity(self, parent):
        frame = ttk.LabelFrame(parent, text="Velocity Control", padding=8)
        frame.pack(fill='x', pady=4)

        # Top row: buttons
        top = ttk.Frame(frame)
        top.pack(fill='x', pady=(0, 4))

        self.vel_start_btn = ttk.Button(top, text="Start VEL", width=10, command=self._vel_start)
        self.vel_start_btn.pack(side='left', padx=4)
        self.vel_stop_btn = ttk.Button(top, text="Stop", width=8, command=self._vel_stop, state='disabled')
        self.vel_stop_btn.pack(side='left', padx=4)

        self.vel_status_label = ttk.Label(top, text="Inactive", style='Gray.TLabel')
        self.vel_status_label.pack(side='left', padx=(12, 0))

        ttk.Button(top, text="Zero All", width=8, command=self._vel_zero).pack(side='right', padx=4)

        # Sliders
        slider_frame = ttk.Frame(frame)
        slider_frame.pack(fill='x')

        self.vel_vx_var = tk.IntVar(value=0)
        self.vel_vy_var = tk.IntVar(value=0)
        self.vel_wz_var = tk.IntVar(value=0)

        for label, var, desc in [
            ("vx", self.vel_vx_var, "Forward / Back"),
            ("vy", self.vel_vy_var, "Left / Right"),
            ("wz", self.vel_wz_var, "CCW / CW"),
        ]:
            row = ttk.Frame(slider_frame)
            row.pack(fill='x', pady=1)
            ttk.Label(row, text=f"{label}:", width=3).pack(side='left')
            scale = tk.Scale(row, from_=-255, to=255, orient='horizontal',
                             variable=var, length=350, resolution=5, showvalue=True)
            scale.pack(side='left', padx=(4, 8))
            scale.bind('<Double-Button-1>', lambda e, v=var: v.set(0))
            ttk.Label(row, text=desc, style='Gray.TLabel').pack(side='left')

    def _build_diagnostics(self, parent):
        frame = ttk.LabelFrame(parent, text="Diagnostics", padding=8)
        frame.pack(fill='x', pady=4)

        row1 = ttk.Frame(frame)
        row1.pack(fill='x')

        ttk.Label(row1, text="Motor:").pack(side='left')
        self.motor_var = tk.StringVar(value="FL")
        motor_combo = ttk.Combobox(row1, textvariable=self.motor_var, values=["FL", "FR", "RL", "RR"],
                                   width=5, state='readonly')
        motor_combo.pack(side='left', padx=(4, 16))

        ttk.Label(row1, text="PWM:").pack(side='left')
        self.pwm_var = tk.IntVar(value=0)
        self.pwm_scale = tk.Scale(row1, from_=-255, to=255, orient='horizontal',
                                  variable=self.pwm_var, length=250, resolution=5,
                                  showvalue=True)
        self.pwm_scale.pack(side='left', padx=(4, 0))

        row2 = ttk.Frame(frame)
        row2.pack(fill='x', pady=(6, 0))

        ttk.Button(row2, text="Run Motor", width=12, command=self._test_motor).pack(side='left', padx=4)
        ttk.Button(row2, text="Test Encoders", width=12, command=self._test_encoders).pack(side='left', padx=4)
        ttk.Button(row2, text="Calibrate", width=12, command=self._calibrate).pack(side='left', padx=4)
        ttk.Button(row2, text="Stop Test", width=12, command=self._stop).pack(side='left', padx=4)

    def _build_telemetry(self, parent):
        frame = ttk.LabelFrame(parent, text="Telemetry", padding=8)
        frame.pack(fill='x', pady=4)

        # Encoder row
        enc_row = ttk.Frame(frame)
        enc_row.pack(fill='x')

        self._enc_labels = {}
        for name in ['FL', 'FR', 'RL', 'RR']:
            ttk.Label(enc_row, text=f"{name}:", style='Mono.TLabel').pack(side='left')
            lbl = ttk.Label(enc_row, text="---", width=8, style='MonoBold.TLabel', anchor='e')
            lbl.pack(side='left', padx=(0, 12))
            self._enc_labels[name] = lbl

        # Odometry row
        odom_row = ttk.Frame(frame)
        odom_row.pack(fill='x', pady=(4, 0))

        self._odom_labels = {}
        for name, unit in [('vx', 'mm/s'), ('vy', 'mm/s'), ('wz', 'mrad/s')]:
            ttk.Label(odom_row, text=f"{name}:", style='Mono.TLabel').pack(side='left')
            lbl = ttk.Label(odom_row, text="---", width=6, style='MonoBold.TLabel', anchor='e')
            lbl.pack(side='left', padx=(0, 2))
            ttk.Label(odom_row, text=unit, style='Gray.TLabel').pack(side='left', padx=(0, 12))
            self._odom_labels[name] = lbl

    def _build_monitor(self, parent):
        frame = ttk.LabelFrame(parent, text="Serial Monitor", padding=8)
        frame.pack(fill='both', expand=True, pady=4)

        # Console text area (dark theme)
        self.console = scrolledtext.ScrolledText(
            frame, height=15, state='disabled',
            font=('Consolas', 9), wrap='word',
            bg='#1e1e1e', fg='#cccccc',
            insertbackground='white', selectbackground='#264f78')
        self.console.pack(fill='both', expand=True)

        # Text tags for coloring
        self.console.tag_config('tx', foreground='#569cd6')      # Blue - sent
        self.console.tag_config('rx', foreground='#b5cea8')      # Light green - received
        self.console.tag_config('ok', foreground='#4ec9b0', font=('Consolas', 9, 'bold'))
        self.console.tag_config('err', foreground='#f44747', font=('Consolas', 9, 'bold'))
        self.console.tag_config('info', foreground='#808080')    # Gray - system messages
        self.console.tag_config('ts', foreground='#666666')      # Dim - timestamp

        # Input row
        input_row = ttk.Frame(frame)
        input_row.pack(fill='x', pady=(6, 0))

        self.cmd_var = tk.StringVar()
        cmd_entry = ttk.Entry(input_row, textvariable=self.cmd_var, font=('Consolas', 11))
        cmd_entry.pack(side='left', fill='x', expand=True, padx=(0, 6), ipady=4)
        cmd_entry.bind('<Return>', lambda e: self._send_manual())

        ttk.Button(input_row, text="Send", width=8, command=self._send_manual).pack(side='left', padx=2)
        ttk.Button(input_row, text="Clear", width=8, command=self._clear_console).pack(side='left', padx=2)

        self.autoscroll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(input_row, text="Auto", variable=self.autoscroll_var).pack(side='left', padx=(8, 0))

        self.filter_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(input_row, text="Filter", variable=self.filter_var).pack(side='left', padx=(4, 0))

    def _build_statusbar(self, parent):
        bar = ttk.Frame(parent)
        bar.pack(fill='x', pady=(2, 0))
        ttk.Label(bar, text="Keys: W/A/S/D = Move | Q/E = Rotate | Space = Stop | R = Read | 7/9/1/3 = Diag",
                  font=('Arial', 8), foreground='gray').pack()

    # ----- Key Bindings -----

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

    # ----- Connection -----

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

    # ----- Robot State -----

    def _set_state(self, state: str):
        styles = {
            'IDLE': 'Gray.TLabel', 'MOVING': 'Green.TLabel',
            'CALIBRATING': 'Yellow.TLabel',
            'ERROR': 'Red.TLabel', 'BUSY': 'Yellow.TLabel', '': 'Gray.TLabel',
        }
        self.state_label.config(
            text=f"  State: {state}" if state else "",
            style=styles.get(state, 'Gray.TLabel')
        )

    # ----- Serial I/O -----

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
        """Handle received serial data (called from reader thread)."""
        # Schedule GUI updates on main thread
        self.root.after(0, lambda: self._process_rx(data))

    def _process_rx(self, data: str):
        """Process received data: update telemetry, state, and console."""
        # --- Update telemetry dashboard ---
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

        # --- Update robot state ---
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

        # --- Console output (with optional filter) ---
        if is_telemetry and self.filter_var.get():
            return  # Filtered out

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

    # ----- Commands -----

    def _move(self, direction: str):
        if self._vel_active:
            return  # Block position moves while VEL is active
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
            self._vel_stop()  # Already sends STOP
        else:
            self._send("STOP")

    def _test_motor(self):
        motor = self.motor_var.get()
        pwm = self.pwm_var.get()
        self._send(f"TMOTOR,{motor},{pwm}")

    def _test_encoders(self):
        self._send("TENC")

    def _calibrate(self):
        self._send("CALIB")

    # ----- VEL Mode -----

    def _vel_start(self):
        if self._vel_active:
            return
        self._vel_active = True
        self.vel_start_btn.config(state='disabled')
        self.vel_stop_btn.config(state='normal')
        self.vel_status_label.config(text="Streaming at 10Hz", style='Green.TLabel')
        for btn in self._move_buttons:
            btn.config(state='disabled')
        self._vel_send_tick()

    def _vel_stop(self):
        if not self._vel_active:
            return
        self._vel_active = False
        if self._vel_timer_id is not None:
            self.root.after_cancel(self._vel_timer_id)
            self._vel_timer_id = None
        self.vel_start_btn.config(state='normal')
        self.vel_stop_btn.config(state='disabled')
        self.vel_status_label.config(text="Inactive", style='Gray.TLabel')
        for btn in self._move_buttons:
            btn.config(state='normal')
        self._send("STOP")

    def _vel_send_tick(self):
        if not self._vel_active:
            return
        vx = self.vel_vx_var.get()
        vy = self.vel_vy_var.get()
        wz = self.vel_wz_var.get()
        if self.serial.send(f"VEL,{vx},{vy},{wz}"):
            pass  # Don't log every tick - too noisy
        self._vel_timer_id = self.root.after(100, self._vel_send_tick)  # 10Hz

    def _vel_zero(self):
        self.vel_vx_var.set(0)
        self.vel_vy_var.set(0)
        self.vel_wz_var.set(0)

    # ----- Console Logging -----

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

    # ----- Run -----

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.mainloop()

    def _on_close(self):
        self._vel_stop()
        self.serial.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    RobotGUI().run()
