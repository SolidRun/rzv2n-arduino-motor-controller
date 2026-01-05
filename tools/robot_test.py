#!/usr/bin/env python3
"""
Robot Test Utility for Mecanum 4WD Controller
GUI version - Works on Linux and Windows with auto port detection and Bluetooth support
"""

import serial
import serial.tools.list_ports
import subprocess
import time
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from typing import Optional, Callable
from queue import Queue

# Bluetooth imports - pybluez for Linux/Windows
try:
    import bluetooth
    BLUETOOTH_AVAILABLE = True
except ImportError:
    BLUETOOTH_AVAILABLE = False
    print("Warning: Bluetooth library not installed. Bluetooth features disabled.")
    print("Run setup.py to install dependencies, or install manually:")
    print("  pip install pybluez-updated  # For Linux with Python 3.12+")


class RobotController:
    """Handles communication with the robot via Serial or Bluetooth"""

    def __init__(self):
        self.serial: Optional[serial.Serial] = None
        self.bt_socket = None
        self.connected = False
        self.connection_type = None  # 'serial' or 'bluetooth'
        self.connection_info = None  # Store port/address for reconnect
        self.read_thread: Optional[threading.Thread] = None
        self.running = False
        self.on_data_callback: Optional[Callable[[str], None]] = None
        self.on_disconnect_callback: Optional[Callable[[str], None]] = None
        self.last_error = None

        # Response tracking
        self.last_send_time = 0
        self.last_rx_time = time.time()

    @staticmethod
    def list_ports() -> list:
        """List all available serial ports"""
        ports = []
        for port in serial.tools.list_ports.comports():
            ports.append({
                'device': port.device,
                'description': port.description,
                'vid': port.vid,
                'pid': port.pid
            })
        return ports

    @staticmethod
    def detect_arduino() -> list:
        """Auto-detect Arduino ports"""
        arduino_ports = []
        for port in serial.tools.list_ports.comports():
            desc_lower = port.description.lower()
            is_arduino = False

            # Check description
            if any(k in desc_lower for k in ['arduino', 'ch340', 'cp210', 'ftdi', 'usb serial', 'usb-serial']):
                is_arduino = True
            # Check VID
            elif port.vid in [0x2341, 0x1A86, 0x0403, 0x10C4]:  # Arduino, CH340, FTDI, CP210x
                is_arduino = True

            if is_arduino:
                arduino_ports.append(port.device)

        return arduino_ports

    @staticmethod
    def scan_bluetooth_devices() -> list:
        """Scan for nearby Bluetooth devices"""
        if not BLUETOOTH_AVAILABLE:
            return []

        try:
            devices = bluetooth.discover_devices(duration=8, lookup_names=True, flush_cache=True, lookup_class=False)
            return [{'address': addr, 'name': name} for addr, name in devices]
        except Exception as e:
            print(f"Bluetooth scan error: {e}")
            return []

    def connect(self, port: str, baudrate: int = 9600) -> tuple:
        """Connect to Arduino via Serial. Returns (success, message)"""
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.1
            )
            self.connected = True
            self.connection_type = 'serial'
            self.connection_info = {'port': port, 'baudrate': baudrate}
            self.running = True
            self.last_rx_time = time.time()

            # Start reader thread
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()

            return True, f"Connected to {port}"
        except serial.SerialException as e:
            return False, str(e)

    @staticmethod
    def check_bt_status(address: str) -> dict:
        """Check Bluetooth device status using bluetoothctl. Returns dict with paired/connected/trusted."""
        info = {'paired': False, 'connected': False, 'trusted': False}
        try:
            result = subprocess.run(
                ['bluetoothctl', 'info', address],
                capture_output=True, text=True, timeout=5
            )
            output = result.stdout
            info['paired'] = 'Paired: yes' in output
            info['connected'] = 'Connected: yes' in output
            info['trusted'] = 'Trusted: yes' in output
        except Exception:
            pass
        return info

    def connect_bluetooth(self, bt_address: str) -> tuple:
        """Connect to Arduino via Bluetooth. Returns (success, message)"""
        if not BLUETOOTH_AVAILABLE:
            return False, "Bluetooth library not available"

        try:
            # Check if device is already connected elsewhere
            bt_info = self.check_bt_status(bt_address)
            if bt_info['connected']:
                print(f"[BT_CONNECT] Device already connected, will try to connect anyway...")

            print(f"[BT_CONNECT] Attempting to connect to {bt_address} on channel 1...")
            print(f"[BT_CONNECT] Device status - Paired: {bt_info['paired']}, Connected: {bt_info['connected']}")

            # Create Bluetooth socket
            self.bt_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            print(f"[BT_CONNECT] Socket created")

            # Set socket timeout for connect
            self.bt_socket.settimeout(10.0)

            # Connect to device on channel 1 (standard SPP channel)
            print(f"[BT_CONNECT] Connecting to ({bt_address}, 1)...")
            self.bt_socket.connect((bt_address, 1))
            print(f"[BT_CONNECT] Connected successfully!")

            # Set shorter timeout for reads
            self.bt_socket.settimeout(0.1)

            self.connected = True
            self.connection_type = 'bluetooth'
            self.connection_info = {'address': bt_address}
            self.running = True
            self.last_rx_time = time.time()

            # Start reader thread
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            print(f"[BT_CONNECT] Reader thread started")

            return True, f"Connected to Bluetooth device {bt_address}"
        except bluetooth.BluetoothError as e:
            print(f"[BT_CONNECT] Bluetooth error: {e}")
            self._cleanup_bt_socket()
            return False, f"Bluetooth connection error: {str(e)}"
        except Exception as e:
            print(f"[BT_CONNECT] Unexpected error: {e}")
            self._cleanup_bt_socket()
            return False, str(e)

    def _cleanup_bt_socket(self):
        """Clean up Bluetooth socket on error"""
        if self.bt_socket:
            try:
                self.bt_socket.close()
            except Exception:
                pass
            self.bt_socket = None

    def disconnect(self):
        """Disconnect from Arduino (Serial or Bluetooth)"""
        self.running = False
        self.connected = False

        if self.read_thread:
            self.read_thread.join(timeout=1)

        # Close serial connection
        if self.serial:
            try:
                if self.serial.is_open:
                    self.serial.close()
            except Exception:
                pass
            self.serial = None

        # Close Bluetooth connection
        self._cleanup_bt_socket()

        self.connection_type = None

    def _read_loop(self):
        """Background thread to read data from Serial or Bluetooth"""
        buffer = ""
        consecutive_errors = 0

        while self.running:
            try:
                if self.connection_type == 'serial' and self.serial:
                    if not self.serial.is_open:
                        self.last_error = "Serial port closed"
                        self._handle_connection_lost()
                        break
                    if self.serial.in_waiting:
                        line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            self.last_rx_time = time.time()
                            if self.on_data_callback:
                                self.on_data_callback(line)
                        consecutive_errors = 0

                elif self.connection_type == 'bluetooth' and self.bt_socket:
                    try:
                        data = self.bt_socket.recv(256).decode('utf-8', errors='ignore')
                        if data:
                            buffer += data
                            consecutive_errors = 0
                            self.last_rx_time = time.time()
                            # Process complete lines
                            while '\n' in buffer:
                                line, buffer = buffer.split('\n', 1)
                                line = line.strip()
                                if line and self.on_data_callback:
                                    self.on_data_callback(line)
                    except bluetooth.BluetoothError as e:
                        err_str = str(e).lower()
                        # Check for real connection errors (not just timeout)
                        if any(x in err_str for x in ['connection reset', 'broken pipe', 'not connected', 'transport endpoint']):
                            self.last_error = f"Bluetooth disconnected: {e}"
                            self._handle_connection_lost()
                            break
                        # Timeout is normal, ignore
                    except OSError as e:
                        # Socket errors indicate lost connection
                        if e.errno in [104, 107, 32]:  # Connection reset, not connected, broken pipe
                            self.last_error = f"Connection lost: {e}"
                            self._handle_connection_lost()
                            break

            except serial.SerialException as e:
                self.last_error = f"Serial error: {e}"
                self._handle_connection_lost()
                break
            except Exception as e:
                consecutive_errors += 1
                if consecutive_errors > 10:
                    self.last_error = f"Too many errors: {e}"
                    self._handle_connection_lost()
                    break

            time.sleep(0.02)  # 50Hz polling

    def _handle_connection_lost(self):
        """Handle unexpected connection loss"""
        self.running = False
        self.connected = False

        # Close sockets/ports
        if self.serial:
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None

        self._cleanup_bt_socket()
        self.connection_type = None

        # Notify GUI
        if self.on_disconnect_callback:
            self.on_disconnect_callback(self.last_error)

    def send(self, cmd: str) -> bool:
        """Send command to Arduino (Serial or Bluetooth)"""
        if not self.connected:
            print(f"[SEND] Not connected, cannot send: {cmd}")
            return False

        try:
            message = f"{cmd}\n".encode()

            if self.connection_type == 'serial' and self.serial:
                print(f"[SERIAL_TX] Sending: {cmd}")
                self.serial.write(message)
                self.serial.flush()  # Ensure data is sent
                self.last_send_time = time.time()
                return True
            elif self.connection_type == 'bluetooth' and self.bt_socket:
                print(f"[BT_TX] Sending via Bluetooth: {cmd} (bytes: {len(message)})")
                self.bt_socket.send(message)
                print(f"[BT_TX] Send successful")
                self.last_send_time = time.time()
                return True
            else:
                print(f"[SEND] No active connection (type={self.connection_type})")
                return False
        except Exception as e:
            print(f"[SEND] Error sending command: {e}")
            self.last_error = str(e)
            return False


class RobotGUI:
    """Main GUI application for controlling the robot"""

    # Default Bluetooth address for quick connection
    DEFAULT_BT_ADDRESS = "EC:5C:84:11:EC:97"
    DEFAULT_BT_NAME = "Arduino_BT_Bridge"

    def __init__(self):
        self.robot = RobotController()
        self.robot.on_data_callback = self.on_serial_data
        self.robot.on_disconnect_callback = self.on_connection_lost
        self.bt_devices = []

        # Command state tracking
        self.waiting_for_response = False
        self.last_command = None
        self.command_start_time = 0

        # Create main window
        self.root = tk.Tk()
        self.root.title("Mecanum Robot Controller")
        self.root.geometry("800x700")
        self.root.resizable(True, True)

        # Configure style
        self._configure_style()
        self._create_widgets()
        self._bind_keys()

        # Start with Bluetooth mode if available, otherwise Serial
        if BLUETOOTH_AVAILABLE:
            self.conn_type_var.set("bluetooth")
            self.on_connection_type_changed()
            # Pre-populate with default Bluetooth address
            self.bt_devices = [{'address': self.DEFAULT_BT_ADDRESS, 'name': self.DEFAULT_BT_NAME}]
            self.bt_combo['values'] = [f"{self.DEFAULT_BT_NAME} ({self.DEFAULT_BT_ADDRESS})"]
            self.bt_var.set(f"{self.DEFAULT_BT_NAME} ({self.DEFAULT_BT_ADDRESS})")
        else:
            self.conn_type_var.set("serial")
            self.on_connection_type_changed()
            self.refresh_ports()

        # Start periodic UI update
        self._update_status()

    def _configure_style(self):
        """Configure ttk styles for better appearance"""
        style = ttk.Style()

        # Try to use a modern theme
        available_themes = style.theme_names()
        for theme in ['clam', 'alt', 'default']:
            if theme in available_themes:
                style.theme_use(theme)
                break

        # Configure custom styles
        style.configure('Connected.TLabel', foreground='green', font=('Arial', 10, 'bold'))
        style.configure('Disconnected.TLabel', foreground='red', font=('Arial', 10, 'bold'))
        style.configure('Warning.TLabel', foreground='orange', font=('Arial', 10, 'bold'))
        style.configure('BTStatus.TLabel', foreground='#0066cc', font=('Arial', 9))
        style.configure('Message.TLabel', font=('Arial', 11, 'bold'))

    def _create_widgets(self):
        """Create all GUI widgets"""
        # Main container with padding
        main_frame = ttk.Frame(self.root, padding=5)
        main_frame.pack(fill='both', expand=True)

        # === Connection Frame ===
        conn_frame = ttk.LabelFrame(main_frame, text="Connection", padding=10)
        conn_frame.pack(fill='x', pady=(0, 5))

        # Connection type selection
        self.conn_type_var = tk.StringVar(value="bluetooth" if BLUETOOTH_AVAILABLE else "serial")

        type_frame = ttk.Frame(conn_frame)
        type_frame.pack(fill='x', pady=(0, 5))

        ttk.Radiobutton(type_frame, text="Bluetooth", variable=self.conn_type_var,
                        value="bluetooth", command=self.on_connection_type_changed,
                        state='normal' if BLUETOOTH_AVAILABLE else 'disabled').pack(side='left', padx=5)
        ttk.Radiobutton(type_frame, text="Serial/USB", variable=self.conn_type_var,
                        value="serial", command=self.on_connection_type_changed).pack(side='left', padx=5)

        # Bluetooth connection widgets
        self.bt_frame = ttk.Frame(conn_frame)

        ttk.Label(self.bt_frame, text="Device:").pack(side='left', padx=(0, 5))

        self.bt_var = tk.StringVar()
        self.bt_combo = ttk.Combobox(self.bt_frame, textvariable=self.bt_var, width=45)
        self.bt_combo.pack(side='left', padx=5, fill='x', expand=True)

        self.bt_scan_btn = ttk.Button(self.bt_frame, text="Scan", command=self.scan_bluetooth, width=8)
        self.bt_scan_btn.pack(side='left', padx=2)

        self.bt_check_btn = ttk.Button(self.bt_frame, text="Check", command=self.check_bt_status, width=8)
        self.bt_check_btn.pack(side='left', padx=2)

        # Bluetooth status row
        self.bt_status_frame = ttk.Frame(conn_frame)
        self.bt_status_label = ttk.Label(self.bt_status_frame, text="BT: Not checked", style='BTStatus.TLabel')
        self.bt_status_label.pack(side='left', padx=5)

        # Serial connection widgets
        self.serial_frame = ttk.Frame(conn_frame)

        ttk.Label(self.serial_frame, text="Port:").pack(side='left', padx=(0, 5))

        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(self.serial_frame, textvariable=self.port_var, width=30)
        self.port_combo.pack(side='left', padx=5)

        ttk.Button(self.serial_frame, text="Refresh", command=self.refresh_ports, width=8).pack(side='left', padx=2)

        # Connection controls
        control_frame = ttk.Frame(conn_frame)
        control_frame.pack(fill='x', pady=(10, 0))

        self.connect_btn = ttk.Button(control_frame, text="Connect", command=self.toggle_connection, width=12)
        self.connect_btn.pack(side='left', padx=5)

        self.status_label = ttk.Label(control_frame, text="Disconnected", style='Disconnected.TLabel')
        self.status_label.pack(side='left', padx=10)

        # Response indicator
        self.response_label = ttk.Label(control_frame, text="", foreground='gray')
        self.response_label.pack(side='right', padx=10)

        # === Movement Controls Frame ===
        ctrl_frame = ttk.LabelFrame(main_frame, text="Movement Controls", padding=10)
        ctrl_frame.pack(fill='x', pady=5)

        # Parameters row
        param_frame = ttk.Frame(ctrl_frame)
        param_frame.pack(fill='x', pady=(0, 10))

        ttk.Label(param_frame, text="Speed:").pack(side='left', padx=(0, 5))
        self.speed_var = tk.IntVar(value=100)
        self.speed_spin = ttk.Spinbox(param_frame, from_=20, to=255, textvariable=self.speed_var, width=5)
        self.speed_spin.pack(side='left', padx=(0, 10))

        ttk.Label(param_frame, text="Distance:").pack(side='left', padx=(0, 5))
        self.ticks_var = tk.IntVar(value=1000)
        self.ticks_spin = ttk.Spinbox(param_frame, from_=100, to=10000, increment=100, textvariable=self.ticks_var, width=7)
        self.ticks_spin.pack(side='left', padx=(0, 10))

        # Preset buttons
        preset_frame = ttk.Frame(param_frame)
        preset_frame.pack(side='left', padx=10)
        for dist in [500, 1000, 2000, 5000]:
            ttk.Button(preset_frame, text=str(dist), width=5,
                       command=lambda d=dist: self.ticks_var.set(d)).pack(side='left', padx=1)

        # Direction buttons - 3x3 grid
        dir_frame = ttk.Frame(ctrl_frame)
        dir_frame.pack(pady=5)

        # Button configuration: (row, col, text, command)
        buttons = [
            (0, 0, "↖ FL", "DIAGFL"),
            (0, 1, "↑ FWD", "FWD"),
            (0, 2, "↗ FR", "DIAGFR"),
            (1, 0, "← LEFT", "LEFT"),
            (1, 1, "■ STOP", "STOP"),
            (1, 2, "→ RIGHT", "RIGHT"),
            (2, 0, "↙ BL", "DIAGBL"),
            (2, 1, "↓ BWD", "BWD"),
            (2, 2, "↘ BR", "DIAGBR"),
        ]

        for row, col, text, cmd in buttons:
            if cmd == "STOP":
                btn = ttk.Button(dir_frame, text=text, width=10, command=self.stop)
            else:
                btn = ttk.Button(dir_frame, text=text, width=10, command=lambda c=cmd: self.move(c))
            btn.grid(row=row, column=col, padx=3, pady=3)

        # Rotation buttons
        rotate_frame = ttk.Frame(ctrl_frame)
        rotate_frame.pack(pady=5)

        ttk.Button(rotate_frame, text="↺ Rotate Left", width=14,
                   command=lambda: self.move("TLEFT")).pack(side='left', padx=10)
        ttk.Button(rotate_frame, text="↻ Rotate Right", width=14,
                   command=lambda: self.move("TRIGHT")).pack(side='left', padx=10)

        # Utility buttons
        util_frame = ttk.Frame(ctrl_frame)
        util_frame.pack(pady=5)

        ttk.Button(util_frame, text="Read Encoders", command=self.read_encoders, width=14).pack(side='left', padx=5)
        ttk.Button(util_frame, text="Calibrate", command=self.calibrate, width=14).pack(side='left', padx=5)

        # === Manual Command Frame ===
        cmd_frame = ttk.LabelFrame(main_frame, text="Manual Command", padding=10)
        cmd_frame.pack(fill='x', pady=5)

        cmd_input_frame = ttk.Frame(cmd_frame)
        cmd_input_frame.pack(fill='x')

        self.cmd_var = tk.StringVar()
        self.cmd_entry = ttk.Entry(cmd_input_frame, textvariable=self.cmd_var, width=50)
        self.cmd_entry.pack(side='left', padx=(0, 5), fill='x', expand=True)
        self.cmd_entry.bind('<Return>', lambda e: self.send_manual())

        ttk.Button(cmd_input_frame, text="Send", command=self.send_manual, width=8).pack(side='left', padx=2)

        # Quick commands
        quick_frame = ttk.Frame(cmd_frame)
        quick_frame.pack(fill='x', pady=(5, 0))

        ttk.Label(quick_frame, text="Quick:").pack(side='left', padx=(0, 5))
        for cmd_name in ["STOP", "READ", "SYNC"]:
            ttk.Button(quick_frame, text=cmd_name, width=6,
                       command=lambda c=cmd_name: self.send_cmd(c)).pack(side='left', padx=2)

        # === Message Display Frame ===
        msg_frame = ttk.LabelFrame(main_frame, text="Last Response", padding=10)
        msg_frame.pack(fill='x', pady=5)

        # Large message display
        self.message_var = tk.StringVar(value="No response yet")
        self.message_label = ttk.Label(msg_frame, textvariable=self.message_var,
                                        style='Message.TLabel', anchor='center')
        self.message_label.pack(fill='x', pady=5)

        # Message details row
        msg_detail_frame = ttk.Frame(msg_frame)
        msg_detail_frame.pack(fill='x')

        self.msg_time_label = ttk.Label(msg_detail_frame, text="", foreground='gray', font=('Arial', 8))
        self.msg_time_label.pack(side='left')

        self.msg_latency_label = ttk.Label(msg_detail_frame, text="", foreground='gray', font=('Arial', 8))
        self.msg_latency_label.pack(side='right')

        # === Log Frame ===
        log_frame = ttk.LabelFrame(main_frame, text="Communication Log", padding=10)
        log_frame.pack(fill='both', expand=True, pady=5)

        self.log_text = scrolledtext.ScrolledText(log_frame, height=8, state='disabled',
                                                   font=('Consolas', 9), wrap='word')
        self.log_text.pack(fill='both', expand=True)

        # Configure log text tags
        self.log_text.tag_config('tx', foreground='#0066cc')
        self.log_text.tag_config('rx', foreground='#009933')
        self.log_text.tag_config('error', foreground='#cc0000')
        self.log_text.tag_config('info', foreground='#666666')
        self.log_text.tag_config('ok', foreground='#009933', font=('Consolas', 9, 'bold'))
        self.log_text.tag_config('done', foreground='#009933', font=('Consolas', 9, 'bold'))

        # Log controls
        log_ctrl_frame = ttk.Frame(log_frame)
        log_ctrl_frame.pack(fill='x', pady=(5, 0))

        self.autoscroll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(log_ctrl_frame, text="Auto-scroll", variable=self.autoscroll_var).pack(side='left')
        ttk.Button(log_ctrl_frame, text="Clear", command=self.clear_log, width=8).pack(side='right')

        # === Status Bar ===
        status_frame = ttk.Frame(main_frame)
        status_frame.pack(fill='x', pady=(5, 0))

        ttk.Label(status_frame, text="Keys: W/A/S/D=Move | Q/E=Rotate | 7/9/1/3=Diag | Space=Stop | R=Read",
                  font=('Arial', 8), foreground='gray').pack()

    def _bind_keys(self):
        """Bind keyboard shortcuts"""
        # Movement: WASD
        self.root.bind('<w>', lambda e: self.move("FWD"))
        self.root.bind('<W>', lambda e: self.move("FWD"))
        self.root.bind('<s>', lambda e: self.move("BWD"))
        self.root.bind('<S>', lambda e: self.move("BWD"))
        self.root.bind('<a>', lambda e: self.move("LEFT"))
        self.root.bind('<A>', lambda e: self.move("LEFT"))
        self.root.bind('<d>', lambda e: self.move("RIGHT"))
        self.root.bind('<D>', lambda e: self.move("RIGHT"))

        # Rotation: Q/E
        self.root.bind('<q>', lambda e: self.move("TLEFT"))
        self.root.bind('<Q>', lambda e: self.move("TLEFT"))
        self.root.bind('<e>', lambda e: self.move("TRIGHT"))
        self.root.bind('<E>', lambda e: self.move("TRIGHT"))

        # Diagonal: 7/9/1/3
        self.root.bind('<7>', lambda e: self.move("DIAGFL"))
        self.root.bind('<9>', lambda e: self.move("DIAGFR"))
        self.root.bind('<1>', lambda e: self.move("DIAGBL"))
        self.root.bind('<3>', lambda e: self.move("DIAGBR"))
        self.root.bind('<KP_7>', lambda e: self.move("DIAGFL"))
        self.root.bind('<KP_9>', lambda e: self.move("DIAGFR"))
        self.root.bind('<KP_1>', lambda e: self.move("DIAGBL"))
        self.root.bind('<KP_3>', lambda e: self.move("DIAGBR"))

        # Stop: Space or Escape
        self.root.bind('<space>', lambda e: self.stop())
        self.root.bind('<Escape>', lambda e: self.stop())

        # Read encoders: R
        self.root.bind('<r>', lambda e: self.read_encoders())
        self.root.bind('<R>', lambda e: self.read_encoders())

    def _update_status(self):
        """Periodic status update"""
        if self.robot.connected:
            # Check for response timeout
            if self.waiting_for_response:
                elapsed = time.time() - self.command_start_time
                if elapsed > 15.0:
                    self.response_label.config(text="No response", foreground='orange')
                    self.waiting_for_response = False
                else:
                    self.response_label.config(text=f"Waiting... {elapsed:.0f}s", foreground='gray')
        else:
            self.response_label.config(text="")

        # Schedule next update
        self.root.after(500, self._update_status)

    def on_connection_type_changed(self):
        """Handle connection type change"""
        conn_type = self.conn_type_var.get()

        # Hide all frames
        self.bt_frame.pack_forget()
        self.bt_status_frame.pack_forget()
        self.serial_frame.pack_forget()

        # Show selected frame
        if conn_type == "serial":
            self.serial_frame.pack(fill='x', pady=5)
        else:
            self.bt_frame.pack(fill='x', pady=5)
            self.bt_status_frame.pack(fill='x', pady=(0, 5))

    def check_bt_status(self):
        """Check Bluetooth device status"""
        bt_selection = self.bt_var.get()
        if not bt_selection:
            self.log("No device selected", 'error')
            self.bt_status_label.config(text="BT: No device selected", foreground='red')
            return

        try:
            bt_address = bt_selection.split('(')[1].split(')')[0]
        except IndexError:
            self.log("Invalid device selection", 'error')
            self.bt_status_label.config(text="BT: Invalid selection", foreground='red')
            return

        self.bt_status_label.config(text="BT: Checking...", foreground='gray')
        self.root.update()

        info = self.robot.check_bt_status(bt_address)

        status_parts = []
        status_parts.append("Paired" if info['paired'] else "Not Paired")
        if info['connected']:
            status_parts.append("Connected")
        if info['trusted']:
            status_parts.append("Trusted")

        status_str = ', '.join(status_parts)
        self.log(f"BT Status [{bt_address}]: {status_str}", 'info')

        # Update status label with color coding
        if info['paired'] and info['connected']:
            self.bt_status_label.config(text=f"BT: {status_str}", foreground='green')
        elif info['paired']:
            self.bt_status_label.config(text=f"BT: {status_str}", foreground='#0066cc')
        else:
            self.bt_status_label.config(text=f"BT: {status_str}", foreground='orange')

        if info['connected'] and not self.robot.connected:
            self.log("Warning: Device connected elsewhere", 'error')
            self.bt_status_label.config(text=f"BT: {status_str} (elsewhere!)", foreground='orange')

    def refresh_ports(self):
        """Refresh serial ports"""
        ports = self.robot.list_ports()
        arduino_ports = self.robot.detect_arduino()

        port_list = [p['device'] for p in ports]
        self.port_combo['values'] = port_list

        if arduino_ports:
            self.port_var.set(arduino_ports[0])
            self.log(f"Auto-detected: {arduino_ports[0]}", 'info')
        elif port_list:
            self.port_var.set(port_list[0])

    def scan_bluetooth(self):
        """Scan for Bluetooth devices"""
        if not BLUETOOTH_AVAILABLE:
            messagebox.showerror("Error", "Bluetooth not available")
            return

        self.bt_scan_btn.config(state='disabled', text="Scanning...")
        self.log("Scanning for Bluetooth devices...", 'info')

        def scan_thread():
            devices = self.robot.scan_bluetooth_devices()
            self.root.after(0, lambda: self._on_bt_scan_complete(devices))

        threading.Thread(target=scan_thread, daemon=True).start()

    def _on_bt_scan_complete(self, devices):
        """Handle Bluetooth scan results"""
        self.bt_scan_btn.config(state='normal', text="Scan")

        if not devices:
            self.log("No devices found", 'info')
            return

        existing = {d['address'] for d in self.bt_devices}
        new_devices = [d for d in devices if d['address'] not in existing]

        if new_devices:
            self.bt_devices.extend(new_devices)
            self.bt_combo['values'] = [f"{d['name']} ({d['address']})" for d in self.bt_devices]
            self.log(f"Found {len(new_devices)} new device(s)", 'info')
        else:
            self.log("No new devices found", 'info')

    def toggle_connection(self):
        """Connect or disconnect"""
        if self.robot.connected:
            self.robot.disconnect()
            self.connect_btn.config(text="Connect")
            self.status_label.config(text="Disconnected", style='Disconnected.TLabel')
            self.bt_status_label.config(text="BT: Disconnected", foreground='gray')
            self.message_var.set("Disconnected")
            self.message_label.config(foreground='gray')
            self.log("Disconnected", 'info')
        else:
            conn_type = self.conn_type_var.get()

            if conn_type == "serial":
                port = self.port_var.get()
                if not port:
                    messagebox.showerror("Error", "Select a port")
                    return

                success, msg = self.robot.connect(port)
                if success:
                    self._on_connected("Serial")
                    self.log(msg, 'info')
                else:
                    messagebox.showerror("Error", msg)
                    self.log(f"Error: {msg}", 'error')
            else:
                bt_selection = self.bt_var.get()
                if not bt_selection:
                    messagebox.showerror("Error", "Select a Bluetooth device")
                    return

                try:
                    bt_address = bt_selection.split('(')[1].split(')')[0]
                except IndexError:
                    messagebox.showerror("Error", "Invalid selection")
                    return

                self.log(f"Connecting to {bt_address}...", 'info')
                self.connect_btn.config(state='disabled', text="Connecting...")

                def connect_thread():
                    success, msg = self.robot.connect_bluetooth(bt_address)
                    self.root.after(0, lambda: self._on_bt_connect_result(success, msg))

                threading.Thread(target=connect_thread, daemon=True).start()

    def _on_connected(self, conn_type: str):
        """Handle successful connection"""
        self.connect_btn.config(text="Disconnect", state='normal')
        self.status_label.config(text=f"Connected ({conn_type})", style='Connected.TLabel')

        # Update BT status if Bluetooth connection
        if conn_type == "Bluetooth":
            self.bt_status_label.config(text="BT: Connected", foreground='green')

        # Update message display
        self.message_var.set("Connected - Ready for commands")
        self.message_label.config(foreground='green')

    def _on_bt_connect_result(self, success: bool, msg: str):
        """Handle Bluetooth connection result"""
        self.connect_btn.config(state='normal')

        if success:
            self._on_connected("Bluetooth")
            self.log(msg, 'ok')
        else:
            self.connect_btn.config(text="Connect")
            self.bt_status_label.config(text="BT: Connection failed", foreground='red')
            self.message_var.set(f"Connection failed: {msg}")
            self.message_label.config(foreground='red')
            messagebox.showerror("Connection Error", msg)
            self.log(f"Error: {msg}", 'error')

    def log(self, msg: str, tag: str = None):
        """Add message to log"""
        self.log_text.config(state='normal')
        timestamp = time.strftime("%H:%M:%S")
        line = f"[{timestamp}] {msg}\n"

        if tag:
            self.log_text.insert('end', line, tag)
        else:
            self.log_text.insert('end', line)

        if self.autoscroll_var.get():
            self.log_text.see('end')
        self.log_text.config(state='disabled')

    def clear_log(self):
        """Clear log"""
        self.log_text.config(state='normal')
        self.log_text.delete('1.0', 'end')
        self.log_text.config(state='disabled')

    def on_serial_data(self, data: str):
        """Handle received data"""
        # Calculate latency if we're waiting for a response
        latency_ms = 0
        if self.waiting_for_response and self.command_start_time > 0:
            latency_ms = int((time.time() - self.command_start_time) * 1000)

        # Determine tag and color based on response
        tag = 'rx'
        msg_color = '#333333'
        if data.startswith('DONE'):
            tag = 'done'
            msg_color = 'green'
            self.waiting_for_response = False
            self.response_label.config(text="Done", foreground='green')
        elif data.startswith('OK'):
            tag = 'ok'
            msg_color = '#009933'
        elif data.startswith('ERROR'):
            tag = 'error'
            msg_color = 'red'
            self.waiting_for_response = False
            self.response_label.config(text="Error", foreground='red')
        elif data.startswith('READY'):
            tag = 'ok'
            msg_color = '#009933'
        elif data.startswith('BUSY'):
            tag = 'error'
            msg_color = 'orange'
        elif data.startswith('STALL') or data.startswith('Stall'):
            tag = 'error'
            msg_color = 'red'
        elif data.startswith('Moving:'):
            tag = 'info'
            msg_color = '#666666'

        def update_ui():
            # Update log
            self.log(f"← {data}", tag)

            # Update prominent message display
            self.message_var.set(data)
            self.message_label.config(foreground=msg_color)

            # Update timestamp
            self.msg_time_label.config(text=time.strftime("%H:%M:%S"))

            # Update latency if applicable
            if latency_ms > 0:
                self.msg_latency_label.config(text=f"Latency: {latency_ms}ms")
            else:
                self.msg_latency_label.config(text="")

        self.root.after(0, update_ui)

    def on_connection_lost(self, error_msg: str):
        """Handle connection loss"""
        def update_ui():
            self.connect_btn.config(text="Connect", state='normal')
            self.status_label.config(text="Connection Lost!", style='Warning.TLabel')
            self.bt_status_label.config(text="BT: Disconnected", foreground='red')
            self.message_var.set(f"CONNECTION LOST: {error_msg}")
            self.message_label.config(foreground='red')
            self.log(f"CONNECTION LOST: {error_msg}", 'error')
            messagebox.showwarning("Connection Lost", f"Connection lost:\n{error_msg}")
        self.root.after(0, update_ui)

    def send_cmd(self, cmd: str):
        """Send command"""
        if self.robot.send(cmd):
            self.log(f"→ {cmd}", 'tx')
            self.waiting_for_response = True
            self.command_start_time = time.time()
            self.last_command = cmd
            self.response_label.config(text="Sent...", foreground='gray')
        else:
            self.log("Not connected", 'error')

    def send_manual(self):
        """Send manual command"""
        cmd = self.cmd_var.get().strip().upper()
        if cmd:
            self.send_cmd(cmd)
            self.cmd_var.set("")

    def move(self, direction: str):
        """Send movement command"""
        speed = self.speed_var.get()
        ticks = self.ticks_var.get()

        # Validate
        if speed < 20 or speed > 255:
            self.log(f"Invalid speed: {speed} (must be 20-255)", 'error')
            return
        if ticks < 1:
            self.log(f"Invalid ticks: {ticks} (must be > 0)", 'error')
            return

        # Build command
        cmd_map = {
            "FWD": f"FWD,{speed},{ticks}",
            "BWD": f"BWD,{speed},{ticks}",
            "LEFT": f"LEFT,{speed},{ticks}",
            "RIGHT": f"RIGHT,{speed},{ticks}",
            "TLEFT": f"TURN,{speed},{ticks}",
            "TRIGHT": f"TURN,{speed},{-ticks}",
            "DIAGFL": f"DIAGFL,{speed},{ticks}",
            "DIAGFR": f"DIAGFR,{speed},{ticks}",
            "DIAGBL": f"DIAGBL,{speed},{ticks}",
            "DIAGBR": f"DIAGBR,{speed},{ticks}",
        }

        if direction in cmd_map:
            self.send_cmd(cmd_map[direction])
        else:
            self.log(f"Unknown direction: {direction}", 'error')

    def stop(self):
        """Emergency stop"""
        self.send_cmd("STOP")

    def read_encoders(self):
        """Read encoders"""
        self.send_cmd("READ")

    def calibrate(self):
        """Calibrate motors"""
        if messagebox.askyesno("Calibrate", "Robot will move during calibration.\nContinue?"):
            self.send_cmd("SYNC")

    def run(self):
        """Start GUI"""
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()

    def on_close(self):
        """Handle window close"""
        self.robot.disconnect()
        self.root.destroy()


def main():
    app = RobotGUI()
    app.run()


if __name__ == "__main__":
    main()
