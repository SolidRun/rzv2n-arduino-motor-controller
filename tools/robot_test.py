#!/usr/bin/env python3
"""
Robot Test Utility for Mecanum 4WD Controller
GUI version - Works on Linux and Windows with auto port detection and Bluetooth support
"""

import serial
import serial.tools.list_ports
import time
import sys
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from typing import Optional

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
    def __init__(self):
        self.serial: Optional[serial.Serial] = None
        self.bt_socket = None
        self.connected = False
        self.connection_type = None  # 'serial' or 'bluetooth'
        self.read_thread: Optional[threading.Thread] = None
        self.running = False
        self.on_data_callback = None

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
            self.running = True

            # Start reader thread
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()

            return True, f"Connected to {port}"
        except serial.SerialException as e:
            return False, str(e)

    def connect_bluetooth(self, bt_address: str) -> tuple:
        """Connect to Arduino via Bluetooth. Returns (success, message)"""
        if not BLUETOOTH_AVAILABLE:
            return False, "Bluetooth library not available"

        try:
            print(f"[BT_CONNECT] Attempting to connect to {bt_address} on channel 1...")

            # Create Bluetooth socket
            self.bt_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            print(f"[BT_CONNECT] Socket created")

            # Connect to device on channel 1 (standard SPP channel)
            print(f"[BT_CONNECT] Connecting to ({bt_address}, 1)...")
            self.bt_socket.connect((bt_address, 1))
            print(f"[BT_CONNECT] Connected successfully!")

            self.bt_socket.settimeout(0.1)

            self.connected = True
            self.connection_type = 'bluetooth'
            self.running = True

            # Start reader thread
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            print(f"[BT_CONNECT] Reader thread started")

            return True, f"Connected to Bluetooth device {bt_address}"
        except bluetooth.BluetoothError as e:
            print(f"[BT_CONNECT] Bluetooth error: {e}")
            return False, f"Bluetooth connection error: {str(e)}"
        except Exception as e:
            print(f"[BT_CONNECT] Unexpected error: {e}")
            return False, str(e)

    def disconnect(self):
        """Disconnect from Arduino (Serial or Bluetooth)"""
        self.running = False
        self.connected = False
        if self.read_thread:
            self.read_thread.join(timeout=1)

        # Close serial connection
        if self.serial and self.serial.is_open:
            self.serial.close()

        # Close Bluetooth connection
        if self.bt_socket:
            try:
                self.bt_socket.close()
            except Exception:
                pass
            self.bt_socket = None

        self.connection_type = None

    def _read_loop(self):
        """Background thread to read data from Serial or Bluetooth"""
        buffer = ""

        while self.running:
            try:
                if self.connection_type == 'serial' and self.serial and self.serial.is_open:
                    if self.serial.in_waiting:
                        line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                        if line and self.on_data_callback:
                            self.on_data_callback(line)

                elif self.connection_type == 'bluetooth' and self.bt_socket:
                    try:
                        data = self.bt_socket.recv(128).decode('utf-8', errors='ignore')
                        if data:
                            buffer += data
                            # Process complete lines
                            while '\n' in buffer:
                                line, buffer = buffer.split('\n', 1)
                                line = line.strip()
                                if line and self.on_data_callback:
                                    self.on_data_callback(line)
                    except bluetooth.BluetoothError:
                        pass  # Timeout or no data
                    except Exception:
                        pass

            except Exception:
                pass
            time.sleep(0.05)

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
                return True
            elif self.connection_type == 'bluetooth' and self.bt_socket:
                print(f"[BT_TX] Sending via Bluetooth: {cmd} (bytes: {len(message)})")
                self.bt_socket.send(message)
                print(f"[BT_TX] Send successful")
                return True
            else:
                print(f"[SEND] No active connection (type={self.connection_type})")
                return False
        except Exception as e:
            print(f"[SEND] Error sending command: {e}")
            return False


class RobotGUI:
    # Default Bluetooth address for quick connection
    DEFAULT_BT_ADDRESS = "EC:5C:84:11:EC:97"
    DEFAULT_BT_NAME = "Arduino_BT_Bridge"

    def __init__(self):
        self.robot = RobotController()
        self.robot.on_data_callback = self.on_serial_data
        self.bt_devices = []

        # Create main window
        self.root = tk.Tk()
        self.root.title("Mecanum Robot Controller")
        self.root.geometry("750x650")
        self.root.resizable(True, True)

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
            # Refresh ports on start for serial mode
            self.refresh_ports()

    def _create_widgets(self):
        # Connection frame
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=10)
        conn_frame.pack(fill='x', padx=10, pady=5)

        # Connection type selection
        self.conn_type_var = tk.StringVar(value="bluetooth" if BLUETOOTH_AVAILABLE else "serial")

        type_frame = ttk.Frame(conn_frame)
        type_frame.grid(row=0, column=0, columnspan=5, sticky='w', pady=(0,5))

        ttk.Radiobutton(type_frame, text="Bluetooth", variable=self.conn_type_var,
                       value="bluetooth", command=self.on_connection_type_changed,
                       state='normal' if BLUETOOTH_AVAILABLE else 'disabled').pack(side='left', padx=5)
        ttk.Radiobutton(type_frame, text="Serial/USB", variable=self.conn_type_var,
                       value="serial", command=self.on_connection_type_changed).pack(side='left', padx=5)

        # Bluetooth connection widgets (shown first)
        self.bt_frame = ttk.Frame(conn_frame)
        self.bt_frame.grid(row=1, column=0, columnspan=5, sticky='ew', pady=5)

        ttk.Label(self.bt_frame, text="Address:").grid(row=0, column=0, padx=5, sticky='w')

        self.bt_var = tk.StringVar()
        self.bt_combo = ttk.Combobox(self.bt_frame, textvariable=self.bt_var, width=40)
        self.bt_combo.grid(row=0, column=1, padx=5, sticky='ew')

        self.bt_frame.columnconfigure(1, weight=1)

        self.bt_scan_btn = ttk.Button(self.bt_frame, text="Scan More", command=self.scan_bluetooth, width=12)
        self.bt_scan_btn.grid(row=0, column=2, padx=5)

        # Serial connection widgets
        self.serial_frame = ttk.Frame(conn_frame)
        self.serial_frame.grid(row=1, column=0, columnspan=5, sticky='ew', pady=5)

        ttk.Label(self.serial_frame, text="Port:").grid(row=0, column=0, padx=5)

        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(self.serial_frame, textvariable=self.port_var, width=25)
        self.port_combo.grid(row=0, column=1, padx=5)

        ttk.Button(self.serial_frame, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=5)

        # Connection controls
        control_frame = ttk.Frame(conn_frame)
        control_frame.grid(row=2, column=0, columnspan=5, pady=5)

        self.connect_btn = ttk.Button(control_frame, text="Connect", command=self.toggle_connection, width=15)
        self.connect_btn.pack(side='left', padx=5)

        self.status_label = ttk.Label(control_frame, text="Disconnected", foreground="red", font=('Arial', 10, 'bold'))
        self.status_label.pack(side='left', padx=10)

        # Control frame
        ctrl_frame = ttk.LabelFrame(self.root, text="Controls (use keyboard or buttons)", padding=10)
        ctrl_frame.pack(fill='x', padx=10, pady=5)

        # Speed and ticks
        param_frame = ttk.Frame(ctrl_frame)
        param_frame.pack(fill='x', pady=5)

        ttk.Label(param_frame, text="Speed:").pack(side='left', padx=5)
        self.speed_var = tk.IntVar(value=100)
        self.speed_spin = ttk.Spinbox(param_frame, from_=20, to=255, textvariable=self.speed_var, width=6)
        self.speed_spin.pack(side='left', padx=5)

        ttk.Label(param_frame, text="Ticks:").pack(side='left', padx=5)
        self.ticks_var = tk.IntVar(value=1000)
        self.ticks_spin = ttk.Spinbox(param_frame, from_=100, to=10000, increment=100, textvariable=self.ticks_var, width=8)
        self.ticks_spin.pack(side='left', padx=5)

        # Direction buttons
        btn_frame = ttk.Frame(ctrl_frame)
        btn_frame.pack(pady=10)

        btn_size = 8
        ttk.Button(btn_frame, text="↑\nFWD", width=btn_size, command=lambda: self.move("FWD")).grid(row=0, column=1, padx=2, pady=2)
        ttk.Button(btn_frame, text="←\nLEFT", width=btn_size, command=lambda: self.move("LEFT")).grid(row=1, column=0, padx=2, pady=2)
        ttk.Button(btn_frame, text="STOP", width=btn_size, command=self.stop).grid(row=1, column=1, padx=2, pady=2)
        ttk.Button(btn_frame, text="→\nRIGHT", width=btn_size, command=lambda: self.move("RIGHT")).grid(row=1, column=2, padx=2, pady=2)
        ttk.Button(btn_frame, text="↓\nBWD", width=btn_size, command=lambda: self.move("BWD")).grid(row=2, column=1, padx=2, pady=2)

        # Extra buttons
        extra_frame = ttk.Frame(ctrl_frame)
        extra_frame.pack(pady=5)

        ttk.Button(extra_frame, text="Turn Left", command=lambda: self.move("TLEFT")).pack(side='left', padx=5)
        ttk.Button(extra_frame, text="Turn Right", command=lambda: self.move("TRIGHT")).pack(side='left', padx=5)
        ttk.Button(extra_frame, text="Read Encoders", command=self.read_encoders).pack(side='left', padx=5)
        ttk.Button(extra_frame, text="Smart FWD", command=self.smart_forward).pack(side='left', padx=5)
        ttk.Button(extra_frame, text="Calibrate", command=self.calibrate).pack(side='left', padx=5)

        # Manual command frame
        cmd_frame = ttk.LabelFrame(self.root, text="Manual Command", padding=10)
        cmd_frame.pack(fill='x', padx=10, pady=5)

        self.cmd_var = tk.StringVar()
        self.cmd_entry = ttk.Entry(cmd_frame, textvariable=self.cmd_var, width=40)
        self.cmd_entry.pack(side='left', padx=5, fill='x', expand=True)
        self.cmd_entry.bind('<Return>', lambda e: self.send_manual())

        ttk.Button(cmd_frame, text="Send", command=self.send_manual).pack(side='left', padx=5)

        # Quick commands
        quick_frame = ttk.Frame(cmd_frame)
        quick_frame.pack(side='left', padx=10)
        ttk.Button(quick_frame, text="STOP", command=lambda: self.send_cmd("STOP"), width=6).pack(side='left', padx=2)
        ttk.Button(quick_frame, text="READ", command=lambda: self.send_cmd("READ"), width=6).pack(side='left', padx=2)
        ttk.Button(quick_frame, text="SYNC", command=lambda: self.send_cmd("SYNC"), width=6).pack(side='left', padx=2)

        # Log frame
        log_frame = ttk.LabelFrame(self.root, text="Communication Log", padding=10)
        log_frame.pack(fill='both', expand=True, padx=10, pady=5)

        self.log_text = scrolledtext.ScrolledText(log_frame, height=12, state='disabled', font=('Consolas', 9))
        self.log_text.pack(fill='both', expand=True)

        # Configure log text tags for better readability
        self.log_text.tag_config('tx', foreground='blue')
        self.log_text.tag_config('rx', foreground='green')
        self.log_text.tag_config('error', foreground='red')
        self.log_text.tag_config('info', foreground='gray')

        # Log buttons
        log_btn_frame = ttk.Frame(log_frame)
        log_btn_frame.pack(fill='x', pady=5)

        self.autoscroll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(log_btn_frame, text="Auto-scroll", variable=self.autoscroll_var).pack(side='left')
        ttk.Button(log_btn_frame, text="Clear Log", command=self.clear_log).pack(side='right')

        # Keyboard hints
        hint_frame = ttk.Frame(self.root)
        hint_frame.pack(fill='x', padx=10, pady=5)
        ttk.Label(hint_frame, text="Keyboard: W/S/A/D = Move | Q/E = Turn | Space = Stop | R = Read",
                  font=('Arial', 9), foreground='gray').pack()

    def _bind_keys(self):
        """Bind keyboard shortcuts"""
        self.root.bind('<w>', lambda e: self.move("FWD"))
        self.root.bind('<W>', lambda e: self.move("FWD"))
        self.root.bind('<s>', lambda e: self.move("BWD"))
        self.root.bind('<S>', lambda e: self.move("BWD"))
        self.root.bind('<a>', lambda e: self.move("LEFT"))
        self.root.bind('<A>', lambda e: self.move("LEFT"))
        self.root.bind('<d>', lambda e: self.move("RIGHT"))
        self.root.bind('<D>', lambda e: self.move("RIGHT"))
        self.root.bind('<q>', lambda e: self.move("TLEFT"))
        self.root.bind('<Q>', lambda e: self.move("TLEFT"))
        self.root.bind('<e>', lambda e: self.move("TRIGHT"))
        self.root.bind('<E>', lambda e: self.move("TRIGHT"))
        self.root.bind('<space>', lambda e: self.stop())
        self.root.bind('<r>', lambda e: self.read_encoders())
        self.root.bind('<R>', lambda e: self.read_encoders())
        self.root.bind('<Escape>', lambda e: self.stop())

    def on_connection_type_changed(self):
        """Handle connection type change (Serial/Bluetooth)"""
        if self.conn_type_var.get() == "serial":
            self.bt_frame.grid_remove()
            self.serial_frame.grid()
        else:
            self.serial_frame.grid_remove()
            self.bt_frame.grid()

    def refresh_ports(self):
        """Refresh available serial ports"""
        ports = self.robot.list_ports()
        arduino_ports = self.robot.detect_arduino()

        port_list = [p['device'] for p in ports]
        self.port_combo['values'] = port_list

        # Auto-select Arduino if found
        if arduino_ports:
            self.port_var.set(arduino_ports[0])
            self.log(f"Auto-detected Arduino: {arduino_ports[0]}")
        elif port_list:
            self.port_var.set(port_list[0])

    def scan_bluetooth(self):
        """Scan for Bluetooth devices"""
        if not BLUETOOTH_AVAILABLE:
            messagebox.showerror("Error", "Bluetooth library not available.\nInstall with: pip install pybluez-updated")
            return

        self.bt_scan_btn.config(state='disabled', text="Scanning...")
        self.log("Scanning for Bluetooth devices (8 seconds)...", 'info')

        def scan_thread():
            devices = self.robot.scan_bluetooth_devices()
            self.root.after(0, lambda: self.on_bluetooth_scan_complete(devices))

        threading.Thread(target=scan_thread, daemon=True).start()

    def on_bluetooth_scan_complete(self, devices):
        """Handle Bluetooth scan results"""
        self.bt_scan_btn.config(state='normal', text="Scan More")

        if not devices:
            self.log("No new Bluetooth devices found", 'info')
            return

        # Merge with existing devices (keep default + add new ones)
        existing_addrs = {dev['address'] for dev in self.bt_devices}
        new_devices = [dev for dev in devices if dev['address'] not in existing_addrs]

        if new_devices:
            self.bt_devices.extend(new_devices)
            device_list = [f"{dev['name']} ({dev['address']})" for dev in self.bt_devices]
            self.bt_combo['values'] = device_list
            self.log(f"Found {len(new_devices)} new device(s). Total: {len(self.bt_devices)}", 'info')
        else:
            self.log("No new devices found. All scanned devices already listed.", 'info')

    def toggle_connection(self):
        """Connect or disconnect (Serial or Bluetooth)"""
        if self.robot.connected:
            self.robot.disconnect()
            self.connect_btn.config(text="Connect")
            self.status_label.config(text="✗ Disconnected", foreground="red")
            self.log("Disconnected", 'info')
        else:
            conn_type = self.conn_type_var.get()

            if conn_type == "serial":
                # Serial connection
                port = self.port_var.get()
                if not port:
                    messagebox.showerror("Error", "Please select a port")
                    return

                success, msg = self.robot.connect(port)
                if success:
                    self.connect_btn.config(text="Disconnect")
                    self.status_label.config(text="✓ Connected (Serial)", foreground="green")
                    self.log(msg, 'info')
                else:
                    messagebox.showerror("Connection Error", msg)
                    self.log(f"Error: {msg}", 'error')

            else:
                # Bluetooth connection
                bt_selection = self.bt_var.get()
                if not bt_selection:
                    messagebox.showerror("Error", "Please scan and select a Bluetooth device")
                    return

                # Extract address from selection (format: "Name (XX:XX:XX:XX:XX:XX)")
                try:
                    bt_address = bt_selection.split('(')[1].split(')')[0]
                except IndexError:
                    messagebox.showerror("Error", "Invalid device selection")
                    return

                self.log(f"Connecting to Bluetooth: {bt_address}...", 'info')
                success, msg = self.robot.connect_bluetooth(bt_address)

                if success:
                    self.connect_btn.config(text="Disconnect")
                    self.status_label.config(text="✓ Connected (Bluetooth)", foreground="green")
                    self.log(f"✓ {msg}", 'info')
                else:
                    messagebox.showerror("Connection Error", msg)
                    self.log(f"✗ Error: {msg}", 'error')

    def log(self, msg: str, tag: str = None):
        """Add message to log with optional color tag"""
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
        """Callback when data received from Arduino"""
        self.root.after(0, lambda: self.log(f"← RX: {data}", 'rx'))

    def send_cmd(self, cmd: str):
        """Send command and log it"""
        if self.robot.send(cmd):
            self.log(f"→ TX: {cmd}", 'tx')
        else:
            self.log("Error: Not connected", 'error')

    def send_manual(self):
        """Send manual command from entry"""
        cmd = self.cmd_var.get().strip()
        if cmd:
            self.send_cmd(cmd)
            self.cmd_var.set("")

    def move(self, direction: str):
        """Send movement command"""
        speed = self.speed_var.get()
        ticks = self.ticks_var.get()

        if direction == "FWD":
            self.send_cmd(f"FWD,{speed},{ticks}")
        elif direction == "BWD":
            self.send_cmd(f"BWD,{speed},{ticks}")
        elif direction == "LEFT":
            self.send_cmd(f"TURN,{speed},{ticks}")
        elif direction == "RIGHT":
            self.send_cmd(f"TURN,{speed},{-ticks}")
        elif direction == "TLEFT":
            # For rotation, we need different logic - using raw motor control
            self.log("Turn left not implemented in protocol")
        elif direction == "TRIGHT":
            self.log("Turn right not implemented in protocol")

    def stop(self):
        """Emergency stop"""
        self.send_cmd("STOP")

    def read_encoders(self):
        """Read encoder values"""
        self.send_cmd("READ")

    def smart_forward(self):
        """Smart forward movement"""
        self.send_cmd("FWDS")

    def calibrate(self):
        """Calibrate motors"""
        if messagebox.askyesno("Calibrate", "Robot will move during calibration. Continue?"):
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
