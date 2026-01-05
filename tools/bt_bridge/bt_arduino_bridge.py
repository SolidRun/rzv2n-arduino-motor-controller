#!/usr/bin/env python3
"""
Bluetooth to Arduino Bridge Service (Python Version)
Main service that manages Bluetooth setup and handles RFCOMM connections directly.

Uses BluetoothSocket server instead of rfcomm watch for reliable connections.
"""

import os
import sys
import subprocess
import time
import signal
import logging
import socket
import threading
import serial
from pathlib import Path

# Configuration
BT_CHANNEL = 1
ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUD = 9600
PID_FILE = "/var/run/bt_arduino_bridge.pid"
LOG_DIR = "/tmp"
DEBUG_LOG = f"{LOG_DIR}/bt_bridge_debug.log"
CMD_LOG = f"{LOG_DIR}/bt_bridge_commands.log"
SERVICE_LOG = f"{LOG_DIR}/bt_arduino_bridge.log"

# Get the directory where this script is located
SCRIPT_DIR = Path(__file__).parent.resolve()

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[
        logging.FileHandler(SERVICE_LOG),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)


class ArduinoConnection:
    """Manages Arduino serial connection"""

    def __init__(self, port=ARDUINO_PORT, baudrate=ARDUINO_BAUD):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self._lock = threading.Lock()

    def connect(self):
        """Connect to Arduino"""
        try:
            # Try to find Arduino if default port doesn't exist
            if not Path(self.port).exists():
                self.port = self._find_arduino()
                if not self.port:
                    logger.warning("Arduino not found, will retry on each command")
                    return False

            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                write_timeout=1.0
            )
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

            # Wait for Arduino reset
            time.sleep(2)
            self.serial.reset_input_buffer()

            logger.info(f"Arduino connected: {self.port} @ {self.baudrate}")
            return True

        except Exception as e:
            logger.error(f"Failed to connect to Arduino: {e}")
            self.serial = None
            return False

    def _find_arduino(self):
        """Find Arduino port"""
        # Check /dev/serial/by-id first
        by_id_path = Path("/dev/serial/by-id")
        if by_id_path.exists():
            for port in by_id_path.glob("*Arduino*"):
                return str(port.resolve())
            for port in by_id_path.glob("*"):
                return str(port.resolve())

        # Check common ports
        for port in ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"]:
            if Path(port).exists():
                return port

        return None

    def send(self, data):
        """Send data to Arduino"""
        with self._lock:
            if not self.serial or not self.serial.is_open:
                if not self.connect():
                    return False

            try:
                self.serial.write(data)
                self.serial.flush()
                return True
            except Exception as e:
                logger.error(f"Arduino send error: {e}")
                self.serial = None
                return False

    def read(self, timeout=0.1):
        """Read data from Arduino"""
        with self._lock:
            if not self.serial or not self.serial.is_open:
                return None

            try:
                if self.serial.in_waiting > 0:
                    return self.serial.read(self.serial.in_waiting)
                return None
            except Exception as e:
                logger.error(f"Arduino read error: {e}")
                return None

    def close(self):
        """Close Arduino connection"""
        with self._lock:
            if self.serial and self.serial.is_open:
                try:
                    self.serial.close()
                except Exception:
                    pass
            self.serial = None


class ClientHandler(threading.Thread):
    """Handles a single Bluetooth client connection"""

    def __init__(self, client_socket, client_info, arduino):
        super().__init__(daemon=True)
        self.client_socket = client_socket
        self.client_info = client_info
        self.arduino = arduino
        self.running = True
        self._last_cmd = ""

    def run(self):
        """Handle client connection - bidirectional relay"""
        logger.info(f"=== NEW CLIENT: {self.client_info} ===")

        # Log to command log
        self._log_cmd(f"[CONNECT] Client connected: {self.client_info}")

        # Send welcome message
        try:
            self.client_socket.send(b"OK:CONNECTED\n")
        except Exception as e:
            logger.error(f"Failed to send welcome: {e}")

        bt_buffer = ""
        arduino_buffer = ""

        try:
            while self.running:
                # Read from Bluetooth client
                try:
                    self.client_socket.setblocking(False)
                    data = self.client_socket.recv(1024)

                    if not data:
                        logger.info("Client disconnected (EOF)")
                        break

                    bt_buffer += data.decode('utf-8', errors='ignore')

                    # Process complete lines
                    while '\n' in bt_buffer or '\r' in bt_buffer:
                        if '\n' in bt_buffer:
                            line, bt_buffer = bt_buffer.split('\n', 1)
                        else:
                            line, bt_buffer = bt_buffer.split('\r', 1)

                        cmd = line.strip()
                        if cmd:
                            logger.info(f"<<< BT RX: [{cmd}]")
                            self._log_cmd(f"[BT_RX] {cmd}")
                            self._last_cmd = cmd

                            # Forward to Arduino
                            if self.arduino.send(f"{cmd}\n".encode()):
                                logger.info(f">>> ARDUINO TX: [{cmd}]")
                                self._log_cmd(f"[ARDUINO_TX] {cmd}")

                except BlockingIOError:
                    pass  # No data available
                except socket.error as e:
                    if e.errno not in (11, 35):  # EAGAIN, EWOULDBLOCK
                        logger.error(f"BT recv error: {e}")
                        break
                except Exception as e:
                    logger.error(f"BT read error: {e}")
                    break

                # Read from Arduino and forward to Bluetooth
                arduino_data = self.arduino.read()
                if arduino_data:
                    arduino_buffer += arduino_data.decode('utf-8', errors='ignore')

                    while '\n' in arduino_buffer:
                        line, arduino_buffer = arduino_buffer.split('\n', 1)
                        line = line.strip()

                        if line:
                            # Filter echo of command we just sent
                            if line == self._last_cmd:
                                logger.info(f"<<< ARDUINO RX (echo filtered): [{line}]")
                                self._last_cmd = ""
                            else:
                                logger.info(f"<<< ARDUINO RX: [{line}]")
                                self._log_cmd(f"[ARDUINO_RX] {line}")

                                # Forward to Bluetooth client
                                try:
                                    self.client_socket.setblocking(True)
                                    self.client_socket.settimeout(1.0)
                                    self.client_socket.send(f"{line}\n".encode())
                                    logger.info(f">>> BT TX: [{line}]")
                                    self._log_cmd(f"[BT_TX] {line}")
                                except Exception as e:
                                    logger.error(f"BT send error: {e}")
                                    break

                time.sleep(0.02)  # Small delay to prevent CPU spinning

        except Exception as e:
            logger.error(f"Client handler error: {e}")

        finally:
            logger.info(f"=== CLIENT DISCONNECTED: {self.client_info} ===")
            self._log_cmd(f"[DISCONNECT] Client disconnected: {self.client_info}")
            try:
                self.client_socket.close()
            except Exception:
                pass

    def _log_cmd(self, msg):
        """Log to command log file"""
        try:
            from datetime import datetime
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            with open(CMD_LOG, 'a') as f:
                f.write(f"[{timestamp}] {msg}\n")
        except Exception:
            pass

    def stop(self):
        """Stop the handler"""
        self.running = False


class BluetoothBridge:
    """Main Bluetooth bridge service using socket server"""

    def __init__(self):
        self.running = True
        self.server_socket = None
        self.agent_process = None
        self.bt_address = None
        self.arduino = ArduinoConnection()
        self.clients = []

    def setup_signal_handlers(self):
        """Setup signal handlers for graceful shutdown"""
        signal.signal(signal.SIGTERM, self.signal_handler)
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        logger.info(f"Received signal {signum}, shutting down...")
        self.running = False
        self.cleanup()
        sys.exit(0)

    def run_command(self, cmd, shell=False, check=False, timeout=None):
        """Run a shell command and return result"""
        try:
            result = subprocess.run(
                cmd if shell else cmd.split(),
                shell=shell,
                capture_output=True,
                text=True,
                timeout=timeout,
                check=check
            )
            return result.returncode == 0, result.stdout, result.stderr
        except subprocess.TimeoutExpired:
            logger.error(f"Command timed out: {cmd}")
            return False, "", "Timeout"
        except subprocess.CalledProcessError as e:
            logger.error(f"Command failed: {cmd} - {e}")
            return False, "", str(e)
        except Exception as e:
            logger.error(f"Error running command: {e}")
            return False, "", str(e)

    def cleanup_stale_connections(self):
        """Clean up any stale rfcomm connections from previous sessions"""
        logger.info("Cleaning up stale connections...")

        # Release any existing rfcomm devices
        for i in range(5):
            self.run_command(f"rfcomm release /dev/rfcomm{i}")

        # Kill any leftover rfcomm processes
        self.run_command("pkill -9 rfcomm")

        # Disconnect any connected Bluetooth devices
        success, stdout, _ = self.run_command("bluetoothctl paired-devices")
        if success and stdout:
            for line in stdout.strip().split('\n'):
                if line.startswith("Device "):
                    parts = line.split()
                    if len(parts) >= 2:
                        addr = parts[1]
                        # Check if connected
                        conn_success, conn_stdout, _ = self.run_command(f"bluetoothctl info {addr}")
                        if conn_success and "Connected: yes" in conn_stdout:
                            logger.info(f"Disconnecting stale connection: {addr}")
                            self.run_command(f"bluetoothctl disconnect {addr}")
                            time.sleep(0.5)

        logger.info("Stale connection cleanup complete")

    def setup_bluetooth(self):
        """Initialize and configure Bluetooth adapter"""
        logger.info("Setting up Bluetooth...")

        # Clean up stale connections first
        self.cleanup_stale_connections()

        # Unblock rfkill
        logger.info("Unblocking rfkill...")
        self.run_command("rfkill unblock all")
        time.sleep(1)

        # Bring up hci0
        logger.info("Bringing up hci0...")
        for attempt in range(1, 6):
            success, _, _ = self.run_command("hciconfig hci0 up")
            if success:
                logger.info("hci0 is UP")
                break
            logger.debug(f"Attempt {attempt}/5 to bring up hci0")
            time.sleep(2)
        else:
            logger.error("Failed to bring up hci0 after 5 attempts")
            return False

        # Get Bluetooth address
        success, stdout, _ = self.run_command("hciconfig hci0")
        if success:
            for line in stdout.split('\n'):
                if "BD Address" in line:
                    parts = line.split()
                    for i, p in enumerate(parts):
                        if p == "Address:":
                            self.bt_address = parts[i+1]
                            break
                        # Alternative format: "BD Address: XX:XX:XX:XX:XX:XX"
                        if ":" in p and len(p) == 17:
                            self.bt_address = p
                            break
                    logger.info(f"Bluetooth address: {self.bt_address}")
                    break

        # Set device name
        logger.info("Setting device name to Arduino_BT_Bridge...")
        self.run_command("hciconfig hci0 name 'Arduino_BT_Bridge'")

        # Also set via machine-info
        try:
            with open('/etc/machine-info', 'w') as f:
                f.write("PRETTY_HOSTNAME=Arduino_BT_Bridge\n")
        except Exception as e:
            logger.debug(f"Could not write /etc/machine-info: {e}")

        # Configure basic settings with bluetoothctl
        logger.info("Configuring Bluetooth settings...")
        bt_commands = """power on
system-alias Arduino_BT_Bridge
discoverable on
pairable on
quit
"""
        try:
            proc = subprocess.Popen(
                ['bluetoothctl'],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            stdout, _ = proc.communicate(input=bt_commands, timeout=10)
            logger.debug(f"bluetoothctl config output: {stdout.strip()}")
        except Exception as e:
            logger.warning(f"bluetoothctl configuration warning: {e}")

        # Start persistent agent for auto-accepting pairing
        self.start_bt_agent()

        # Make discoverable using hciconfig
        logger.info("Setting discoverable and pairable...")
        self.run_command("hciconfig hci0 piscan")

        # Set device class to indicate serial port capability
        # Class 0x001F00 = Uncategorized device with Serial Port service
        self.run_command("hciconfig hci0 class 0x001f00")

        # Trust all already-paired devices
        self._trust_all_paired_devices()

        logger.info("Bluetooth setup complete")
        return True

    def start_bt_agent(self):
        """Start a D-Bus Bluetooth agent that auto-accepts pairing instantly"""
        logger.info("Starting Bluetooth pairing agent...")

        # Try D-Bus agent first (instant response), fall back to bluetoothctl
        if self._start_dbus_agent():
            return

        # Fallback: Use bt-agent command if available
        if self._start_bt_agent_command():
            return

        # Last resort: bluetoothctl with NoInputNoOutput capability
        self._start_bluetoothctl_agent()

    def _start_dbus_agent(self):
        """Start a proper D-Bus agent using Python dbus library"""
        try:
            import dbus
            import dbus.service
            import dbus.mainloop.glib
            from gi.repository import GLib

            logger.info("Starting D-Bus Bluetooth agent...")

            # Setup D-Bus
            dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
            bus = dbus.SystemBus()

            AGENT_INTERFACE = "org.bluez.Agent1"
            AGENT_PATH = "/org/bluez/AutoAcceptAgent"

            class AutoAcceptAgent(dbus.service.Object):
                """D-Bus agent that auto-accepts all pairing requests"""

                @dbus.service.method(AGENT_INTERFACE, in_signature="", out_signature="")
                def Release(self):
                    logger.info("Agent: Released")

                @dbus.service.method(AGENT_INTERFACE, in_signature="os", out_signature="")
                def AuthorizeService(self, device, uuid):
                    logger.info(f"Agent: AuthorizeService {device} {uuid} -> ACCEPTED")

                @dbus.service.method(AGENT_INTERFACE, in_signature="o", out_signature="s")
                def RequestPinCode(self, device):
                    logger.info(f"Agent: RequestPinCode {device} -> 0000")
                    return "0000"

                @dbus.service.method(AGENT_INTERFACE, in_signature="o", out_signature="u")
                def RequestPasskey(self, device):
                    logger.info(f"Agent: RequestPasskey {device} -> 0")
                    return dbus.UInt32(0)

                @dbus.service.method(AGENT_INTERFACE, in_signature="ouq", out_signature="")
                def DisplayPasskey(self, device, passkey, entered):
                    logger.info(f"Agent: DisplayPasskey {device} {passkey}")

                @dbus.service.method(AGENT_INTERFACE, in_signature="os", out_signature="")
                def DisplayPinCode(self, device, pincode):
                    logger.info(f"Agent: DisplayPinCode {device} {pincode}")

                @dbus.service.method(AGENT_INTERFACE, in_signature="ou", out_signature="")
                def RequestConfirmation(self, device, passkey):
                    logger.info(f"Agent: RequestConfirmation {device} {passkey} -> CONFIRMED")
                    # Just return - empty return means confirmation accepted

                @dbus.service.method(AGENT_INTERFACE, in_signature="o", out_signature="")
                def RequestAuthorization(self, device):
                    logger.info(f"Agent: RequestAuthorization {device} -> AUTHORIZED")

                @dbus.service.method(AGENT_INTERFACE, in_signature="", out_signature="")
                def Cancel(self):
                    logger.info("Agent: Cancel")

            # Create and register agent
            agent = AutoAcceptAgent(bus, AGENT_PATH)

            # Get AgentManager
            agent_manager = dbus.Interface(
                bus.get_object("org.bluez", "/org/bluez"),
                "org.bluez.AgentManager1"
            )

            # Unregister any existing agent first
            try:
                agent_manager.UnregisterAgent(AGENT_PATH)
            except Exception:
                pass

            # Register our agent with NoInputNoOutput capability
            agent_manager.RegisterAgent(AGENT_PATH, "NoInputNoOutput")
            agent_manager.RequestDefaultAgent(AGENT_PATH)

            logger.info("D-Bus Bluetooth agent registered successfully")

            # Run the main loop in a separate thread
            def run_dbus_loop():
                loop = GLib.MainLoop()
                self._dbus_loop = loop
                try:
                    loop.run()
                except Exception as e:
                    logger.debug(f"D-Bus loop ended: {e}")

            self._dbus_agent = agent
            self._dbus_thread = threading.Thread(target=run_dbus_loop, daemon=True)
            self._dbus_thread.start()

            return True

        except ImportError as e:
            logger.info(f"D-Bus libraries not available: {e}")
            return False
        except Exception as e:
            logger.warning(f"Could not start D-Bus agent: {e}")
            return False

    def _start_bt_agent_command(self):
        """Try to use bt-agent command (from bluez-tools)"""
        try:
            # Check if bt-agent exists
            result = subprocess.run(["which", "bt-agent"], capture_output=True)
            if result.returncode != 0:
                return False

            logger.info("Starting bt-agent for auto-pairing...")

            # IMPORTANT: Use NoInputNoOutput FIRST - this allows automatic pairing
            # without any user confirmation. DisplayYesNo requires manual confirmation
            # which bt-agent cannot handle automatically.
            # NoInputNoOutput tells the remote device we can't display or input anything,
            # so it should just accept the pairing.
            for capability in ["NoInputNoOutput", "KeyboardDisplay", "DisplayYesNo"]:
                logger.info(f"Trying bt-agent with capability: {capability}")

                self.agent_process = subprocess.Popen(
                    ["bt-agent", "-c", capability],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True
                )

                time.sleep(1)

                if self.agent_process.poll() is None:
                    logger.info(f"bt-agent started successfully with {capability}")

                    # Monitor thread for logging and auto-confirmation
                    def monitor():
                        while self.running and self.agent_process and self.agent_process.poll() is None:
                            try:
                                line = self.agent_process.stdout.readline()
                                if line:
                                    line = line.strip()
                                    logger.info(f"bt-agent: {line}")

                                    # Auto-confirm any passkey/PIN requests
                                    line_lower = line.lower()
                                    if "confirm" in line_lower or "passkey" in line_lower or "yes/no" in line_lower:
                                        logger.info(">>> Auto-confirming pairing request")
                                        try:
                                            self.agent_process.stdin.write("yes\n")
                                            self.agent_process.stdin.flush()
                                        except Exception:
                                            pass
                                    elif "pin" in line_lower or "enter" in line_lower:
                                        logger.info(">>> Sending PIN 0000")
                                        try:
                                            self.agent_process.stdin.write("0000\n")
                                            self.agent_process.stdin.flush()
                                        except Exception:
                                            pass
                            except Exception:
                                break

                    self.agent_thread = threading.Thread(target=monitor, daemon=True)
                    self.agent_thread.start()
                    return True
                else:
                    # Try next capability
                    logger.info(f"bt-agent with {capability} failed, trying next...")

            return False

        except Exception as e:
            logger.debug(f"bt-agent not available: {e}")
            return False

    def _start_bluetoothctl_agent(self):
        """Fallback: Use bluetoothctl with expect-like interaction"""
        logger.info("Starting bluetoothctl agent (fallback)...")

        try:
            # Use pexpect-like approach with direct stdin/stdout
            self.agent_process = subprocess.Popen(
                ['bluetoothctl'],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=0  # Unbuffered for faster response
            )

            # Register agent - use NoInputNoOutput for automatic pairing
            # This tells the remote device we can't display or confirm anything,
            # so pairing should proceed automatically with Just Works pairing
            commands = [
                "agent NoInputNoOutput",
                "default-agent",
            ]

            for cmd in commands:
                self.agent_process.stdin.write(cmd + "\n")
                self.agent_process.stdin.flush()
                time.sleep(0.3)

            logger.info("bluetoothctl agent started with NoInputNoOutput capability")

            # Monitor and auto-respond
            self.agent_thread = threading.Thread(target=self._monitor_bluetoothctl_agent, daemon=True)
            self.agent_thread.start()

        except Exception as e:
            logger.warning(f"Could not start bluetoothctl agent: {e}")
            self.agent_process = None

    def _monitor_bluetoothctl_agent(self):
        """Monitor bluetoothctl and respond to prompts instantly"""
        import select
        import re

        while self.running and self.agent_process and self.agent_process.poll() is None:
            try:
                # Use very short timeout for fast response
                ready, _, _ = select.select([self.agent_process.stdout], [], [], 0.1)

                if ready:
                    line = self.agent_process.stdout.readline()
                    if line:
                        line = line.strip()
                        if line:
                            logger.info(f"Agent: {line}")

                        line_lower = line.lower()

                        # Respond IMMEDIATELY to any prompt
                        if "(yes/no)" in line_lower or "confirm passkey" in line_lower:
                            logger.info(">>> Sending 'yes' immediately")
                            self.agent_process.stdin.write("yes\n")
                            self.agent_process.stdin.flush()
                        elif "passkey" in line_lower and "enter" in line_lower:
                            logger.info(">>> Sending passkey '0000'")
                            self.agent_process.stdin.write("0000\n")
                            self.agent_process.stdin.flush()
                        elif "pin" in line_lower and ("enter" in line_lower or "code" in line_lower):
                            logger.info(">>> Sending PIN '0000'")
                            self.agent_process.stdin.write("0000\n")
                            self.agent_process.stdin.flush()

                        # Auto-trust newly paired devices
                        # Look for "Device XX:XX:XX:XX:XX:XX Paired: yes"
                        if "paired: yes" in line_lower:
                            # Extract MAC address
                            mac_match = re.search(r'([0-9A-Fa-f]{2}[:-]){5}[0-9A-Fa-f]{2}', line)
                            if mac_match:
                                mac = mac_match.group(0)
                                logger.info(f">>> Auto-trusting device: {mac}")
                                self.agent_process.stdin.write(f"trust {mac}\n")
                                self.agent_process.stdin.flush()

                        # Also trust on new device connection
                        if "device" in line_lower and "connected: yes" in line_lower:
                            mac_match = re.search(r'([0-9A-Fa-f]{2}[:-]){5}[0-9A-Fa-f]{2}', line)
                            if mac_match:
                                mac = mac_match.group(0)
                                logger.info(f">>> Device connected, ensuring trust: {mac}")
                                self.agent_process.stdin.write(f"trust {mac}\n")
                                self.agent_process.stdin.flush()

            except Exception as e:
                if self.running:
                    logger.debug(f"Agent monitor error: {e}")
                break

    def ensure_bluetoothd_running(self):
        """Ensure bluetoothd is running"""
        # Check if bluetoothd is running
        success, stdout, _ = self.run_command("pgrep -x bluetoothd")
        if success and stdout.strip():
            logger.info("bluetoothd is running")
            return True

        logger.warning("bluetoothd not running, starting it...")

        # Try systemctl first
        self.run_command("systemctl start bluetooth")
        time.sleep(2)

        # Check again
        success, stdout, _ = self.run_command("pgrep -x bluetoothd")
        if success and stdout.strip():
            logger.info("bluetoothd started via systemctl")
            return True

        # Manual start as fallback
        for path in ["/usr/lib/bluetooth/bluetoothd", "/usr/libexec/bluetooth/bluetoothd"]:
            if Path(path).exists():
                logger.info(f"Starting bluetoothd manually: {path}")
                subprocess.Popen(
                    [path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
                time.sleep(2)
                return True

        logger.error("Could not start bluetoothd!")
        return False

    def register_spp_service(self):
        """Register Serial Port Profile service"""
        logger.info("Registering Serial Port Profile service...")

        # Ensure bluetoothd is running first
        self.ensure_bluetoothd_running()

        # Check if SDP server is accessible
        success, stdout, stderr = self.run_command("sdptool browse local")
        if not success or "Failed to connect" in stderr:
            logger.warning("SDP server not accessible.")
            logger.warning("bluetoothd needs --compat flag for sdptool.")
            logger.warning("Run install.sh to configure systemd override, or manually:")
            logger.warning("  1. Create /etc/systemd/system/bluetooth.service.d/compat.conf with:")
            logger.warning("     [Service]")
            logger.warning("     ExecStart=")
            logger.warning("     ExecStart=/usr/libexec/bluetooth/bluetoothd --compat")
            logger.warning("  2. Run: systemctl daemon-reload && systemctl restart bluetooth")

            # Try to restart bluetooth - maybe the override is already there
            logger.info("Trying to restart bluetooth.service...")
            self.run_command("systemctl restart bluetooth")
            time.sleep(3)

            # Check again
            success, stdout, stderr = self.run_command("sdptool browse local")
            if not success or "Failed to connect" in stderr:
                logger.warning("SDP still not accessible. SPP registration will fail.")
                logger.warning("Connection may still work if client initiates correctly.")
                return False

        # Now try to register SPP
        success, stdout, stderr = self.run_command(f"sdptool add --channel={BT_CHANNEL} SP")
        if success:
            logger.info("SPP service registered successfully on channel " + str(BT_CHANNEL))
        else:
            logger.warning(f"SPP registration failed: {stderr}")
            logger.warning("Continuing anyway - socket server may still accept connections")

        return True

    def start_server(self):
        """Start Bluetooth RFCOMM socket server"""
        logger.info(f"Starting Bluetooth socket server on channel {BT_CHANNEL}...")

        try:
            # Import bluetooth module
            try:
                import bluetooth
            except ImportError:
                logger.error("PyBluez not installed. Install with: pip install pybluez")
                logger.info("Falling back to raw socket approach...")
                return self._start_raw_server()

            # Create server socket
            self.server_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            self.server_socket.bind(("", BT_CHANNEL))
            self.server_socket.listen(1)

            logger.info(f"Bluetooth server listening on channel {BT_CHANNEL}")
            logger.info(f"Device name: Arduino_BT_Bridge")
            logger.info(f"Address: {self.bt_address}")
            logger.info("Ready for connections!")

            # Accept connections in a loop
            while self.running:
                try:
                    self.server_socket.settimeout(1.0)  # Allow checking self.running
                    client_socket, client_info = self.server_socket.accept()

                    logger.info(f"New connection from: {client_info}")

                    # Start client handler thread
                    handler = ClientHandler(client_socket, client_info, self.arduino)
                    handler.start()
                    self.clients.append(handler)

                    # Clean up finished handlers
                    self.clients = [c for c in self.clients if c.is_alive()]

                except bluetooth.BluetoothError as e:
                    if "timed out" not in str(e).lower():
                        logger.error(f"Server accept error: {e}")
                except Exception as e:
                    if self.running:
                        logger.error(f"Server error: {e}")

            return True

        except Exception as e:
            logger.error(f"Failed to start Bluetooth server: {e}")
            return False

    def _start_raw_server(self):
        """Fallback: Start server using rfcomm listen instead of rfcomm watch"""
        import select

        logger.info("Using rfcomm listen fallback (no PyBluez)...")

        rfcomm_dev = "/dev/rfcomm0"

        # Make sure rfcomm device doesn't exist
        if Path(rfcomm_dev).exists():
            logger.info(f"Releasing existing {rfcomm_dev}...")
            self.run_command(f"rfcomm release {rfcomm_dev}")
            time.sleep(1)

        # Re-register agent after bluetoothd restart (it may have been unregistered)
        self._restart_agent()

        logger.info(f"Listening for connections on channel {BT_CHANNEL}...")
        logger.info(f"Device: Arduino_BT_Bridge ({self.bt_address})")
        logger.info("Waiting for client to connect...")

        while self.running:
            try:
                # Use rfcomm listen to wait for a connection
                # This blocks until a client connects
                logger.info("Starting rfcomm listen...")

                # Start rfcomm listen in background
                listen_cmd = f"rfcomm listen {rfcomm_dev} {BT_CHANNEL}"
                logger.info(f"Command: {listen_cmd}")

                listen_process = subprocess.Popen(
                    listen_cmd.split(),
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True
                )

                # Wait for connection (check process output)
                connected = False
                client_addr = None

                while self.running and not connected:
                    # Check if process has output
                    ready, _, _ = select.select([listen_process.stdout], [], [], 1.0)

                    if ready:
                        line = listen_process.stdout.readline()
                        if line:
                            line = line.strip()
                            logger.info(f"rfcomm: {line}")

                            # Check for connection
                            if "Connection from" in line:
                                # Extract client address
                                try:
                                    # Format: "Connection from XX:XX:XX:XX:XX:XX to /dev/rfcomm0"
                                    parts = line.split()
                                    for i, p in enumerate(parts):
                                        if p == "from" and i + 1 < len(parts):
                                            client_addr = parts[i + 1]
                                            break
                                except Exception:
                                    client_addr = "unknown"

                                connected = True
                                logger.info(f"=== CLIENT CONNECTED: {client_addr} ===")

                    # Check if rfcomm device appeared (connection established)
                    if not connected and Path(rfcomm_dev).exists():
                        logger.info(f"rfcomm device {rfcomm_dev} appeared, connection established")
                        connected = True
                        client_addr = "unknown"

                    # Check if process died (only if not connected)
                    if not connected and listen_process.poll() is not None:
                        logger.warning("rfcomm listen exited unexpectedly")
                        break

                if connected and Path(rfcomm_dev).exists():
                    # Handle the connection
                    self._handle_rfcomm_connection(rfcomm_dev, client_addr)

                    # Connection ended, release and loop back
                    logger.info("Connection ended, releasing rfcomm...")
                    self.run_command(f"rfcomm release {rfcomm_dev}")
                    time.sleep(1)

                else:
                    # No connection or process died
                    try:
                        listen_process.terminate()
                        listen_process.wait(timeout=2)
                    except Exception:
                        try:
                            listen_process.kill()
                        except Exception:
                            pass

                    # Small delay before retrying
                    time.sleep(2)

            except Exception as e:
                logger.error(f"Error in listen loop: {e}")
                time.sleep(2)

        return True

    def _restart_agent(self):
        """Restart the Bluetooth agent after bluetoothd restart"""
        logger.info("Re-registering Bluetooth agent...")

        # Kill old agent if exists
        if self.agent_process:
            try:
                self.agent_process.terminate()
                self.agent_process.wait(timeout=2)
            except Exception:
                try:
                    self.agent_process.kill()
                except Exception:
                    pass
            self.agent_process = None

        # Start new agent
        self.start_bt_agent()

        # Make sure discoverable and pairable are still set
        self.run_command("hciconfig hci0 piscan")

        # Trust all paired devices and ensure discoverable
        self._trust_all_paired_devices()

    def _trust_all_paired_devices(self):
        """Trust all paired devices to allow automatic reconnection"""
        logger.info("Trusting all paired devices...")

        # Get list of paired devices
        success, stdout, _ = self.run_command("bluetoothctl paired-devices")
        if not success or not stdout:
            return

        # Build commands to trust each device and set discoverable
        bt_commands = "discoverable on\npairable on\n"

        for line in stdout.strip().split('\n'):
            if line.startswith("Device "):
                parts = line.split()
                if len(parts) >= 2:
                    mac = parts[1]
                    bt_commands += f"trust {mac}\n"
                    logger.info(f"Trusting device: {mac}")

        bt_commands += "quit\n"

        try:
            proc = subprocess.Popen(
                ['bluetoothctl'],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            proc.communicate(input=bt_commands, timeout=10)
        except Exception as e:
            logger.debug(f"Error trusting devices: {e}")

    def _handle_rfcomm_connection(self, rfcomm_dev, client_addr):
        """Handle an established RFCOMM connection"""
        logger.info(f"Handling connection on {rfcomm_dev} from {client_addr}")

        # Log to command log
        try:
            from datetime import datetime
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            with open(CMD_LOG, 'a') as f:
                f.write(f"[{timestamp}] [CONNECT] Client: {client_addr}\n")
        except Exception:
            pass

        bt_buffer = ""
        arduino_buffer = ""
        last_cmd = ""

        try:
            # Open rfcomm device for read/write
            # Use raw mode to prevent echo
            self.run_command(f"stty -F {rfcomm_dev} raw -echo")

            fd = os.open(rfcomm_dev, os.O_RDWR | os.O_NONBLOCK)
            logger.info(f"Opened {rfcomm_dev} for communication")

            # Send welcome message
            try:
                os.write(fd, b"OK:CONNECTED\n")
                logger.info(">>> BT TX: [OK:CONNECTED]")
            except Exception as e:
                logger.warning(f"Could not send welcome: {e}")

            import select

            while self.running:
                # Check for data from BT client
                try:
                    ready, _, _ = select.select([fd], [], [], 0.05)
                    if ready:
                        data = os.read(fd, 1024)

                        if not data:
                            logger.info("Client disconnected (EOF)")
                            break

                        bt_buffer += data.decode('utf-8', errors='ignore')

                        # Process complete lines
                        while '\n' in bt_buffer or '\r' in bt_buffer:
                            if '\n' in bt_buffer:
                                line, bt_buffer = bt_buffer.split('\n', 1)
                            else:
                                line, bt_buffer = bt_buffer.split('\r', 1)

                            cmd = line.strip()
                            if cmd:
                                logger.info(f"<<< BT RX: [{cmd}]")
                                self._log_cmd(f"[BT_RX] {cmd}")
                                last_cmd = cmd

                                # Forward to Arduino
                                if self.arduino.send(f"{cmd}\n".encode()):
                                    logger.info(f">>> ARDUINO TX: [{cmd}]")
                                    self._log_cmd(f"[ARDUINO_TX] {cmd}")

                except OSError as e:
                    if e.errno not in (11, 35):  # EAGAIN, EWOULDBLOCK
                        logger.error(f"BT read error: {e}")
                        break
                except Exception as e:
                    logger.error(f"BT error: {e}")
                    break

                # Check for data from Arduino
                arduino_data = self.arduino.read()
                if arduino_data:
                    arduino_buffer += arduino_data.decode('utf-8', errors='ignore')

                    while '\n' in arduino_buffer:
                        line, arduino_buffer = arduino_buffer.split('\n', 1)
                        line = line.strip()

                        if line:
                            # Filter echo
                            if line == last_cmd:
                                logger.info(f"<<< ARDUINO RX (echo filtered): [{line}]")
                                last_cmd = ""
                            else:
                                logger.info(f"<<< ARDUINO RX: [{line}]")
                                self._log_cmd(f"[ARDUINO_RX] {line}")

                                # Forward to BT client
                                try:
                                    os.write(fd, f"{line}\n".encode())
                                    logger.info(f">>> BT TX: [{line}]")
                                    self._log_cmd(f"[BT_TX] {line}")
                                except Exception as e:
                                    logger.error(f"BT write error: {e}")
                                    break

        except Exception as e:
            logger.error(f"Connection handler error: {e}")

        finally:
            logger.info(f"=== CLIENT DISCONNECTED: {client_addr} ===")
            self._log_cmd(f"[DISCONNECT] Client: {client_addr}")
            try:
                os.close(fd)
            except Exception:
                pass

    def _log_cmd(self, msg):
        """Log to command log file"""
        try:
            from datetime import datetime
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            with open(CMD_LOG, 'a') as f:
                f.write(f"[{timestamp}] {msg}\n")
        except Exception:
            pass

    def cleanup(self):
        """Cleanup on exit"""
        logger.info("Cleaning up...")

        # Stop all client handlers
        for client in self.clients:
            client.stop()

        # Close server socket
        if self.server_socket:
            try:
                self.server_socket.close()
            except Exception:
                pass

        # Stop D-Bus agent loop if running
        if hasattr(self, '_dbus_loop') and self._dbus_loop:
            try:
                self._dbus_loop.quit()
            except Exception:
                pass

        # Stop agent process
        if self.agent_process:
            try:
                self.agent_process.stdin.write("quit\n")
                self.agent_process.stdin.flush()
                self.agent_process.terminate()
                self.agent_process.wait(timeout=2)
            except Exception:
                try:
                    self.agent_process.kill()
                except Exception:
                    pass

        # Close Arduino connection
        self.arduino.close()

        # Release rfcomm device
        self.run_command("rfcomm release /dev/rfcomm0")

        # Remove PID file
        try:
            Path(PID_FILE).unlink(missing_ok=True)
        except Exception as e:
            logger.debug(f"Could not remove PID file: {e}")

        logger.info("Cleanup complete")

    def run(self):
        """Main service loop"""
        logger.info("=" * 70)
        logger.info("Bluetooth-Arduino Bridge Service Starting")
        logger.info(f"Debug log: {DEBUG_LOG}")
        logger.info(f"Command log: {CMD_LOG}")
        logger.info(f"Service log: {SERVICE_LOG}")
        logger.info("=" * 70)

        # Write PID file
        try:
            with open(PID_FILE, 'w') as f:
                f.write(str(os.getpid()))
        except Exception as e:
            logger.warning(f"Could not write PID file: {e}")

        # Setup signal handlers
        self.setup_signal_handlers()

        # Setup Bluetooth
        if not self.setup_bluetooth():
            logger.error("Bluetooth setup failed")
            return 1

        # Connect to Arduino (non-fatal if not found)
        self.arduino.connect()

        # Register SPP service
        if not self.register_spp_service():
            logger.warning("SPP service registration failed, continuing anyway")

        # Start server
        if not self.start_server():
            logger.error("Failed to start Bluetooth server")
            return 1

        # Cleanup on exit
        self.cleanup()
        return 0


def main():
    """Main entry point"""
    bridge = BluetoothBridge()
    return bridge.run()


if __name__ == "__main__":
    sys.exit(main())
