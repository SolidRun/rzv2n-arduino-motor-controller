#!/usr/bin/env python3
"""
Bluetooth to Arduino Bridge Service (Python Version)
Main service that manages Bluetooth setup and spawns connection handlers
"""

import os
import sys
import subprocess
import time
import signal
import logging
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
HANDLER_SCRIPT = SCRIPT_DIR / "handle_connection.py"

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


class BluetoothBridge:
    """Main Bluetooth bridge service"""

    def __init__(self):
        self.running = True
        self.rfcomm_process = None
        self.bt_address = None

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

    def setup_bluetooth(self):
        """Initialize and configure Bluetooth adapter"""
        logger.info("Setting up Bluetooth...")

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
                    self.bt_address = line.split()[-1]
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

        # Configure with bluetoothctl
        logger.info("Configuring with bluetoothctl...")
        bt_commands = """system-alias Arduino_BT_Bridge
power on
discoverable on
pairable on
agent on
exit
"""
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
            logger.warning(f"bluetoothctl configuration warning: {e}")

        # Fallback: Make discoverable using hciconfig
        logger.info("Setting discoverable and pairable...")
        self.run_command("hciconfig hci0 piscan")

        logger.info("Bluetooth setup complete")
        return True

    def find_arduino(self):
        """Find Arduino port"""
        logger.info("Searching for Arduino...")

        # Method 1: Check /dev/serial/by-id
        by_id_path = Path("/dev/serial/by-id")
        if by_id_path.exists():
            for port in by_id_path.glob("*Arduino*"):
                arduino_port = port.resolve()
                logger.info(f"Found Arduino via by-id: {arduino_port}")
                return str(arduino_port)

        # Method 2: Check common ports
        for port in ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"]:
            if Path(port).exists():
                logger.info(f"Found potential Arduino: {port}")
                return port

        logger.info("Arduino not found (will auto-detect when needed)")
        return None

    def register_spp_service(self):
        """Register Serial Port Profile service"""
        logger.info("Registering Serial Port Profile service...")
        success, stdout, stderr = self.run_command(f"sdptool add --channel={BT_CHANNEL} SP")
        if success:
            logger.info("SPP service registered successfully")
        else:
            logger.warning(f"SPP registration warning: {stderr}")
        return True

    def start_rfcomm_watch(self):
        """Start rfcomm in watch mode to handle incoming connections"""
        rfcomm_dev = "/dev/rfcomm0"

        # Make sure rfcomm device doesn't exist
        if Path(rfcomm_dev).exists():
            logger.info(f"Releasing existing {rfcomm_dev}...")
            self.run_command(f"rfcomm release {rfcomm_dev}")
            time.sleep(1)

        logger.info(f"Starting rfcomm watch on channel {BT_CHANNEL}...")
        logger.info(f"Handler script: {HANDLER_SCRIPT}")

        if not HANDLER_SCRIPT.exists():
            logger.error(f"Handler script not found: {HANDLER_SCRIPT}")
            return False

        # rfcomm watch will call our handler script when a client connects
        # Format: rfcomm watch <dev> <channel> <command> [args]
        cmd = [
            "rfcomm",
            "watch",
            rfcomm_dev,
            str(BT_CHANNEL),
            str(HANDLER_SCRIPT),
            "{}"  # {} will be replaced with the rfcomm device path
        ]

        logger.info(f"Command: {' '.join(cmd)}")

        try:
            # Start rfcomm watch as a subprocess
            self.rfcomm_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True
            )

            logger.info("rfcomm watch started successfully")
            logger.info(f"Bluetooth device 'Arduino_BT_Bridge' is ready for connections")
            logger.info(f"Listening on {self.bt_address}:{BT_CHANNEL}")

            # Monitor the rfcomm watch process
            while self.running:
                try:
                    # Check if process is still running
                    returncode = self.rfcomm_process.poll()
                    if returncode is not None:
                        logger.error(f"rfcomm watch exited with code {returncode}")
                        return False

                    # Read and log output
                    if self.rfcomm_process.stdout:
                        line = self.rfcomm_process.stdout.readline()
                        if line:
                            logger.info(f"rfcomm: {line.strip()}")

                    time.sleep(0.1)

                except Exception as e:
                    logger.error(f"Error monitoring rfcomm: {e}")
                    break

            return True

        except Exception as e:
            logger.error(f"Failed to start rfcomm watch: {e}")
            return False

    def cleanup(self):
        """Cleanup on exit"""
        logger.info("Cleaning up...")

        # Stop rfcomm process
        if self.rfcomm_process:
            try:
                self.rfcomm_process.terminate()
                self.rfcomm_process.wait(timeout=5)
            except Exception as e:
                logger.error(f"Error terminating rfcomm process: {e}")
                try:
                    self.rfcomm_process.kill()
                except:
                    pass

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
        logger.info("Bluetooth-Arduino Bridge Service Starting (Python Version)")
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

        # Find Arduino (non-fatal if not found)
        self.find_arduino()

        # Register SPP service
        if not self.register_spp_service():
            logger.error("SPP service registration failed")
            return 1

        # Start rfcomm watch
        if not self.start_rfcomm_watch():
            logger.error("Failed to start rfcomm watch")
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
