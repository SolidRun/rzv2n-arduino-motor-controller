#!/usr/bin/env python3
"""
Bluetooth to Arduino Bridge - Connection Handler
Bidirectional relay: BT <-> Arduino

- Continuously reads from Arduino and forwards to BT
- Receives commands from BT and forwards to Arduino
- Uses select() to handle both directions without blocking
"""

import sys
import os
import serial
import time
import select
import threading
import subprocess
from datetime import datetime

# Configuration
ARDUINO_BAUD = 9600
ARDUINO_PORT = "/dev/ttyACM0"
DEBUG_LOG = "/tmp/bt_bridge_debug.log"
CMD_LOG = "/tmp/bt_bridge_commands.log"
SERVICE_LOG = "/tmp/bt_arduino_bridge.log"


class Logger:
    """Thread-safe logger"""
    _lock = threading.Lock()

    @staticmethod
    def _timestamp():
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    @staticmethod
    def _timestamp_ms():
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    @staticmethod
    def debug(msg):
        with Logger._lock:
            with open(DEBUG_LOG, 'a') as f:
                f.write(f"[{Logger._timestamp()}] DEBUG: {msg}\n")

    @staticmethod
    def info(msg):
        with Logger._lock:
            line = f"[{Logger._timestamp()}] INFO: {msg}\n"
            with open(SERVICE_LOG, 'a') as f:
                f.write(line)
            print(line, end='', flush=True)

    @staticmethod
    def cmd(msg):
        with Logger._lock:
            with open(CMD_LOG, 'a') as f:
                f.write(f"[{Logger._timestamp_ms()}] {msg}\n")

    @staticmethod
    def error(msg):
        with Logger._lock:
            line = f"[{Logger._timestamp()}] ERROR: {msg}\n"
            with open(SERVICE_LOG, 'a') as f:
                f.write(line)
            print(line, end='', file=sys.stderr, flush=True)


class ArduinoBridge:
    """Bidirectional bridge between Bluetooth and Arduino"""

    def __init__(self, rfcomm_device, arduino_port=ARDUINO_PORT, baudrate=ARDUINO_BAUD):
        self.rfcomm_device = rfcomm_device
        self.arduino_port = arduino_port
        self.baudrate = baudrate
        self.arduino = None
        self.bt_fd_read = None
        self.bt_fd_write = None
        self.running = False

    def setup_arduino(self):
        """Initialize Arduino serial connection"""
        Logger.info(f"Initializing Arduino at {self.arduino_port}")

        try:
            self.arduino = serial.Serial(
                port=self.arduino_port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,  # Non-blocking reads
                write_timeout=1.0,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )

            self.arduino.reset_input_buffer()
            self.arduino.reset_output_buffer()

            Logger.info(f"Arduino configured: {self.arduino_port} @ {self.baudrate}")

            # Wait for Arduino reset
            time.sleep(2)
            self.arduino.reset_input_buffer()

            Logger.info("Arduino ready")
            return True

        except Exception as e:
            Logger.error(f"Failed to open Arduino: {e}")
            return False

    def open_rfcomm(self):
        """Open RFCOMM device for read and write"""
        try:
            # Disable echo on RFCOMM device to prevent command reflection
            subprocess.run(['stty', '-F', self.rfcomm_device, '-echo', 'raw'], check=False)

            # Open for read/write with non-blocking
            self.bt_fd_read = os.open(self.rfcomm_device, os.O_RDWR | os.O_NONBLOCK)
            self.bt_fd_write = self.bt_fd_read  # Use same fd for both
            Logger.info(f"Opened RFCOMM for read/write: {self.rfcomm_device}")
            return True
        except Exception as e:
            Logger.error(f"Failed to open RFCOMM: {e}")
            return False

    def send_to_bluetooth(self, msg):
        """Send message to Bluetooth client"""
        try:
            data = f"{msg}\n".encode('utf-8')
            os.write(self.bt_fd_write, data)
            time.sleep(0.02)  # Delay to ensure separate BT transmission
            Logger.info(f">>> BT TX: [{msg}]")
            Logger.cmd(f"[BT_TX] {msg}")
            return True
        except Exception as e:
            Logger.error(f"Error sending to BT: {e}")
            return False

    def send_to_arduino(self, cmd):
        """Send command to Arduino"""
        if not self.arduino or not self.arduino.is_open:
            return False

        try:
            message = f"{cmd}\n".encode('utf-8')
            self.arduino.write(message)
            self.arduino.flush()
            Logger.info(f">>> ARDUINO TX: [{cmd}]")
            Logger.cmd(f"[ARDUINO_TX] {cmd}")
            return True
        except Exception as e:
            Logger.error(f"Error sending to Arduino: {e}")
            return False

    def run(self):
        """Main bidirectional relay loop"""
        Logger.info(f"=== NEW CONNECTION on {self.rfcomm_device} ===")

        if not self.setup_arduino():
            return 1

        if not self.open_rfcomm():
            if self.arduino:
                self.arduino.close()
            return 1

        Logger.info("Starting bidirectional relay...")
        self.running = True

        bt_buffer = ""
        arduino_buffer = ""
        last_cmd = ""  # Track last command sent to filter echo

        try:
            while self.running:
                # Check both BT and Arduino for data
                # Arduino uses pyserial (has fileno), BT uses file descriptor
                arduino_fd = self.arduino.fileno()

                ready_read, _, _ = select.select(
                    [self.bt_fd_read, arduino_fd], [], [], 0.05
                )

                # === Read from Bluetooth, send to Arduino ===
                if self.bt_fd_read in ready_read:
                    try:
                        data = os.read(self.bt_fd_read, 1024).decode('utf-8', errors='ignore')

                        if not data:
                            Logger.info("BT client disconnected (EOF)")
                            break

                        bt_buffer += data

                        # Process complete lines
                        while '\n' in bt_buffer or '\r' in bt_buffer:
                            if '\n' in bt_buffer:
                                line, bt_buffer = bt_buffer.split('\n', 1)
                            else:
                                line, bt_buffer = bt_buffer.split('\r', 1)

                            cmd = line.strip()
                            if cmd:
                                Logger.info(f"<<< BT RX: [{cmd}]")
                                Logger.cmd(f"[BT_RX] {cmd}")
                                last_cmd = cmd  # Store to filter echo
                                self.send_to_arduino(cmd)

                    except OSError as e:
                        if e.errno != 11:  # Not EAGAIN
                            Logger.error(f"BT read error: {e}")
                            break

                # === Read from Arduino, send to Bluetooth ===
                if arduino_fd in ready_read:
                    try:
                        # Read available data from Arduino
                        if self.arduino.in_waiting > 0:
                            data = self.arduino.read(self.arduino.in_waiting).decode('utf-8', errors='ignore')

                            if data:
                                arduino_buffer += data

                                # Process complete lines
                                while '\n' in arduino_buffer:
                                    line, arduino_buffer = arduino_buffer.split('\n', 1)
                                    line = line.strip()

                                    if line:
                                        # Filter out echoed command
                                        if line == last_cmd:
                                            Logger.info(f"<<< ARDUINO RX (echo filtered): [{line}]")
                                            last_cmd = ""  # Clear after filtering
                                        else:
                                            Logger.info(f"<<< ARDUINO RX: [{line}]")
                                            Logger.cmd(f"[ARDUINO_RX] {line}")
                                            self.send_to_bluetooth(line)

                    except Exception as e:
                        Logger.error(f"Arduino read error: {e}")

        except KeyboardInterrupt:
            Logger.info("Interrupted")

        finally:
            self.running = False
            Logger.info("Connection closed")

            if self.bt_fd_read is not None:
                try:
                    os.close(self.bt_fd_read)
                except:
                    pass

            if self.arduino and self.arduino.is_open:
                try:
                    self.arduino.close()
                except:
                    pass

        return 0


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <rfcomm_device>", file=sys.stderr)
        return 1

    bridge = ArduinoBridge(sys.argv[1])
    return bridge.run()


if __name__ == "__main__":
    sys.exit(main())
