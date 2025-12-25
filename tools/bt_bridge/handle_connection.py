#!/usr/bin/env python3
"""
Bluetooth to Arduino Bridge - Connection Handler
Called by rfcomm watch when a Bluetooth client connects
"""

import sys
import os
import serial
import time
import select
from datetime import datetime

# Configuration
BT_CHANNEL = 1
ARDUINO_BAUD = 9600
ARDUINO_PORT = "/dev/ttyACM0"
DEBUG_LOG = "/tmp/bt_bridge_debug.log"
CMD_LOG = "/tmp/bt_bridge_commands.log"
SERVICE_LOG = "/tmp/bt_arduino_bridge.log"

class Logger:
    """Simple logger for the bridge"""

    @staticmethod
    def _timestamp():
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    @staticmethod
    def _timestamp_ms():
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    @staticmethod
    def debug(msg):
        with open(DEBUG_LOG, 'a') as f:
            f.write(f"[{Logger._timestamp()}] DEBUG: {msg}\n")

    @staticmethod
    def info(msg):
        line = f"[{Logger._timestamp()}] INFO: {msg}\n"
        with open(SERVICE_LOG, 'a') as f:
            f.write(line)
        print(line, end='', flush=True)

    @staticmethod
    def cmd(msg):
        with open(CMD_LOG, 'a') as f:
            f.write(f"[{Logger._timestamp_ms()}] {msg}\n")

    @staticmethod
    def error(msg):
        line = f"[{Logger._timestamp()}] ERROR: {msg}\n"
        with open(SERVICE_LOG, 'a') as f:
            f.write(line)
        with open(DEBUG_LOG, 'a') as f:
            f.write(line)
        print(line, end='', file=sys.stderr, flush=True)


class ArduinoBridge:
    """Bridges Bluetooth RFCOMM connection to Arduino serial"""

    def __init__(self, rfcomm_device, arduino_port=ARDUINO_PORT, baudrate=ARDUINO_BAUD):
        self.rfcomm_device = rfcomm_device
        self.arduino_port = arduino_port
        self.baudrate = baudrate
        self.arduino = None
        self.rfcomm_fd = None

    def setup_arduino(self):
        """Initialize Arduino serial connection"""
        Logger.info(f"Initializing Arduino at {self.arduino_port}")

        try:
            # Open serial connection with proper settings
            self.arduino = serial.Serial(
                port=self.arduino_port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,  # 100ms timeout for reads
                write_timeout=1.0,  # 1s timeout for writes
                xonxoff=False,  # No software flow control
                rtscts=False,   # No hardware flow control
                dsrdtr=False    # No DSR/DTR flow control
            )

            # Flush any existing data
            self.arduino.reset_input_buffer()
            self.arduino.reset_output_buffer()

            Logger.info(f"Arduino configured: {self.arduino_port} @ {self.baudrate} baud, 8N1, raw mode")

            # Wait for Arduino to reset (common with Arduino auto-reset on serial connection)
            time.sleep(2)

            # Flush again after reset
            self.arduino.reset_input_buffer()
            self.arduino.reset_output_buffer()

            Logger.info("Arduino ready")
            return True

        except serial.SerialException as e:
            Logger.error(f"Failed to open Arduino serial: {e}")
            return False
        except Exception as e:
            Logger.error(f"Unexpected error setting up Arduino: {e}")
            return False

    def open_rfcomm(self):
        """Open the RFCOMM device for reading"""
        try:
            self.rfcomm_fd = os.open(self.rfcomm_device, os.O_RDONLY | os.O_NONBLOCK)
            Logger.info(f"Opened RFCOMM device: {self.rfcomm_device}")
            return True
        except Exception as e:
            Logger.error(f"Failed to open RFCOMM device: {e}")
            return False

    def send_to_arduino(self, cmd):
        """Send command to Arduino with newline termination"""
        if not self.arduino or not self.arduino.is_open:
            Logger.error("Arduino not connected")
            return None

        try:
            # Send command with newline
            message = f"{cmd}\n".encode('utf-8')
            self.arduino.write(message)
            self.arduino.flush()  # Ensure data is sent immediately

            Logger.info(f">>> ARDUINO TX: [{cmd}]")
            Logger.cmd(f"[ARDUINO_TX] {cmd} | bytes={len(message)}")

            # Try to read response (with timeout)
            time.sleep(0.1)  # Give Arduino time to process

            if self.arduino.in_waiting > 0:
                try:
                    response = self.arduino.readline().decode('utf-8', errors='ignore').strip()
                    if response:
                        Logger.info(f"<<< ARDUINO RX: [{response}]")
                        Logger.cmd(f"[ARDUINO_RX] {response}")
                        return response
                except Exception as e:
                    Logger.debug(f"Error reading Arduino response: {e}")

            # No response or empty response
            return "OK"

        except serial.SerialTimeoutException:
            Logger.error("Timeout writing to Arduino")
            return "ERROR"
        except Exception as e:
            Logger.error(f"Error sending to Arduino: {e}")
            return "ERROR"

    def send_to_bluetooth(self, msg):
        """Send response back to Bluetooth client"""
        try:
            # Open for writing (we opened read-only for the main loop)
            with open(self.rfcomm_device, 'w') as f:
                f.write(f"{msg}\n")
                f.flush()
            Logger.cmd(f"[BT_TX] {msg}")
            return True
        except Exception as e:
            Logger.error(f"Error sending to Bluetooth: {e}")
            return False

    def run(self):
        """Main relay loop"""
        Logger.info(f"=== NEW CONNECTION on {self.rfcomm_device} ===")
        Logger.cmd(f"[BT_CONNECT] Client connected to {self.rfcomm_device}")

        # Setup Arduino
        if not self.setup_arduino():
            Logger.error("Failed to setup Arduino, exiting")
            return 1

        # Open RFCOMM device
        if not self.open_rfcomm():
            Logger.error("Failed to open RFCOMM device, exiting")
            if self.arduino:
                self.arduino.close()
            return 1

        Logger.info("Starting command relay loop...")

        # Buffer for incoming Bluetooth data
        bt_buffer = ""

        try:
            while True:
                # Use select to check if data is available (non-blocking with timeout)
                ready, _, _ = select.select([self.rfcomm_fd], [], [], 0.1)

                if ready:
                    # Read available data from Bluetooth
                    try:
                        data = os.read(self.rfcomm_fd, 1024).decode('utf-8', errors='ignore')

                        if not data:
                            # EOF - connection closed
                            Logger.info("Bluetooth client disconnected (EOF)")
                            break

                        # Add to buffer
                        bt_buffer += data

                        # Process complete lines
                        while '\n' in bt_buffer or '\r' in bt_buffer:
                            # Split on newline or carriage return
                            if '\n' in bt_buffer:
                                line, bt_buffer = bt_buffer.split('\n', 1)
                            else:
                                line, bt_buffer = bt_buffer.split('\r', 1)

                            # Clean up the command
                            cmd = line.strip()

                            if not cmd:
                                Logger.debug("Empty line, skipping")
                                continue

                            # Log raw input
                            Logger.debug(f"BT raw: {cmd.encode('utf-8').hex()}")
                            Logger.info(f">>> BT RX: [{cmd}]")
                            Logger.cmd(f"[BT_RX] {cmd}")

                            # Send to Arduino and get response
                            response = self.send_to_arduino(cmd)

                            if response:
                                self.send_to_bluetooth(response)
                            else:
                                self.send_to_bluetooth("ERROR: Arduino not connected")

                    except OSError as e:
                        if e.errno == 11:  # EAGAIN - no data available
                            continue
                        else:
                            Logger.error(f"Error reading from Bluetooth: {e}")
                            break
                    except Exception as e:
                        Logger.error(f"Unexpected error in read loop: {e}")
                        break

        except KeyboardInterrupt:
            Logger.info("Interrupted by user")

        finally:
            # Cleanup
            Logger.info(f"Connection closed on {self.rfcomm_device}")
            Logger.cmd("[BT_DISCONNECT] Connection closed")

            if self.rfcomm_fd is not None:
                try:
                    os.close(self.rfcomm_fd)
                except:
                    pass

            if self.arduino and self.arduino.is_open:
                try:
                    self.arduino.close()
                except:
                    pass

        return 0


def main():
    """Main entry point"""
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <rfcomm_device>", file=sys.stderr)
        return 1

    rfcomm_device = sys.argv[1]

    # Create and run bridge
    bridge = ArduinoBridge(rfcomm_device)
    return bridge.run()


if __name__ == "__main__":
    sys.exit(main())
