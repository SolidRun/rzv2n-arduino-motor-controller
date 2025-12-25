#!/usr/bin/env python3
"""
Test the Python handler script without Bluetooth
Tests serial communication directly
"""

import sys
import serial
import time

ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUD = 9600

def test_serial_connection():
    """Test direct serial connection to Arduino"""
    print(f"Testing serial connection to {ARDUINO_PORT}")

    try:
        # Open serial with proper settings (matching handle_connection.py)
        arduino = serial.Serial(
            port=ARDUINO_PORT,
            baudrate=ARDUINO_BAUD,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.5,
            write_timeout=1.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False
        )

        print(f"✓ Serial port opened successfully")
        print(f"  Port: {arduino.port}")
        print(f"  Baudrate: {arduino.baudrate}")
        print(f"  Settings: {arduino.bytesize}N{arduino.stopbits}")

        # Flush buffers
        arduino.reset_input_buffer()
        arduino.reset_output_buffer()
        print("✓ Buffers flushed")

        # Wait for Arduino to reset
        print("  Waiting 2 seconds for Arduino reset...")
        time.sleep(2)

        # Flush again after reset
        arduino.reset_input_buffer()
        arduino.reset_output_buffer()
        print("✓ Arduino ready")

        # Test commands
        test_commands = [
            "READ",
            "FWD,100,500",
            "STOP"
        ]

        for cmd in test_commands:
            print(f"\nSending: {cmd}")

            # Send with newline
            message = f"{cmd}\n".encode('utf-8')
            arduino.write(message)
            arduino.flush()
            print(f"  ✓ Sent {len(message)} bytes")

            # Wait for response
            time.sleep(0.2)

            if arduino.in_waiting > 0:
                response = arduino.readline().decode('utf-8', errors='ignore').strip()
                print(f"  ← Response: {response}")
            else:
                print(f"  (no response)")

        # Close
        arduino.close()
        print("\n✓ Test completed successfully")
        return True

    except serial.SerialException as e:
        print(f"✗ Serial error: {e}")
        return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


if __name__ == "__main__":
    success = test_serial_connection()
    sys.exit(0 if success else 1)
