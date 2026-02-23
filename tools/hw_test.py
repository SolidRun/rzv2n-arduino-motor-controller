#!/usr/bin/env python3
"""
Hardware Test CLI - Interactive motor & encoder diagnostics.

Connects to Arduino running HW Test firmware and provides
a friendly interface for testing individual motors, pins,
and reading encoders.

Usage:
    python3 tools/hw_test.py [PORT]
    python3 tools/hw_test.py /dev/ttyACM0
    python3 tools/hw_test.py          # auto-detect

Build & upload firmware first:
    pio run -e hwtest -t upload
"""

import sys
import time
import serial
import serial.tools.list_ports
import threading

BAUD = 115200
MOTOR_NAMES = {0: "FL", 1: "RL", 2: "RR", 3: "FR"}

# ── ANSI Colors ──────────────────────────────────────────────────────────────

class C:
    RESET  = "\033[0m"
    BOLD   = "\033[1m"
    DIM    = "\033[2m"
    RED    = "\033[31m"
    GREEN  = "\033[32m"
    YELLOW = "\033[33m"
    BLUE   = "\033[34m"
    CYAN   = "\033[36m"

def ok(msg):    print(f"{C.GREEN}[OK]{C.RESET} {msg}")
def err(msg):   print(f"{C.RED}[ERR]{C.RESET} {msg}")
def info(msg):  print(f"{C.CYAN}[..]{C.RESET} {msg}")
def warn(msg):  print(f"{C.YELLOW}[!!]{C.RESET} {msg}")

# ── Serial Communication ────────────────────────────────────────────────────

class Arduino:
    def __init__(self, port, baud=BAUD):
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # Wait for Arduino reset
        # Flush startup messages
        while self.ser.in_waiting:
            print(f"{C.DIM}{self.ser.readline().decode(errors='replace').rstrip()}{C.RESET}")

    def send(self, cmd):
        self.ser.write(f"{cmd}\n".encode())
        self.ser.flush()
        time.sleep(0.05)
        lines = []
        while self.ser.in_waiting:
            line = self.ser.readline().decode(errors='replace').rstrip()
            if line and line != "> ":
                lines.append(line)
        return lines

    def send_print(self, cmd):
        lines = self.send(cmd)
        for line in lines:
            line = line.replace("> ", "")
            print(f"  {line}")
        return lines

    def close(self):
        self.ser.close()


def auto_detect_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if "Arduino" in (p.description or "") or "ttyACM" in p.device or "ttyUSB" in p.device:
            return p.device
    if ports:
        return ports[0].device
    return None

# ── Interactive Menu ─────────────────────────────────────────────────────────

HELP_TEXT = f"""
{C.BOLD}═══ Hardware Test CLI ═══{C.RESET}

  {C.YELLOW}Motor Commands:{C.RESET}
    m <idx> <speed>     Run motor (0=FL 1=RL 2=RR 3=FR), speed -255..255
    m <idx> 0           Stop single motor
    stop                Coast all motors
    brake               Brake all motors

  {C.YELLOW}Raw Pin Commands:{C.RESET}
    pin <ch> <duty>     Set PCA9685 channel 0-15, duty 0-4095
    pin <ch> on         Set channel full ON (4095)
    pin <ch> off        Set channel OFF (0)

  {C.YELLOW}Encoder Commands:{C.RESET}
    enc                 Read all encoders once
    encr                Reset all encoders
    live                Toggle live encoder stream (10Hz)

  {C.YELLOW}Diagnostics:{C.RESET}
    map                 Show hardware pin mapping
    i2c                 Scan I2C bus
    sweep <idx>         Sweep motor speed -255 to +255 (direction test)

  {C.YELLOW}Quick Tests:{C.RESET}
    test all            Spin each motor briefly forward then reverse
    test enc            Reset encoders, spin all forward, print counts

  {C.CYAN}help{C.RESET}  Show this menu    {C.CYAN}quit{C.RESET}  Exit
"""


def cmd_sweep(ard, args):
    """Sweep a motor from -255 to +255 to test direction."""
    if not args:
        err("Usage: sweep <motor_idx>")
        return
    idx = int(args[0])
    if idx < 0 or idx > 3:
        err("Motor index must be 0-3")
        return

    info(f"Sweeping motor {idx} ({MOTOR_NAMES[idx]}) -255 → +255")
    info("Watch motor direction. Press Ctrl+C to abort.")
    try:
        for speed in range(-255, 256, 15):
            ard.send(f"MOT,{idx},{speed}")
            label = "REV" if speed < 0 else ("FWD" if speed > 0 else "STOP")
            bar_len = abs(speed) // 8
            bar = "█" * bar_len
            direction = "◀" if speed < 0 else ("▶" if speed > 0 else "●")
            print(f"\r  {direction} {speed:+4d} [{bar:<32}]  ", end="", flush=True)
            time.sleep(0.04)
        print()
        ard.send("STOP")
        ok("Sweep done, motors coasting")
    except KeyboardInterrupt:
        print()
        ard.send("STOP")
        warn("Aborted, motors stopped")


def cmd_test_all(ard):
    """Briefly spin each motor forward then reverse."""
    info("Testing all motors: forward then reverse")
    for idx in range(4):
        name = MOTOR_NAMES[idx]
        print(f"  Motor {idx} ({name}): ", end="", flush=True)

        print("FWD...", end="", flush=True)
        ard.send(f"MOT,{idx},150")
        time.sleep(0.5)
        ard.send(f"MOT,{idx},0")
        time.sleep(0.3)

        print("REV...", end="", flush=True)
        ard.send(f"MOT,{idx},-150")
        time.sleep(0.5)
        ard.send(f"MOT,{idx},0")
        time.sleep(0.3)

        print(f"{C.GREEN}OK{C.RESET}")

    ard.send("STOP")
    ok("All motors tested")


def cmd_test_enc(ard):
    """Reset encoders, spin all motors, then read counts."""
    ard.send("ENCR")
    info("Encoders reset. Running all motors forward for 1s...")
    ard.send("MOT,0,150")
    ard.send("MOT,1,150")
    ard.send("MOT,2,150")
    ard.send("MOT,3,150")
    time.sleep(1.0)
    ard.send("STOP")
    time.sleep(0.2)
    info("Encoder readings after 1s at PWM=150:")
    ard.send_print("ENC")
    ok("All encoders should show positive, non-zero values")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else auto_detect_port()
    if not port:
        err("No serial port found. Specify port: python3 hw_test.py /dev/ttyACM0")
        sys.exit(1)

    info(f"Connecting to {port} @ {BAUD}...")
    try:
        ard = Arduino(port)
    except serial.SerialException as e:
        err(f"Cannot open {port}: {e}")
        sys.exit(1)
    ok(f"Connected to {port}")

    # Show startup info
    ard.send_print("MAP")
    print(HELP_TEXT)

    try:
        while True:
            try:
                raw = input(f"{C.BOLD}hw>{C.RESET} ").strip()
            except EOFError:
                break
            if not raw:
                continue

            parts = raw.split()
            cmd = parts[0].lower()
            args = parts[1:]

            if cmd in ("quit", "exit", "q"):
                ard.send("STOP")
                break
            elif cmd == "help":
                print(HELP_TEXT)
            elif cmd == "m" or cmd == "mot":
                if len(args) < 2:
                    err("Usage: m <idx> <speed>")
                else:
                    ard.send_print(f"MOT,{args[0]},{args[1]}")
            elif cmd == "pin":
                if len(args) < 2:
                    err("Usage: pin <ch> <duty|on|off>")
                else:
                    duty = args[1]
                    if duty.lower() == "on":
                        duty = "4095"
                    elif duty.lower() == "off":
                        duty = "0"
                    ard.send_print(f"PIN,{args[0]},{duty}")
            elif cmd == "enc":
                ard.send_print("ENC")
            elif cmd == "encr":
                ard.send_print("ENCR")
            elif cmd == "live":
                ard.send_print("LIVE")
                # Read live data until user presses enter
                info("Press Enter to stop live mode")
                ard.ser.timeout = 0.15
                try:
                    import select
                    while True:
                        if ard.ser.in_waiting:
                            line = ard.ser.readline().decode(errors='replace').rstrip()
                            if line and line != "> ":
                                print(f"\r  {line}          ", end="", flush=True)
                        # Check if user pressed enter (non-blocking stdin)
                        if select.select([sys.stdin], [], [], 0)[0]:
                            sys.stdin.readline()
                            break
                except (ImportError, OSError):
                    # Fallback: just wait for enter
                    input()
                ard.send("LIVE")  # Toggle off
                ard.ser.timeout = 1
                print()
            elif cmd == "stop":
                ard.send_print("STOP")
            elif cmd == "brake":
                ard.send_print("BRK")
            elif cmd == "map":
                ard.send_print("MAP")
            elif cmd == "i2c":
                ard.send_print("I2C")
            elif cmd == "sweep":
                cmd_sweep(ard, args)
            elif cmd == "test":
                sub = args[0].lower() if args else ""
                if sub == "all":
                    cmd_test_all(ard)
                elif sub == "enc":
                    cmd_test_enc(ard)
                else:
                    err("Usage: test all | test enc")
            elif cmd == "raw":
                # Pass-through raw command to Arduino
                raw_cmd = " ".join(args) if args else ""
                ard.send_print(raw_cmd)
            else:
                # Try sending as raw command
                ard.send_print(raw.upper())

    except KeyboardInterrupt:
        print()
        ard.send("STOP")
    finally:
        ard.close()
        ok("Disconnected. Motors stopped.")


if __name__ == "__main__":
    main()
