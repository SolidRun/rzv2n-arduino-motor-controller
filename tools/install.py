#!/usr/bin/env python3
"""
Cross-platform installer for Robot Controller GUI
Works on Windows, Linux, and macOS with any Python 3.8+ environment.

Usage:
    python install.py           # normal install
    python install.py --clean   # remove venv and reinstall
    python install.py --check   # verify existing install only
"""

import sys
import os
import subprocess
import shutil
import platform
import argparse
from pathlib import Path

# ── Minimum Python version ────────────────────────────────────────────────────
MIN_PYTHON = (3, 8)

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR   = Path(__file__).resolve().parent
VENV_DIR     = SCRIPT_DIR / "venv"
REQUIREMENTS = SCRIPT_DIR / "requirements.txt"
IS_WINDOWS   = platform.system() == "Windows"

# venv bin directory differs per OS
VENV_BIN    = VENV_DIR / ("Scripts" if IS_WINDOWS else "bin")
VENV_PYTHON = VENV_BIN / ("python.exe" if IS_WINDOWS else "python")
VENV_PIP    = VENV_BIN / ("pip.exe"    if IS_WINDOWS else "pip")

# ── ANSI colours (disabled automatically on Windows < 10 / no-tty) ────────────
def _supports_color() -> bool:
    if not hasattr(sys.stdout, "isatty") or not sys.stdout.isatty():
        return False
    if IS_WINDOWS:
        # Enable VT100 on Windows 10+
        try:
            import ctypes
            kernel32 = ctypes.windll.kernel32
            kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), 7)
            return True
        except Exception:
            return False
    return True

USE_COLOR = _supports_color()

class C:
    RESET  = "\033[0m"  if USE_COLOR else ""
    BOLD   = "\033[1m"  if USE_COLOR else ""
    RED    = "\033[91m" if USE_COLOR else ""
    GREEN  = "\033[92m" if USE_COLOR else ""
    YELLOW = "\033[93m" if USE_COLOR else ""
    CYAN   = "\033[96m" if USE_COLOR else ""

def ok(msg: str):   print(f"{C.GREEN}[OK]  {msg}{C.RESET}")
def warn(msg: str): print(f"{C.YELLOW}[!!]  {msg}{C.RESET}")
def err(msg: str):  print(f"{C.RED}[ERR] {msg}{C.RESET}")
def step(msg: str): print(f"\n{C.CYAN}{C.BOLD}{msg}{C.RESET}")
def header(title: str):
    line = "=" * 54
    print(f"\n{C.CYAN}{line}")
    print(f"    {title}")
    print(f"{line}{C.RESET}\n")

# ── Subprocess helper ─────────────────────────────────────────────────────────
def run(cmd: list, check: bool = True, capture: bool = False) -> subprocess.CompletedProcess:
    return subprocess.run(
        cmd,
        check=check,
        capture_output=capture,
        text=True,
    )

# ── Step implementations ──────────────────────────────────────────────────────

def check_python_version():
    """Ensure the interpreter running this script is new enough."""
    step("Step 1: Checking Python version...")
    v = sys.version_info
    ver_str = f"{v.major}.{v.minor}.{v.micro}"
    if v < MIN_PYTHON:
        err(f"Python {ver_str} is too old. Need {MIN_PYTHON[0]}.{MIN_PYTHON[1]}+")
        print(f"Download a newer Python from: https://www.python.org/downloads/")
        sys.exit(1)
    ok(f"Python {ver_str}  ({sys.executable})")


def check_tkinter():
    """tkinter must exist in the host Python (it ships with official installers)."""
    step("Step 2: Checking tkinter...")
    try:
        import tkinter  # noqa: F401
        ok("tkinter is available")
    except ModuleNotFoundError:
        warn("tkinter not found in this Python installation.")
        if IS_WINDOWS:
            warn("Re-install Python from python.org and tick 'tcl/tk and IDLE'.")
        else:
            warn("Install it with your package manager, e.g.:")
            warn("  sudo apt install python3-tk   # Debian / Ubuntu")
            warn("  sudo dnf install python3-tkinter  # Fedora / RHEL")
        # Non-fatal — GUI may still work via system Python
        warn("Continuing anyway...")


def remove_venv():
    if VENV_DIR.exists():
        warn(f"Removing existing venv at {VENV_DIR} ...")
        shutil.rmtree(VENV_DIR)
        ok("Old venv removed")


def create_venv():
    step("Step 3: Creating virtual environment...")
    try:
        run([sys.executable, "-m", "venv", str(VENV_DIR)])
    except subprocess.CalledProcessError:
        err("Failed to create virtual environment.")
        if not IS_WINDOWS:
            warn("Try: sudo apt install python3-venv")
        sys.exit(1)
    ok(f"Virtual environment created at {VENV_DIR}")


def upgrade_pip():
    step("Step 4: Upgrading pip...")
    try:
        run([str(VENV_PYTHON), "-m", "pip", "install", "--upgrade", "pip", "--quiet"])
        ok("pip upgraded")
    except subprocess.CalledProcessError:
        warn("pip upgrade failed — continuing with existing version")


def install_packages():
    step("Step 5: Installing Python packages...")
    if REQUIREMENTS.exists():
        print(f"  Installing from {REQUIREMENTS.name} ...")
        try:
            run([str(VENV_PIP), "install", "-r", str(REQUIREMENTS), "--quiet"])
            ok(f"All packages from {REQUIREMENTS.name} installed")
            return
        except subprocess.CalledProcessError:
            warn("requirements.txt install failed — falling back to individual packages")

    # Fallback: install known packages directly
    print("  Installing pyserial...")
    run([str(VENV_PIP), "install", "pyserial>=3.5", "--quiet"])
    ok("pyserial installed")


def verify_install():
    step("Step 6: Verifying installation...")

    # pyserial
    result = run([str(VENV_PYTHON), "-c",
                  "import serial; print(serial.VERSION)"], capture=True, check=False)
    if result.returncode == 0:
        ok(f"pyserial {result.stdout.strip()}")
    else:
        err("pyserial import failed inside venv.")
        sys.exit(1)

    # tkinter (checked against host Python — venv inherits it on Windows,
    # on Linux a missing system package is the usual cause)
    result = run([sys.executable, "-c", "import tkinter; print('ok')"],
                 capture=True, check=False)
    if result.returncode == 0:
        ok("tkinter (host Python)")
    else:
        warn("tkinter not available — GUI may not launch")


def print_summary():
    header("Setup Complete!")
    print("Run the GUI with:\n")
    if IS_WINDOWS:
        print(f"  {C.GREEN}venv\\Scripts\\python.exe robot_test.py{C.RESET}")
    else:
        print(f"  {C.GREEN}venv/bin/python robot_test.py{C.RESET}")
    print()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Robot Controller GUI installer")
    parser.add_argument("--clean", action="store_true",
                        help="Remove existing venv and reinstall from scratch")
    parser.add_argument("--check", action="store_true",
                        help="Only verify an existing installation, do not install")
    args = parser.parse_args()

    header("Robot Controller GUI - Setup")
    print(f"Platform  : {platform.system()} {platform.release()} ({platform.machine()})")
    print(f"Python    : {sys.version.split()[0]}  ({sys.executable})")
    print(f"Directory : {SCRIPT_DIR}")

    if args.check:
        verify_install()
        return

    check_python_version()
    check_tkinter()

    if args.clean:
        remove_venv()
    elif VENV_DIR.exists():
        warn(f"Existing venv found at {VENV_DIR}")
        warn("Run with --clean to rebuild from scratch, or continuing with upgrade...")

    if not VENV_DIR.exists():
        create_venv()

    upgrade_pip()
    install_packages()
    verify_install()
    print_summary()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInstallation cancelled by user.")
        sys.exit(1)
