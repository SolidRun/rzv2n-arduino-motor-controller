#!/usr/bin/env python3
"""
Cross-platform launcher for Robot Controller GUI
Works on both Windows and Linux
"""

import os
import sys
import platform
import subprocess
from pathlib import Path


def main():
    """Launch the Robot Controller GUI"""
    script_dir = Path(__file__).parent
    venv_dir = script_dir / "venv"

    # Determine the Python executable path based on OS
    system = platform.system()
    if system == "Windows":
        python_exe = venv_dir / "Scripts" / "python.exe"
    else:
        python_exe = venv_dir / "bin" / "python"

    # Check if virtual environment exists
    if not venv_dir.exists() or not python_exe.exists():
        print("Error: Virtual environment not found!")
        print(f"Please run the setup script first:")
        print(f"  python{3 if system != 'Windows' else ''} setup.py")
        sys.exit(1)

    # Check if dependencies are installed
    result = subprocess.run(
        [str(python_exe), "-c", "import serial"],
        capture_output=True
    )

    if result.returncode != 0:
        print("Error: Dependencies not installed!")
        print(f"Please run the setup script first:")
        print(f"  python{3 if system != 'Windows' else ''} setup.py")
        sys.exit(1)

    # Launch the GUI
    gui_script = script_dir / "robot_test.py"
    print("Starting Robot Controller GUI...")
    print(f"Python: {python_exe}")
    print(f"Script: {gui_script}")
    print()

    try:
        subprocess.run([str(python_exe), str(gui_script)])
    except KeyboardInterrupt:
        print("\nGUI closed by user")
    except Exception as e:
        print(f"Error launching GUI: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
