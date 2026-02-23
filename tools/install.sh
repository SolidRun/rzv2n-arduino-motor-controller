#!/bin/bash
# Complete setup script for Robot Controller GUI
# Installs system dependencies, creates venv, and installs Python packages

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}======================================================${NC}"
echo -e "${BLUE}    Robot Controller GUI - Complete Setup${NC}"
echo -e "${BLUE}======================================================${NC}\n"

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    echo -e "${RED}ERROR: Do not run this script as root (with sudo)${NC}"
    echo "Run as regular user: ./install.sh"
    echo "The script will ask for sudo password when needed."
    exit 1
fi

# Step 1: Install system dependencies
echo -e "${BLUE}Step 1: Installing system dependencies...${NC}"
echo "This will install: build-essential python3-dev python3-tk python3-venv"
echo ""

# Check what needs to be installed
MISSING_PACKAGES=()

dpkg -s build-essential &>/dev/null || MISSING_PACKAGES+=("build-essential")
dpkg -s python3-dev &>/dev/null || MISSING_PACKAGES+=("python3-dev")
dpkg -s python3-tk &>/dev/null || MISSING_PACKAGES+=("python3-tk")
dpkg -s python3-venv &>/dev/null || MISSING_PACKAGES+=("python3-venv")

if [ ${#MISSING_PACKAGES[@]} -eq 0 ]; then
    echo -e "${GREEN}✓ All system dependencies already installed${NC}\n"
else
    echo "Missing packages: ${MISSING_PACKAGES[@]}"
    echo "Installing with sudo (you may be asked for your password)..."

    # Try to update package list, but continue even if it fails
    echo "Updating package list..."
    if ! sudo apt update 2>&1; then
        echo -e "${YELLOW}⚠ Warning: apt update failed, but continuing...${NC}"
        echo -e "${YELLOW}  This might be due to corrupted sources.list files${NC}"
        echo -e "${YELLOW}  Attempting to install packages anyway...${NC}"
    fi

    # Try to install packages
    if sudo apt install -y "${MISSING_PACKAGES[@]}" 2>&1; then
        echo -e "${GREEN}✓ System dependencies installed${NC}\n"
    else
        echo -e "${YELLOW}⚠ Some system packages failed to install${NC}"
        echo -e "${YELLOW}  Continuing anyway - Bluetooth may not work${NC}\n"
    fi
fi

# Step 2: Remove old virtual environment if it exists
if [ -d "venv" ]; then
    echo -e "${YELLOW}Removing old virtual environment...${NC}"
    rm -rf venv
    echo -e "${GREEN}✓ Old venv removed${NC}\n"
fi

# Step 3: Create virtual environment
echo -e "${BLUE}Step 2: Creating virtual environment...${NC}"
python3 -m venv venv
echo -e "${GREEN}✓ Virtual environment created${NC}\n"

# Step 4: Upgrade pip
echo -e "${BLUE}Step 3: Upgrading pip...${NC}"
./venv/bin/pip install --upgrade pip -q
echo -e "${GREEN}✓ pip upgraded${NC}\n"

# Step 5: Install Python packages
echo -e "${BLUE}Step 4: Installing Python packages...${NC}"

# Install pyserial (always works)
echo "  - Installing pyserial..."
./venv/bin/pip install "pyserial>=3.5" -q
echo -e "${GREEN}    ✓ pyserial installed${NC}"

echo ""

# Step 6: Verify installation
echo -e "${BLUE}Step 5: Verifying installation...${NC}"

# Check pyserial
if ./venv/bin/python -c "import serial" 2>/dev/null; then
    VERSION=$(./venv/bin/python -c "import serial; print(serial.VERSION)")
    echo -e "${GREEN}✓ pyserial $VERSION${NC}"
else
    echo -e "${RED}✗ pyserial not working${NC}"
    exit 1
fi

# Check tkinter
if python3 -c "import tkinter" 2>/dev/null; then
    echo -e "${GREEN}✓ tkinter${NC}"
else
    echo -e "${RED}✗ tkinter not working${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}======================================================${NC}"
echo -e "${GREEN}    Setup Complete!${NC}"
echo -e "${GREEN}======================================================${NC}\n"

echo "To run the Robot Controller GUI:"
echo ""
echo -e "  ${GREEN}python3 run.py${NC}"
echo ""

echo ""
