#!/usr/bin/env bash
# Create the RobotMK1 Python virtual environment on the Raspberry Pi (Raspberry Pi OS).
#
#   ./setup_venv.sh               create .venv and install everything (run again to update)
#   source .venv/bin/activate     use it in a terminal; then:  python main.py
#
# The venv is created with --system-site-packages, so Python packages installed with apt
# stay visible inside it, e.g. the Hailo-8 bindings from "sudo apt install hailo-all".
set -euo pipefail
cd "$(dirname "$0")"

echo "== system packages (asks for your password)"
sudo apt-get update
sudo apt-get install -y python3-venv python3-dev python3-pip libgl1 libglib2.0-0 libusb-1.0-0

echo "== virtual environment in .venv"
if [ ! -d .venv ]; then
    python3 -m venv --system-site-packages .venv
fi
.venv/bin/python -m pip install --upgrade pip wheel
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m pip install -e .        # "import modules" works everywhere in the venv

echo "== check"
.venv/bin/python -c "import serial, yaml, depthai, modules.engines, modules.camera; print('ok, depthai', depthai.__version__)"

echo
echo "Done. In a new terminal:   cd $(pwd) && source .venv/bin/activate && python main.py"
