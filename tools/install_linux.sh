#!/usr/bin/env bash
# Bootstrap the COLMAG project on a fresh Linux PC.
#
# Usage:
#   bash install_linux.sh
#   bash install_linux.sh --system-deps   # also install apt packages

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
VENV_DIR="${VENV_DIR:-$ROOT_DIR/venv}"

install_system_deps=false
if [[ "${1:-}" == "--system-deps" ]]; then
    install_system_deps=true
fi

echo "=== COLMAG Linux Setup ==="
echo "Project: $ROOT_DIR"
echo "Python:  $PYTHON_BIN"
echo "Venv:    $VENV_DIR"
echo ""

if $install_system_deps; then
    if ! command -v apt-get >/dev/null 2>&1; then
        echo "ERROR: --system-deps currently supports apt-based Linux only." >&2
        exit 1
    fi
    echo "[1/3] Installing system dependencies..."
    sudo apt-get update
    sudo apt-get install -y \
        build-essential \
        git \
        libgl1 \
        libglib2.0-0 \
        libsm6 \
        libxext6 \
        libxrender1 \
        python3 \
        python3-dev \
        python3-pip \
        python3-tk \
        python3-venv
else
    echo "[1/3] Skipping apt packages. Re-run with --system-deps if this is a fresh machine."
fi

echo "[2/3] Creating virtual environment..."
if [[ ! -d "$VENV_DIR" ]]; then
    "$PYTHON_BIN" -m venv "$VENV_DIR"
fi

echo "[3/3] Installing Python dependencies..."
"$VENV_DIR/bin/python" -m pip install --upgrade pip setuptools wheel
"$VENV_DIR/bin/python" -m pip install -r "$ROOT_DIR/requirements.txt"

echo ""
echo "=== Setup complete ==="
echo "Activate:"
echo "  source \"$VENV_DIR/bin/activate\""
echo ""
echo "Smoke tests:"
echo "  python -m py_compile magnetometer_reader.py colmag/interaction.py"
echo "  python test_virtual_joystick.py"
echo "  python test_writing_filter.py"
echo ""
echo "Try the UI without hardware:"
echo "  python magnetometer_reader.py --input-source touchpad --no-csv --classifier-labels ABCXLRUD0123"
