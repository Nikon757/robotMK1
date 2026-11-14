#!/usr/bin/env bash
# RobotMK1 mapping - one-shot installer for Raspberry Pi OS (64-bit, Trixie or Bookworm).
#
#   ./install.sh                 python venv + apt libs + udev rules            (fusion mapper + tools)
#   ./install.sh --docker        ... and install Docker (for the RTAB-Map stack)
#   ./install.sh --build-image   ... and build the RTAB-Map Docker image (20-40 min)
#   ./install.sh --hailo         ... and install the Hailo-8 runtime (hailo-all)
#   ./install.sh --all           everything
#
# Re-running is safe.  Log out / in (or reboot) afterwards so the new group memberships apply.
set -euo pipefail
cd "$(dirname "$0")"

WITH_DOCKER=0; BUILD_IMAGE=0; WITH_HAILO=0
for a in "$@"; do
  case "$a" in
    --docker) WITH_DOCKER=1 ;;
    --build-image) WITH_DOCKER=1; BUILD_IMAGE=1 ;;
    --hailo) WITH_HAILO=1 ;;
    --all) WITH_DOCKER=1; BUILD_IMAGE=1; WITH_HAILO=1 ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) echo "unknown option: $a"; exit 2 ;;
  esac
done

echo "== [1/6] apt packages"
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  python3 python3-venv python3-dev python3-pip \
  libusb-1.0-0 udev usbutils git curl ca-certificates \
  libgl1 libglib2.0-0 libxcb-xinerama0 libxcb-cursor0 libxkbcommon-x11-0 libfontconfig1 libdbus-1-3 \
  x11-xserver-utils

echo "== [2/6] udev rules (LiDAR -> /dev/robot_lidar, STM32 -> /dev/robot_mcu, OAK permissions)"
sudo install -m 644 config/99-robot-usb.rules /etc/udev/rules.d/99-robot-usb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout,plugdev,video "$USER" || true

echo "== [3/6] python virtual environment (.venv, --system-site-packages)"
PY=python3
$PY --version
if [ ! -d .venv ]; then
  $PY -m venv --system-site-packages .venv
fi
.venv/bin/pip install --upgrade pip wheel
.venv/bin/pip install -r requirements.txt

echo "== [4/6] verifying python modules"
.venv/bin/python - <<'PYEOF'
import importlib, sys
ok = True
for m in ("numpy", "scipy", "cv2", "depthai", "serial", "yaml", "matplotlib"):
    try:
        mod = importlib.import_module(m)
        print(f"  {m:12s} {getattr(mod, '__version__', '')}")
    except Exception as e:
        ok = False
        print(f"  {m:12s} MISSING ({e})")
try:
    import hailo_platform  # noqa
    print("  hailo_platform OK (Hailo runtime present)")
except Exception:
    print("  hailo_platform not importable (install with ./install.sh --hailo; only needed for --hailo)")
sys.exit(0 if ok else 1)
PYEOF

if [ "$WITH_HAILO" = "1" ]; then
  echo "== [5/6] Hailo-8 runtime"
  sudo apt-get install -y hailo-all
  echo "   -> reboot once, then check:  hailortcli fw-control identify"
  echo "   -> put a Hailo-8 (not 8L) YOLOv8 .hef into models/ (see DEPENDENCIES.md)"
else
  echo "== [5/6] Hailo skipped (use --hailo)"
fi

if [ "$WITH_DOCKER" = "1" ]; then
  echo "== [6/6] Docker"
  if ! command -v docker >/dev/null 2>&1; then
    curl -fsSL https://get.docker.com | sudo sh
  fi
  sudo usermod -aG docker "$USER" || true
  sudo systemctl enable --now docker
  if [ "$BUILD_IMAGE" = "1" ]; then
    echo "   building robotmk1/rtabmap:jazzy (20-40 min on a Pi 5) ..."
    sudo docker build -t robotmk1/rtabmap:jazzy mappers/rtabmap
  fi
else
  echo "== [6/6] Docker skipped (use --docker / --build-image for the RTAB-Map stack)"
fi

mkdir -p maps models
echo
echo "Done.  Log out and back in (group changes), plug in the sensors, then:"
echo "   source .venv/bin/activate"
echo "   python run.py check"
