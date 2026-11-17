# Installing RobotMK1 on the Raspberry Pi

Step-by-step setup of the Raspberry Pi 5 running **Raspberry Pi OS (64-bit)**, written for
Trixie (Python 3.13); Bookworm works the same way. At the end you can run the scripts from the
console menu, open Jupyter Lab from your laptop, and run ROS 2 in Docker.

The control-board firmware is flashed from the Windows PC (Keil + ST-Link), not from the Pi:
see [firmware/README.md](../firmware/README.md) §3.

Contents

1. [Base system](#1-base-system)
2. [Get the project](#2-get-the-project)
3. [USB device names and permissions](#3-usb-device-names-and-permissions)
4. [Python virtual environment](#4-python-virtual-environment)
5. [First run: the scripts](#5-first-run-the-scripts)
6. [OAK-D Lite camera](#6-oak-d-lite-camera)
7. [Hailo-8 accelerator](#7-hailo-8-accelerator)
8. [Jupyter Lab](#8-jupyter-lab)
9. [ROS 2 in Docker](#9-ros-2-in-docker)
10. [Troubleshooting](#10-troubleshooting)

---

## 1. Base system

1. **Flash the OS.** Use Raspberry Pi Imager to write **Raspberry Pi OS (64-bit)** to the SD
   card or SSD. In the Imager's settings, set:
   - hostname `RobotMK1`;
   - your user name and password;
   - Wi-Fi;
   - **enable SSH**.
2. **Log in.** Boot the Pi and log in from the laptop:

   ```bash
   ssh ivan@RobotMK1.local
   ```

3. **Update everything and install Git:**

   ```bash
   sudo apt update && sudo apt full-upgrade -y && sudo apt install -y git
   ```

4. **Check the USB power limit.** On a Pi 5 that is **not** powered by the official 5 A supply
   (for example, from the robot's power board), the USB ports are limited to 600 mA in total.
   That is too little for the camera and the LiDAR together. Only if your power board really
   delivers 5 A at 5 V, add this line to `/boot/firmware/config.txt` and reboot:

   ```
   usb_max_current_enable=1
   ```

---

## 2. Get the project

```bash
cd ~ && git clone https://github.com/Nikon757/robotMK1.git
```

- **Private repository:** log in to GitHub first (`gh auth login`, or an SSH key).
- **Without GitHub:** copy the folder from the PC instead:

  ```bash
  scp -r "D:\downloads\A robot docs\robotMK1" ivan@RobotMK1.local:~/
  ```

- **Updating later:**

  ```bash
  cd ~/robotMK1 && git pull
  ```

---

## 3. USB device names and permissions

The control board (CH340) and the LiDAR (CP2102) both appear as `/dev/ttyUSB*`, and their
numbers swap when they reconnect. The rules in `config/99-robot-usb.rules` give them fixed
names, `/dev/robot_mcu` and `/dev/robot_lidar`, and also let you use the OAK-D without root:

```bash
cd ~/robotMK1 && sudo cp config/99-robot-usb.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger
```

```bash
sudo usermod -aG dialout,plugdev,video $USER
```

Log out and back in, or reboot, then check:

```bash
ls -l /dev/robot_mcu /dev/robot_lidar
```

If a name is missing, `lsusb` should list `1a86:7523` (the board) and `10c4:ea60` (the LiDAR).
If a `/dev/ttyUSB*` port appears and then disappears, remove the screen-reader package that
grabs it:

```bash
sudo apt remove brltty
```

---

## 4. Python virtual environment

```bash
cd ~/robotMK1 && bash setup_venv.sh
```

It takes a few minutes the first time. What it does:
- **System packages:** installs the ones the Python packages need.
- **Virtual environment:** creates `.venv` with `--system-site-packages`, so packages
  installed with apt, such as the Hailo bindings, stay visible inside it.
- **Python packages:** installs `requirements.txt` (pyserial, PyYAML, numpy, depthai v2,
  OpenCV, Jupyter Lab).
- **The project's own code:** installs `modules/`, so `import modules` works from any folder,
  including notebooks.
- **Check:** ends by testing that everything imports.

**In every new terminal:**

```bash
cd ~/robotMK1 && source .venv/bin/activate
```

The prompt then starts with `(.venv)`. Run `bash setup_venv.sh` again after `requirements.txt`
changes.

---

## 5. First run: the scripts

```bash
python main.py
```

The menu lists every script in `scripts/`. Type its number, optionally followed by options
(`3 --distance 1.0`). `h 3` shows script 3's options. Each script can also run directly, e.g.
`python scripts/03_forward.py --distance 0.3`.

**Recommended order the first time:**

1. **`5`: camera pan.** The wheels don't move, so it's a safe check that the board responds.
2. **`0`: encoder calibration.** Wheels lifted; you turn each rear wheel 5 times by hand. It
   writes `counts_per_metre` into `config/robot.yaml`; scripts 1–4 refuse to run without it.
3. **`3`: forward 0.5 m.** Check the distance with a tape. For better accuracy, then run
   `0 --drive`.
4. **`1` (45° left) and `2` (turn around).** If a script stops with "turned the wrong way",
   swap `steering.left` and `steering.right` in `config/robot.yaml`.
5. **`4`: drive and stream.** Open the printed address in a browser.

**Safety:** keep the battery on and the robot on the floor with free space around it.
- **Ctrl+C** stops the motors at once.
- **The board** stops them within 0.5 s if a program dies.
- **The scripts** stop by themselves if the wheels stall or the board restarts.

**All robot settings** are in `config/robot.yaml`: geometry, encoder calibration, steering and
camera limits, motor power.

---

## 6. OAK-D Lite camera

- **Port:** plug it into a **blue (USB 3)** port. `lsusb` lists it as `03e7:...`; the ID
  changes after the camera boots, which is normal.
- **Python library:** `depthai` (v2 API) is installed by `setup_venv.sh`. With the venv
  active, this lists the camera:

  ```bash
  python -c "import depthai as dai; print(dai.Device.getAllAvailableDevices())"
  ```

- **Live picture:** script 4 streams it to the browser. The JPEG compression runs on the
  camera, so the Pi's CPU stays free.
- **One program at a time:** only one program can use the camera at once. Close a notebook
  that holds it before running script 4.

---

## 7. Hailo-8 accelerator

1. **Fit the module.** Use the Hailo-8 M.2 module with the Raspberry Pi M.2 HAT+ (or the AI
   HAT+), and check the ribbon cable.
2. **Enable PCIe Gen 3.** Add this to `/boot/firmware/config.txt` and reboot:

   ```
   dtparam=pciex1_gen=3
   ```

3. **Install the driver, runtime and tools:**

   ```bash
   sudo apt install -y hailo-all
   ```

   Check the Raspberry Pi documentation that `hailo-all` is available for your OS release.
4. **Reboot and check:**

   ```bash
   hailortcli fw-control identify
   ```

   It should report a Hailo-8 (26 TOPS). Models must be compiled for **Hailo-8**, not
   Hailo-8L.

The Python bindings (`hailo_platform`) come from apt. The venv sees them because it was created
with `--system-site-packages`. The `modules/npu/` code will use them.

---

## 8. Jupyter Lab

Jupyter Lab runs on the Pi, and you use it from the laptop's browser.

1. **Set a password once:**

   ```bash
   source ~/robotMK1/.venv/bin/activate && jupyter lab password
   ```

2. **Start it:**

   ```bash
   cd ~/robotMK1 && source .venv/bin/activate && jupyter lab --no-browser --ip=0.0.0.0 --port=8888
   ```

3. **Open it on the laptop:** `http://RobotMK1.local:8888` (or the Pi's IP address).

In a notebook, the project's modules work directly:

```python
from modules.engines import Robot
from modules.camera import CameraPan

with Robot() as robot:              # stops the motors when the block ends
    robot.forward(0.3)
    robot.turn_by(45, steer=0.7)
    print(robot.pose)
```

**Notes:**
- **One user per device:** a notebook that keeps a `Robot`, a board connection or the camera
  open blocks the scripts from using them. Close it (`robot.close()`) or restart the kernel
  first.
- **Where to keep notebooks:** a `notebooks/` folder is a good place for them.
- **Running in the background:** to keep Jupyter running after you close the SSH session,
  start it inside `tmux`, or set it up as a systemd service.

---

## 9. ROS 2 in Docker

ROS 2 isn't packaged for Raspberry Pi OS, so it runs in a Docker container, using the official
ROS 2 Jazzy images. The alternative would be Ubuntu 24.04 on the Pi, which runs ROS 2 natively,
but Raspberry Pi's Hailo packages target Raspberry Pi OS.

1. **Install Docker** (the official convenience script):

   ```bash
   curl -fsSL https://get.docker.com -o get-docker.sh && sudo sh get-docker.sh
   ```

   ```bash
   sudo usermod -aG docker $USER
   ```

   Log out and back in, then check:

   ```bash
   docker run --rm hello-world
   ```

2. **First ROS 2 test:**

   ```bash
   docker run -it --rm --net=host ros:jazzy-ros-base ros2 topic list
   ```

   It should print `/parameter_events` and `/rosout`.

3. **Access to the robot's devices.** Pass the serial devices and the USB bus into the
   container:

   ```bash
   docker run -it --rm --net=host --device /dev/robot_mcu --device /dev/robot_lidar -v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw' ros:jazzy-ros-base bash
   ```

   `189` is the USB device class, which the OAK-D needs, because it reconnects with a new ID
   after it boots.

The project's own ROS 2 image (Nav2, slam_toolbox, RTAB-Map, the LiDAR and camera drivers) and
its launch files will live in `ros2/`. The earlier Dockerfile is in the Git history
(`robot_mapping/mappers/rtabmap/Dockerfile`, commit a9deefa). The plan is in
[ROADMAP.md](ROADMAP.md).

---

## 10. Troubleshooting

| Symptom | Check |
|---|---|
| `Can't find the control board` | Is the board's Type-C cable in? `lsusb` should show `1a86:7523`. Are the udev rules installed (§3)? |
| `No data from the board` | Battery switched on? Run `python pi_examples/hw_test.py` for a diagnosis |
| `Permission denied` on a port | Did you log out and in after `usermod`? Are the udev rules installed? |
| `counts_per_metre is not set` | Run script 0 first |
| "turned the wrong way" | Swap `steering.left` and `steering.right` in `config/robot.yaml` |
| "wheels aren't turning" | Battery on? Robot blocked? Raise `drive.power` (e.g. 450) |
| Script 4: no picture | Camera on a blue USB 3 port? Another program using it? USB power limit (§1, step 4)? |
| Browser can't open the stream | Same Wi-Fi network? Try the IP address instead of `RobotMK1.local` |
| `ModuleNotFoundError` | Is the venv active (`source .venv/bin/activate`)? Run `bash setup_venv.sh` again |
| `depthai` fails to install | Python version: `python3 --version`; the project expects 3.11–3.13 |
