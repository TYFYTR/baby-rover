# Baby Rover

A ROS 2 prototype differential drive rover running on Raspberry Pi 5.

**Two purposes:**
1. Personal learning platform for ROS 2, sensor integration, and autonomous navigation
2. Architectural prototype for the Knuckey Rover university project (RMIT)

Nodes developed here transfer directly to the university rover — same sensors, same topic names, same node structure.

---

## Hardware

| Component | Spec | Notes |
|-----------|------|-------|
| Compute | Raspberry Pi 5 8GB | Ubuntu 24.04 server (aarch64) |
| Storage | Crucial BX500 1TB SSD | USB 3.0 into Pi blue port |
| Motors | 2x N20 with encoders (WS-26377) | Differential drive |
| Motor controller | TB6612FNG (POLOLU-713) | Driven via Pi GPIO |
| IMU | MPU-6050 (018-MPU-6050) | I2C, address 0x68 |
| Depth camera | Intel RealSense D415 | USB 3.0 (blue port) |

Full wiring diagrams and pin assignments: [`docs/hardware.md`](docs/hardware.md)

---

## System Architecture

```
Sensors → ROS 2 Topics → EKF → Nav2 → /cmd_vel → PID → Motors → Encoders → back to EKF
                                              ↑
                                       State Machine
                                  (watching everything,
                                   ready to interrupt)
```

Every component communicates through ROS 2 topics. Nothing talks directly to anything else.

### Nodes

| Node | Job | Subscribes | Publishes |
|------|-----|------------|-----------|
| `motor_controller` | Drives TB6612, reads encoders | `/cmd_vel` | `/odom` |
| `imu_driver` | Reads MPU-6050 | — | `/imu/data` |
| `realsense2_camera` | RealSense D415 wrapper | — | `/camera/camera/depth/image_rect_raw`, `/camera/camera/color/image_raw` |
| `teleop` | Keyboard input | — | `/cmd_vel` |
| `state_machine` | System state, fault handling | `/cmd_vel`, `/odom`, `/imu/data` | `/system_state` |

### Key Topics

| Topic | Message Type | Direction |
|-------|-------------|-----------|
| `/cmd_vel` | geometry_msgs/Twist | Commands in |
| `/odom` | nav_msgs/Odometry | Wheel odometry out |
| `/imu/data` | sensor_msgs/Imu | IMU data out |
| `/camera/camera/depth/image_rect_raw` | sensor_msgs/Image | Depth data out |
| `/camera/camera/color/image_raw` | sensor_msgs/Image | RGB colour out |

Note: RealSense topics use `/camera/camera/` double prefix — this is the realsense2_camera wrapper default.

---

## Project Structure

| Directory | Purpose |
|-----------|---------|
| `nodes/` | ROS 2 node scripts — run directly on Pi |
| `config/` | ROS 2 parameter files |
| `launch/` | Launch files |
| `docs/` | Architecture, hardware, interface contract, session log |
| `brainstorm/` | Design thinking, open questions, references |
| `tests/` | Integration smoke tests |

---

## Development Setup

### Machines
- **ThinkPad (Ubuntu 24.04)** — write code in Cursor, push to GitHub
- **Raspberry Pi 5** — runs all nodes, edit directly via Cursor SSH

### Connect to Pi via Cursor SSH
1. `Ctrl+Shift+P` → Remote-SSH: Connect to Host
2. Enter `babyrover@10.0.0.76`
3. Password: `babyrover`
4. Open folder `/home/babyrover/baby-rover`

### Connect to Pi via terminal
```bash
ssh babyrover@10.0.0.76
```

---

## Pi Setup — From Scratch

Follow these steps in order on a fresh Pi 5. Do not skip steps.

### 1. Install ROS 2 Jazzy
Follow the official guide for Ubuntu 24.04:
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

### 2. Source ROS 2 permanently (do once)
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify:
```bash
ros2 --version
```

### 3. Clone the repo
```bash
cd ~
git clone https://github.com/TYFYTR/baby-rover.git
cd baby-rover
```

### 4. Install system dependencies
```bash
sudo apt install python3-lgpio i2c-tools python3-smbus git cmake libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libssl-dev build-essential libcurl4-openssl-dev
```

### 5. Install Python dependencies
```bash
pip3 install mpu6050-raspberrypi --break-system-packages
```

### 6. Build librealsense2 from source
Intel does not provide ARM64 binaries — build from source. Takes ~15 minutes on Pi 5.
```bash
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false
make -j4
sudo make install
sudo ldconfig
```

Verify:
```bash
rs-enumerate-devices
# Should show: Intel RealSense D415
```

### 7. Install RealSense ROS 2 wrapper
```bash
sudo apt install ros-jazzy-realsense2-camera
```

### 8. Verify IMU wiring
MPU-6050 must be wired to Pi before this step. See [`docs/hardware.md`](docs/hardware.md).
```bash
i2cdetect -y 1
# You should see 68 in the grid — that's the MPU-6050
```

---

## Running the Nodes

Each node runs in its own terminal. SSH into the Pi for each.

### Motor Controller
Subscribes to `/cmd_vel`, drives motors, publishes `/odom`.
```bash
python3 ~/baby-rover/nodes/motor_controller.py
```

### IMU Driver
Reads MPU-6050, publishes `/imu/data` at 100Hz.
```bash
python3 ~/baby-rover/nodes/imu_driver.py
```

### RealSense D415
Publishes depth and colour streams.
```bash
ros2 launch realsense2_camera rs_launch.py
```

### Verify all topics are publishing
```bash
ros2 topic list
```

Expected output includes:
```
/camera/camera/color/image_raw
/camera/camera/depth/image_rect_raw
/cmd_vel
/imu/data
/odom
```

### Send a test velocity command
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" --once
```

---

## Useful ROS 2 CLI Commands

```bash
ros2 node list                                        # see all running nodes
ros2 topic list                                       # see all active topics
ros2 topic echo /odom                                 # print messages on a topic
ros2 topic hz /imu/data                               # check publish rate
ros2 node info /motor_controller                      # see node subscriptions and publishers
ros2 topic echo /camera/camera/depth/image_rect_raw   # verify depth stream
```

---

## Current Milestone Status

| Milestone | Status |
|-----------|--------|
| 1 — Environment setup | ✅ Complete |
| 2 — Sensors publishing | ✅ Complete |
| 3 — State machine | ⬜ Not started |
| 4 — EKF localisation | ⬜ Not started |
| 5 — Nav2 integration | ⬜ Not started |

### Milestone 2 detail
- `/cmd_vel` → motors responding ✅
- `/odom` publishing ✅
- `/imu/data` publishing ✅
- `/camera/camera/depth/image_rect_raw` publishing ✅
- `/camera/camera/color/image_raw` publishing ✅

---

## Known Issues

| Issue | Status | Notes |
|-------|--------|-------|
| Rover curves right under open loop | Investigating | Suspected cause: 10kΩ pulldown resistors on motor control pins. Dedicated troubleshooting session planned. Do not tune PID until resolved. |
| lgpio callbacks broken on Pi 5 | Workaround in place | lgpio 0.2.0.0 interrupt-driven callbacks don't work on Pi 5. Using 100Hz polling via ROS 2 timer instead. See `docs/decisions.md` ADR-001. |
| RealSense firmware outdated | Low priority | Firmware 5.12.10 installed, recommended 5.17.0.10. Not affecting operation. |

---

## Docs

- [`docs/hardware.md`](docs/hardware.md) — full wiring diagrams and pin assignments
- [`docs/dependencies.md`](docs/dependencies.md) — all packages and install commands
- [`docs/decisions.md`](docs/decisions.md) — architecture decision records (ADRs)
- [`docs/session_log.md`](docs/session_log.md) — chronological work log
- [`docs/interface_contract.md`](docs/interface_contract.md) — topic specs for sensor/nav team handoff