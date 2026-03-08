# Baby Rover — Setup & Dependencies

Tested on: **Ubuntu 24.04 LTS (noble), aarch64 (Raspberry Pi 5)**

---

## 1. ROS 2

Install ROS 2 Jazzy (not Humble — Ubuntu 24.04 requires Jazzy):

Follow the official instructions at https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Add to `~/.bashrc`:
```bash
source /opt/ros/jazzy/setup.bash
```

---

## 2. ROS 2 message packages

```bash
sudo apt install ros-jazzy-geometry-msgs ros-jazzy-nav-msgs ros-jazzy-rclcpp
```

---

## 3. GPIO libraries

Two GPIO libraries are used — lgpio for motor PWM output, gpiod for encoder interrupt input.

```bash
sudo apt install libgpiod-dev liblgpio-dev python3-lgpio
```

**Important:** Only gpiod v1.6.3 is available in the Ubuntu 24.04 apt repos. The C++ node uses the gpiod v1 C API (`gpiod.h`), not the v2 C++ API (`gpiod.hpp`). Do not upgrade to gpiod v2 without rewriting the encoder section.

---

## 4. I2C tools (for IMU)

```bash
sudo apt install i2c-tools python3-smbus
```

Verify the MPU-6050 is detected on the I2C bus (should appear at address 0x68):
```bash
i2cdetect -y 1
```

---

## 5. IMU Python driver

```bash
pip3 install mpu6050-raspberrypi --break-system-packages
```

---

## 6. RealSense D415

No official ARM64 binary exists — must build librealsense2 from source. Compile time ~10-15 minutes on Pi 5.

```bash
sudo apt install git cmake libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libssl-dev build-essential libcurl4-openssl-dev
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false
make -j4
sudo make install
sudo ldconfig
```

Then install the ROS 2 wrapper:
```bash
sudo apt install ros-jazzy-realsense2-camera
```

Launch:
```bash
ros2 launch realsense2_camera rs_launch.py
```

---

## 7. Build tool (colcon)

Ubuntu 24.04 does not ship a working colcon apt package. Install via pip:

```bash
# If python3-colcon-ros is in a broken state:
sudo dpkg --remove --force-depends python3-colcon-ros

# Install colcon
pip3 install colcon-common-extensions --break-system-packages
```

---

## 8. Build the C++ node

```bash
cd ~/baby-rover
colcon build --packages-select rover_control
source install/setup.bash
```

This must be run from `~/baby-rover/` — not from inside `rover_control/`.

`source install/setup.bash` must be run in every new terminal before `ros2 run` will find the package. Add it to `~/.bashrc` to avoid doing this manually:

```bash
echo "source ~/baby-rover/install/setup.bash" >> ~/.bashrc
```

---

## 9. Run

**C++ node:**
```bash
ros2 run rover_control motor_controller
```

**Python node (retained, still functional):**
```bash
python3 ~/baby-rover/nodes/motor_controller.py
```

Do not run both at the same time — they both subscribe to `/cmd_vel` and publish to `/odom` and will conflict.

---

## 10. Send velocity commands (test)

From a second terminal:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

Or use keyboard teleop:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 11. Verify topics

```bash
ros2 topic echo /odom
ros2 topic hz /odom        # should be ~50 Hz
ros2 topic echo /cmd_vel
```

---

## Hardware pin mapping

| Signal | BCM GPIO |
|--------|----------|
| Motor A IN1 | 27 |
| Motor A IN2 | 17 |
| Motor A PWM | 13 |
| Motor B IN1 | 26 |
| Motor B IN2 | 19 |
| Motor B PWM | 12 |
| Encoder A | 21 |
| Encoder B | 24 |
| GPIO chip | /dev/gpiochip4 |

PWM frequency: 1000 Hz  
Control loop: 50 Hz  
Wheel base: 0.34 m  
Wheel circumference: 0.1382 m  
Pulses per revolution: 1050