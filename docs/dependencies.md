# Dependencies

## Pi 5 — System Packages
| Package | Purpose | Install Command |
|---------|---------|-----------------|
| python3-lgpio | GPIO control | `sudo apt install python3-lgpio` |
| i2c-tools | I2C bus scanner | `sudo apt install i2c-tools` |
| python3-smbus | I2C communication | `sudo apt install python3-smbus` |

## Pi 5 — Python Packages
| Package | Purpose | Install Command |
|---------|---------|-----------------|
| mpu6050-raspberrypi | MPU-6050 driver | `pip3 install mpu6050-raspberrypi --break-system-packages` |

## ThinkPad — ROS 2
| Package | Purpose |
|---------|---------|
| ROS 2 Jazzy | Core framework |

## Pi 5 — RealSense D415

### Build librealsense2 from source (required — no ARM64 binary available)
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
Compile time: ~10-15 minutes on Pi 5.

### ROS 2 wrapper
```bash
sudo apt install ros-jazzy-realsense2-camera
```

### Launch
```bash
ros2 launch realsense2_camera rs_launch.py
```