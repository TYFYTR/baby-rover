# Hardware

## Compute
- **Raspberry Pi 5** (8GB) — main compute board
- Ubuntu 24.04 LTS server (aarch64)
- Boot: Crucial BX500 1TB SSD via USB 3.0 enclosure (USB-A into Pi blue port)

## Motors
- **2x N20 Motors with Encoders** — SKU: WS-26377 — differential drive
- **TB6612FNG Dual Motor Controller** — SKU: POLOLU-713
- Encoder feedback used for wheel odometry

## Sensors
- **MPU-6050 IMU** — SKU: 018-MPU-6050
  - 3-axis gyroscope + 3-axis accelerometer
  - Interface: I2C
- **Intel RealSense D415** — RGB-D depth camera
  - Interface: USB 3.0

## ROS 2 Topic Names
These are standard ROS 2 message types — not custom. Using standard names ensures
compatibility with Nav2, robot_localization, and any other ROS 2 package.

| Topic | Message Type | Source |
|-------|-------------|--------|
| `/cmd_vel` | geometry_msgs/Twist | Input — velocity commands from Nav2 or teleop |
| `/odom` | nav_msgs/Odometry | Output — wheel encoder odometry |
| `/imu/data` | sensor_msgs/Imu | Output — MPU-6050 fused IMU data |
| `/camera/depth` | sensor_msgs/Image | Output — RealSense D415 depth frame |
| `/camera/color` | sensor_msgs/Image | Output — RealSense D415 RGB frame |

## Known Issues
- **RealSense D415 on Pi 5 (ARM64):** Intel's official librealsense2 Debian packages
  are x86 only — no ARM64 binary available. Solution is to build librealsense2 from
  source on the Pi. This is a known, solvable problem with documented community
  instructions.
- **N20 wheel speed mismatch:** wheels run at slightly different speeds under open-loop
  control. Fix: encoder feedback + PID I-term correction. Deferred to PID tuning phase.

## Wheel & Encoder Specs
| Parameter | Value |
|-----------|-------|
| Wheel diameter | 44mm |
| Wheel circumference | 138.2mm |
| Encoder resolution | 7 PPR (hall effect) |
| Gear ratio | 150:1 |
| Output shaft resolution | 1050 PPR |
| Distance per pulse | 0.1316mm |
| Wheelbase | 0.12m (estimated — measure and update) |
| Motor A | Right wheel |
| Motor B | Left wheel |

## Wiring

### Power
| Net | From | To | Notes |
|-----|------|----|-------|
| 3.3V | Pi Pin 1 | TB6612 VCC | Logic power |
| 3.3V | Pi Pin 1 | Encoder A VCC | |
| 3.3V | Pi Pin 1 | Encoder B VCC | |
| VMOT | 6x AA (~9V) positive | TB6612 VMOT | Motor power |
| GND | Pi Pin 9 | TB6612 GND | Common ground |
| GND | Pi Pin 9 | Encoder A GND | |
| GND | Pi Pin 9 | Encoder B GND | |
| GND | Pi Pin 9 | Battery negative | |

### TB6612 Enable
| Signal | From | To | Notes |
|--------|------|----|-------|
| STBY | Pi Pin 1 (3.3V) | 10kΩ → STBY | Pulled HIGH permanently — motors always enabled |

### Motor Control (Pi → TB6612)
| Signal | Pi Pin | GPIO | 10kΩ pulldown to GND | Motor |
|--------|--------|------|----------------------|-------|
| AIN1 | 13 | GPIO 27 | Yes | A (Right) |
| AIN2 | 11 | GPIO 17 | Yes | A (Right) |
| PWMA | 33 | GPIO 13 | Yes | A (Right) |
| BIN1 | 37 | GPIO 26 | Yes | B (Left) |
| BIN2 | 35 | GPIO 19 | Yes | B (Left) |
| PWMB | 32 | GPIO 12 | Yes | B (Left) |

### Encoders (Pi ← Encoders)
| Signal | Pi Pin | GPIO | Motor |
|--------|--------|------|-------|
| Encoder A Phase A | 18 | GPIO 24 | A (Right) |
| Encoder A Phase B | 16 | GPIO 23 | A (Right) |
| Encoder B Phase A | 21 | GPIO 9 | B (Left) |
| Encoder B Phase B | 19 | GPIO 10 | B (Left) |

### IMU (Pi ← MPU-6050)
| Signal | Pi Pin | GPIO |
|--------|--------|------|
| VCC | 1 | 3.3V |
| GND | 9 | GND |
| SDA | 3 | GPIO 2 |
| SCL | 5 | GPIO 3 |


### Known Wiring Issues
- **10kΩ pulldowns on all 6 motor control pins** — added for safety during initial setup.
  Suspected contributor to motor speed asymmetry (slight right curve). To be removed and
  retested during dedicated troubleshooting session. Do not tune PID until pulldowns resolved.
- **Motor curve** — rover drifts right under open-loop control. Root cause not yet isolated.
  Troubleshooting plan: strip pulldowns first, retest from zero, document each result.


