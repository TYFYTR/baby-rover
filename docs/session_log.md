# Session Log

## 2026-02-23-20-00 - Alex

### What I worked on
- Installed ROS 2 Jazzy on ThinkPad (Ubuntu 24.04)
- Flashed Ubuntu 24.04 server onto Crucial BX500 SSD for Pi 5
- SSH confirmed working ThinkPad → Pi 5
- Installed ROS 2 Jazzy on Pi 5
- Confirmed talker/listener demo working on both machines
- Confirmed cross-machine ROS 2 communication working over WiFi via DDS
- Created baby-rover repo structure and pushed to GitHub
- Installed Claude Code, ROS, GitLens, Python extensions in Cursor

### What broke
- librealsense2-dkms failed on kernel 6.17 on ThinkPad — removed for now, not needed yet
- Network dropped mid apt upgrade — fixed with --fix-missing flag
- SSH host key conflict after reflash — fixed with ssh-keygen -R

### What's next
- Get IMU publishing to /imu/data on Pi 5
- Get motors responding to /cmd_vel via keyboard teleop
- Get encoders publishing to /odom

## 2026-02-24-09-35 - Alex 

### What was done
- Confirmed wiring from memory and documented in hardware.md
- Identified 10kΩ pulldowns on motor control pins as suspected curve cause
- Deferred motor curve troubleshooting to dedicated session

### Decisions made
- Motor A = Right, Motor B = Left
- Pulldowns to be removed before PID tuning begins

### Blockers / open issues
- hardware.md wiring not yet committed to group repo

### Next session
- Write motor_controller node

## 2026-02-24-10-20 - Alex 

### Updated README

## 2026-02-24-15-14 - Alex

### What was done
- Confirmed Cursor SSH workflow — editing files directly on Pi
- Installed lgpio on Pi 5
- Discovered Pi 5 uses gpiochip4 not gpiochip0
- Wrote motor_controller node — open loop, subscribes to /cmd_vel
- Confirmed ROS 2 pub/sub pipeline working
- Motors responding to /cmd_vel — both wheels spinning correct direction
- Measured wheel diameter = 44mm
- Encoder resolution: 7PPR × 150 gear ratio = 1050 PPR, 1 pulse = 0.1316mm
- Encoder callbacks broken in lgpio 0.2.0.0 on Pi 5 — switched to 100Hz polling via ROS 2 timer
- /odom publishing and incrementing correctly with wheel movement


### Decisions made
- Running nodes directly with Python for now — no colcon package yet
- nodes/ directory on Pi for scripts
- No sudo required — babyrover already in gpio, i2c, spi groups
- Encoder polling at 100Hz instead of interrupt-driven callbacks — lgpio bug, revisit on university rover

### Deferred
- 10kΩ pulldown troubleshooting — suspected curve cause, dedicated session
- colcon workspace setup — after all nodes working
- /odom accuracy tuning — y drift and angular.z oscillation confirm wheel mismatch, do not tune until pulldowns resolved

### Next session
- IMU node — MPU-6050 publishing to /imu/data

## 2026-02-24-17-00 - Alex

### What was done 
- Wired MPU-6050 via I2C — GPIO 2 (SDA), GPIO 3 (SCL)
- Confirmed i2cdetect shows 0x68
- Wrote imu_driver node — publishes to /imu/data at 100Hz
- Installed i2c-tools, mpu6050-raspberrypi, python3-smbus on Pi
