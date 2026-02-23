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

