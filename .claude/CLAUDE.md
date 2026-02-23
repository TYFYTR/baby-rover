# Baby Rover — Claude Context

## What this is
A ROS 2 prototype rover running on Raspberry Pi 5.
Hardware: N20 motors + encoders, TB6612 motor controller,
MPU-6050 IMU, RealSense D415.
Purpose: Learning platform and architectural prototype for
the Knuckey Rover university project (RMIT).

## The two projects
- P004043 Sensor Pack — sensor integration, time sync, calibration
- P004044 Nav & Safety — autonomous navigation, state machine, safety layer
- Baby rover mirrors P004044 architecture at prototype scale

## The chain
Sensors → ROS 2 Topics → EKF → Nav2 → /cmd_vel → PID → Motors → Encoders → back to EKF

## State machine states
Teleop → Auto → Pause → Safe-stop → Fault

## Key topic names
- /cmd_vel — velocity commands in
- /odom — odometry out
- /imu/data — IMU data out
- /camera/depth — RealSense D415 depth out

## Current state
ROS 2 Jazzy installed on ThinkPad Ubuntu 24.04.
Talker/listener demo confirmed working.
Folder structure scaffolded.
Pi 5 not yet configured.

## What I'm working on
Getting all sensors publishing to ROS 2 topics on ThinkPad.
