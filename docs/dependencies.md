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