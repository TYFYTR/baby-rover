#!/usr/bin/env python3
"""
Motor controller node — open loop.
Subscribes to /cmd_vel, drives TB6612 via GPIO.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import lgpio
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import math



# --- Pin definitions (BCM numbering) ---
AIN1 = 27  # Motor A (Right) direction
AIN2 = 17  # Motor A (Right) direction
PWMA = 13  # Motor A (Right) speed

BIN1 = 26  # Motor B (Left) direction
BIN2 = 19  # Motor B (Left) direction
PWMB = 12  # Motor B (Left) speed

STBY = None  # Pulled HIGH via resistor — no GPIO control needed

# --- Robot physical parameters ---
WHEEL_BASE = 0.12       # metres — distance between wheels, measure and update
MAX_SPEED = 100         # PWM duty cycle max (0-100)

MOTOR_PINS = [AIN1, AIN2, PWMA, BIN1, BIN2, PWMB]


class MotorController(Node):

    def __init__(self):
    
        super().__init__('motor_controller')
        self.get_logger().info('Motor controller starting...')

        # Init GPIO
        self.chip = lgpio.gpiochip_open(4)
        for pin in MOTOR_PINS:
            lgpio.gpio_claim_output(self.chip, pin, 0)

        # PWM setup — 1000 Hz
        lgpio.tx_pwm(self.chip, PWMA, 1000, 0)
        lgpio.tx_pwm(self.chip, PWMB, 1000, 0)

        # Encoder setup
        self.enc_a_count = 0
        self.enc_b_count = 0
        self.enc_a_last = 0
        self.enc_b_last = 0

        lgpio.gpio_claim_input(self.chip, 24)
        lgpio.gpio_claim_input(self.chip, 9)

        # Poll encoders at 100Hz
        self.create_timer(0.01, self.poll_encoders)

        # ROS subscriber
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info('Motor controller ready.')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_enc_a = 0
        self.last_enc_b = 0
        self.create_timer(0.02, self.publish_odom)  # 50Hz

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'cmd_vel: linear={msg.linear.x:.2f} angular={msg.angular.z:.2f}')
        self.get_logger().info(f'enc_a={self.enc_a_count} enc_b={self.enc_b_count}')
        linear = msg.linear.x
        angular = msg.angular.z

        # Differential drive mixing
        right = linear - (angular * WHEEL_BASE / 2.0)
        left  = linear + (angular * WHEEL_BASE / 2.0)

        self.drive_motor_a(right)
        self.drive_motor_b(left)

    def drive_motor_a(self, speed):
        duty = min(abs(speed) * MAX_SPEED, MAX_SPEED)
        if speed > 0:
            lgpio.gpio_write(self.chip, AIN1, 1) # was 0
            lgpio.gpio_write(self.chip, AIN2, 0) # was 1
        elif speed < 0:
            lgpio.gpio_write(self.chip, AIN1, 0) # was 1
            lgpio.gpio_write(self.chip, AIN2, 1) # was 0
        else:
            lgpio.gpio_write(self.chip, AIN1, 0)
            lgpio.gpio_write(self.chip, AIN2, 0)
        lgpio.tx_pwm(self.chip, PWMA, 1000, duty)

    def drive_motor_b(self, speed):
        duty = min(abs(speed) * MAX_SPEED, MAX_SPEED)
        if speed > 0:
            lgpio.gpio_write(self.chip, BIN1, 0)
            lgpio.gpio_write(self.chip, BIN2, 1)
        elif speed < 0:
            lgpio.gpio_write(self.chip, BIN1, 1)
            lgpio.gpio_write(self.chip, BIN2, 0)
        else:
            lgpio.gpio_write(self.chip, BIN1, 0)
            lgpio.gpio_write(self.chip, BIN2, 0)
        lgpio.tx_pwm(self.chip, PWMB, 1000, duty)

    def stop_motors(self):
        lgpio.gpio_write(self.chip, AIN1, 0)
        lgpio.gpio_write(self.chip, AIN2, 0)
        lgpio.gpio_write(self.chip, BIN1, 0)
        lgpio.gpio_write(self.chip, BIN2, 0)
        lgpio.tx_pwm(self.chip, PWMA, 1000, 0)
        lgpio.tx_pwm(self.chip, PWMB, 1000, 0)

    def destroy_node(self):
        self.stop_motors()
        lgpio.gpiochip_close(self.chip)
        super().destroy_node()

    def poll_encoders(self):
        a = lgpio.gpio_read(self.chip, 24)
        b = lgpio.gpio_read(self.chip, 9)
        
        if a == 1 and self.enc_a_last == 0:
            self.enc_a_count += 1
        if b == 1 and self.enc_b_last == 0:
            self.enc_b_count += 1
            
        self.enc_a_last = a
        self.enc_b_last = b
    
    def publish_odom(self):
        PULSES_PER_REV = 1050
        WHEEL_CIRCUMFERENCE = 0.1382  # metres

        delta_a = self.enc_a_count - self.last_enc_a
        delta_b = self.enc_b_count - self.last_enc_b
        self.last_enc_a = self.enc_a_count
        self.last_enc_b = self.enc_b_count

        dist_a = (delta_a / PULSES_PER_REV) * WHEEL_CIRCUMFERENCE
        dist_b = (delta_b / PULSES_PER_REV) * WHEEL_CIRCUMFERENCE

        dist = (dist_a + dist_b) / 2.0
        dtheta = (dist_a - dist_b) / WHEEL_BASE

        self.x += dist * math.cos(self.theta)
        self.y += dist * math.sin(self.theta)
        self.theta += dtheta

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.twist.twist.linear.x = dist / 0.02
        msg.twist.twist.angular.z = dtheta / 0.02

        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()