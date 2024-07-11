#!/usr/bin/env python
# encoding: utf-8

# Public libraries
import sys
import math
import random
import threading
from math import pi,sqrt,atan,tan
from time import sleep
from Rosmaster_Lib import Rosmaster
from .Bicycle import Bicycle as bicy  # Importación corregida

# ROS libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32, Bool
from geometry_msgs.msg import Twist, Pose, Quaternion, TransformStamped
from sensor_msgs.msg import Imu, MagneticField, JointState
from nav_msgs.msg import Odometry
from rclpy.clock import Clock
from tf2_ros import TransformBroadcaster

class yahboomcar_driver(Node):
    def __init__(self, name):
        super().__init__(name)
        self.RA2DE = 180 / pi
        self.car = Rosmaster(com="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
        self.car.set_car_type(5)

        # Get parameters
        self.declare_parameter('imu_link', 'imu_link')
        self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value        
        self.declare_parameter('Prefix', "")
        self.Prefix = self.get_parameter('Prefix').get_parameter_value().string_value
        self.declare_parameter('Lon', 0.2275)
        self.L = self.get_parameter('Lon').get_parameter_value().double_value

        # Create subscribers
        self.sub_cmd_vel = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 1)
        self.sub_RGBLight = self.create_subscription(Int32, "RGBLight", self.RGBLightcallback, 100)
        self.sub_BUzzer = self.create_subscription(Bool, "Buzzer", self.Buzzercallback, 100)

        # Create publishers
        self.EdiPublisher = self.create_publisher(Float32, "edition", 100)
        self.volPublisher = self.create_publisher(Float32, "voltage", 100)
        self.imuPublisher = self.create_publisher(Imu, "/imu/data_raw", 100)
        self.magPublisher = self.create_publisher(MagneticField, "/imu/mag", 100)

        # Create timer
        self.timer = self.create_timer(0.1, self.pub_data)

        # Create and initialize variables
        self.edition = Float32()
        self.edition.data = 1.0
        self.car.create_receive_threading()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_time = self.get_clock().now()
        self.phi=0
        self.v=0
    # Callback functions
    def cmd_vel_callback(self, msg):
        if not isinstance(msg, Twist):
            return
        # Calculate speed
        self.v,self.phi = bicy.kinematic_inverse(msg.linear.x,msg.linear.y,msg.angular.z,self.L)
        speed_value = bicy.dead_zone_scaling(self.v,75,130,80,101,al=0.25,ah=0.30)
        steer_value = bicy.dead_zone_scaling(self.phi,75,130,85,101)
        # Set the servo values
        print(f"s: {self.v} d: {self.phi} ---->PWM s: {speed_value} d: {steer_value} ")
        self.car.set_pwm_servo(2, steer_value)
        self.car.set_pwm_servo(1, speed_value)



    def RGBLightcallback(self, msg):
        if not isinstance(msg, Int32):
            return
        for i in range(3):
            self.car.set_colorful_effect(msg.data, 6, parm=1)

    def Buzzercallback(self, msg):
        if not isinstance(msg, Bool):
            return
        if msg.data:
            for i in range(3):
                self.car.set_beep(1)
        else:
            for i in range(3):
                self.car.set_beep(0)
    # Publish data
    def pub_data(self):
        time_stamp = Clock().now()
        imu = Imu()
        battery = Float32()
        edition = Float32()
        mag = MagneticField()

        edition.data = self.car.get_version() * 1.0
        battery.data = self.car.get_battery_voltage() * 1.0
        ax, ay, az = self.car.get_accelerometer_data()
        gx, gy, gz = self.car.get_gyroscope_data()
        mx, my, mz = self.car.get_magnetometer_data()
        mx = mx * 1.0
        my = my * 1.0
        mz = mz * 1.0
        # vx, vy, angular = self.car.get_motion_data()

        # Publish gyroscope data
        imu.header.stamp = time_stamp.to_msg()
        imu.header.frame_id = self.imu_link
        imu.linear_acceleration.x = ax * 1.0
        imu.linear_acceleration.y = ay * 1.0
        imu.linear_acceleration.z = az * 1.0
        imu.angular_velocity.x = gx * 1.0
        imu.angular_velocity.y = gy * 1.0
        imu.angular_velocity.z = gz * 1.0
        mag.header.stamp = time_stamp.to_msg()
        mag.header.frame_id = self.imu_link
        mag.magnetic_field.x = mx * 1.0
        mag.magnetic_field.y = my * 1.0
        mag.magnetic_field.z = mz * 1.0
        self.imuPublisher.publish(imu)
        self.magPublisher.publish(mag)
        self.volPublisher.publish(battery)
        self.EdiPublisher.publish(edition)

        
def main():
    rclpy.init()
    driver = yahboomcar_driver('driver_node')
    rclpy.spin(driver)

if __name__ == '__main__':
    main()
