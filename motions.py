"""
Use this file to set robot on one of three predetermined patterns of motion:
    - line
    - circle (default)
    - spiral
Input using the command line argument --motion
"""

import os

import numpy as np
import rclpy

from rclpy.node import Node

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time

from rclpy.qos import ReliabilityPolicy, DurabilityPolicy


CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=["circle", "spiral", "line"]

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        super().__init__("motion_types")
        
        self.type=motion_type
        
        self.radius_=0.0
        
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False
        
        # for spiral twist (increasing rad clockwise)
        self.spiral_max = 0.0
        self.spiral_incr = 0.02
        self.ang_z = -5.0  # initial turning angle

        # velocity publisher
        self.vel_publisher=self.create_publisher(Twist, "/cmd_vel", 10)
                
        # loggers
        dir = f"data/{motion_types[motion_type]}"
        if not os.path.exists("data"):
            os.mkdir("data")
        if not os.path.exists(dir):
            os.mkdir(dir)
        self.imu_logger=Logger(f"{dir}/imu.csv", headers=["acc_x [m]", "acc_y [m]", "angular_z [rad]", "stamp"])
        self.odom_logger=Logger(f"{dir}/odom.csv", headers=["x [m]","y [m]","th [rad]", "stamp"])
        self.laser_logger=Logger(f"{dir}/laser.csv", headers=["ranges [m]", "stamp"])

        """
        QoS profile:
        Reliability: RELIABLE
        History (Depth): UNKNOWN
        Durability: VOLATILE
        """
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)

        # IMU subscription /imu
        self.imu_subscription = self.create_subscription(Imu, "/imu", self.imu_callback, qos_profile=qos)
        self.imu_initialized = True

        # ENCODER subscription /odom
        self.odom_subscription = self.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile=qos)
        self.odom_initialized = True

        # LaserScan subscription /scan
        self.laser_subscription = self.create_subscription(LaserScan, "/scan", self.laser_callback, qos_profile=qos)
        self.laser_initialized = True

        self.create_timer(0.1, self.timer_callback)
        
        self.successful_init = True


    # sensor callback functions
    # saves the needed fields into a list, and passes the list to the log_values function in utilities.py

    def imu_callback(self, imu_msg: Imu):
        # log imu msgs
        stamp = imu_msg.header.stamp.sec*1e9 + imu_msg.header.stamp.nanosec

        msg = [imu_msg.linear_acceleration.x,
               imu_msg.linear_acceleration.y,
               imu_msg.angular_velocity.z,
               stamp]
        
        self.imu_logger.log_values(msg)
        
    def odom_callback(self, odom_msg: Odometry):
        # log odom msgs
        th = euler_from_quaternion(odom_msg.pose.pose.orientation)
        
        stamp = odom_msg.header.stamp.sec*1e9 + odom_msg.header.stamp.nanosec
        
        msg = [odom_msg.pose.pose.position.x,
               odom_msg.pose.pose.position.y,
               th,
               stamp]

        self.odom_logger.log_values(msg)
                
    def laser_callback(self, laser_msg: LaserScan):
        # log laser msgs with position msg at that time

        # (Note: values < range_min or > range_max should be discarded)
        valid_ranges = [rng if laser_msg.range_min < rng < laser_msg.range_max else np.NaN for rng in laser_msg.ranges]

        stamp = laser_msg.header.stamp.sec*1e9 + laser_msg.header.stamp.nanosec

        self.laser_logger.log_values([valid_ranges,
                                      stamp])
                
    def timer_callback(self):
        
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
            
        if not self.successful_init:
            return
        
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        self.vel_publisher.publish(cmd_vel_msg)
        
    
    # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot

    def make_circular_twist(self):
        # twist msg for circular motion
        msg=Twist()
        msg.linear.x = 1.0
        msg.angular.z = -3.0
        return msg

    def make_spiral_twist(self):
        # twist msg for spiral motion
        # increasing rad clockwise
        if self.ang_z < self.spiral_max:
            # turning angle reduces in magnitude (-ve + +ve)
            self.ang_z += self.spiral_incr
        
        msg=Twist()
        msg.linear.x = 1.0
        msg.angular.z = self.ang_z
        return msg
    
    def make_acc_line_twist(self):
        # twist msg for line motion
        msg=Twist()
        msg.linear.x = 0.1
        return msg

import argparse

if __name__=="__main__":
    

    argParser=argparse.ArgumentParser(description="input the motion type")


    argParser.add_argument("--motion", type=str, default="circle")



    rclpy.init()

    args = argParser.parse_args()

    if args.motion.lower() == "circle":

        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)

    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)

    else:
        print(f"we don't have {args.motion.lower()} motion type")


    
    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
