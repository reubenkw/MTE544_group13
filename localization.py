import sys

from utilities import Logger

from rclpy.time import Time

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.node import Node
from rclpy import create_node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from sensor_msgs.msg import Imu
from kalman_filter import kalman_filter

from rclpy import init, spin, spin_once

import numpy as np
import message_filters


rawSensors=0
kalmanFilter=1
odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)

class localization(Node):
    
    def __init__(self, type, dt, loggerName="robotPose.csv", loggerHeaders=["imu_ax", "imu_ay", "odom_v", "odom_w", "kf_ax", "kf_ay","kf_vx","kf_w","x", "y","stamp"]):

        super().__init__("localizer")

        self.loc_logger=Logger( loggerName , loggerHeaders)
        self.pose=None
        
        if type==rawSensors:
            self.initRawSensors()
        elif type==kalmanFilter:
            self.initKalmanfilter(dt)
        else:
            print("We don't have this type for localization", sys.stderr)
            return  

    def initRawSensors(self):
        self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)
        
    def initKalmanfilter(self, dt):
        
        # TODO Part 3: Set up the quantities for the EKF (hint: you will need the functions for the states and measurements)
        # CONFIRM THIS WITH TAS IN LAB #
        # Q= np.array([0.2, 0.2, 0.05, 0.05, 0.4, 0.1]) * np.eye(6) # trust motion
        # R= np.array([0.05, 0.03, 2.5, 0.6]) * np.eye(4)  # trust measurement

        x= np.zeros(6) # initial state
        
        # x, y, th, w, v, vdot
        Q= np.array([0.2, 0.2, 0.05, 0.05, 0.01, 0.02]) * np.eye(6) # trust motion

        # v, w, ax, ay (higher)
        R= np.array([0.05, 0.03, 2.5, 0.6]) * np.eye(4)  # trust measurement
        
        P= np.zeros((6, 6)) # initial covariance (maybe P=Q)
        
        self.kf=kalman_filter(P, Q, R, x, dt)
        
        # TODO Part 3: Use the odometry and IMU data for the EKF
        # self.odom_sub=message_filters.Subscriber(...)
        # self.imu_sub=message_filters.Subscriber(...)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.odom_sub = message_filters.Subscriber(self, odom, "/odom", qos_profile=qos)
        self.imu_sub = message_filters.Subscriber(self, Imu, "/imu", qos_profile=qos)
        
        time_syncher=message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub], queue_size=10, slop=0.1)
        time_syncher.registerCallback(self.fusion_callback)
        print("Done Kalman filter init")
    
    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):
        
        # TODO Part 3: Use the EKF to perform state estimation
        # Take the measurements
        # your measurements are the linear velocity and angular velocity from odom msg
        # and linear acceleration in x and y from the imu msg
        # the kalman filter should do a proper integration to provide x,y and filter ax,ay
        print("fusion callback")
        lin_vel = odom_msg.twist.twist.linear
        lin_accel = imu_msg.linear_acceleration
        ang_vel = odom_msg.twist.twist.angular

        odom_v = np.sqrt(lin_vel.x ** 2 + lin_vel.y ** 2)
        odom_w = ang_vel.z
        z = np.array([odom_v,
                     odom_w,
                     lin_accel.x,
                     lin_accel.y])
        
        # Implement the two steps for estimation
        self.kf.predict()
        self.kf.update(z)
        
        # Get the estimate
        # states: x, y, th, w, v, vdot
        xhat=self.kf.get_states()

        # Update the pose estimate to be returned by getPose
        self.pose=np.array([
            xhat[0],
            xhat[1],
            xhat[2],
            odom_msg.header.stamp,
        ])

        # TODO Part 4: log your data
        # "imu_ax", "imu_ay", "kf_ax", "kf_ay","kf_vx","kf_w","x", "y","stamp"
        time_ns = Time.from_msg(odom_msg.header.stamp).nanoseconds
        kf_vx, kf_w, kf_ax, kf_ay = self.kf.measurement_model()
        self.loc_logger.log_values([
            lin_accel.x,
            lin_accel.y,
            odom_v,
            odom_w,
            kf_ax,
            kf_ay,
            kf_vx,
            kf_w,
            xhat[0],
            xhat[1],
            time_ns,
            ])
      
    def odom_callback(self, pose_msg):
        
        self.pose=[ pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    euler_from_quaternion(pose_msg.pose.pose.orientation),
                    pose_msg.header.stamp]

    # Return the estimated pose
    def getPose(self):
        return self.pose


if __name__=="__main__":
    
    init()
    
    LOCALIZER=localization()
    
    spin(LOCALIZER)
