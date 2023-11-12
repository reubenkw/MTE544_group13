import sys

from utilities import Logger

from rclpy.time import Time

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
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
    
    def __init__(self, type, dt, loggerName="robotPose.csv", loggerHeaders=["imu_ax", "imu_ay", "kf_ax", "kf_ay","kf_vx","kf_w","x", "y","stamp"]):

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

        x= np.zeros(6) # initial state
        
        Q= 0.5 * np.eye(6)

        R= 0.5 * np.eye(6)
        
        P= np.zeros((6, 6)) # initial covariance
        
        self.kf=kalman_filter(P, Q, R, x, dt)
        
        # TODO Part 3: Use the odometry and IMU data for the EKF
        # self.odom_sub=message_filters.Subscriber(...)
        # self.imu_sub=message_filters.Subscriber(...)
        n0 = create_node("odom_sub")
        n1 = create_node("imu_sub")
        self.odom_sub= message_filters.Subscriber(n0, odom, "/odom")
        self.imu_sub=message_filters.Subscriber(n1, Imu, "/imu")
        
        time_syncher=message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub], queue_size=10, slop=0.1)
        time_syncher.registerCallback(self.fusion_callback)
        print("Done Kalman filter init")
    
    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):
        
        # TODO Part 3: Use the EKF to perform state estimation
        # Take the measurements
        # your measurements are the linear velocity and angular velocity from odom msg
        # and linear acceleration in x and y from the imu msg
        # the kalman filter should do a proper integration to provide x,y and filter ax,ay
        lin_vel = odom_msg.twist.twist.linear
        lin_accel = imu_msg.linear_acceleration
        ang_vel = odom_msg.twist.twist.angular

        print(f"odom vel y: {lin_vel.y}")

        z= np.array([np.sqrt(lin_vel.x ** 2 + lin_vel.y ** 2),
                     ang_vel.z,
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
        kf_ax, kf_ay, kf_vx, kf_w = self.kf.measurement_model()
        self.loc_logger.log_values([
            lin_accel.x,
            lin_accel.y,
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
