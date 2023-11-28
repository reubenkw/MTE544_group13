import sys
import time
from utilities import Logger

from rclpy.time import Time

from utilities import *
from rclpy.node import Node
from geometry_msgs.msg import Twist


from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from sensor_msgs.msg import Imu
from kalman_filter import kalman_filter

from rclpy import init, spin, spin_once

import numpy as np
import message_filters



rawSensors=0; kalmanFilter=1

odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)


class localization(Node):
    
    def __init__(self, type, loggerName="robotPose.csv", loggerHeaders=["pos_x", "pos_y", "pos_theta", "stamp"]):

        super().__init__("localizer")
        
        
        self.loc_logger=Logger( loggerName , loggerHeaders)
        self.pose=None
        
        if type==rawSensors:
            self.initRawSensors()
        elif type==kalmanFilter:
            self.initKalmanfilter()
            self.kalmanInitialized = False
        else:
            print("We don't have this type for localization", sys.stderr)
            return            
    
        self.timelast=time.time()
    
    def initRawSensors(self):
        self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)

    def initKalmanfilter(self):
        
        self.odom_sub=message_filters.Subscriber(self, odom, "/odom", qos_profile=odom_qos)
        self.imu_sub=message_filters.Subscriber(self, Imu, "/imu", qos_profile=odom_qos)
        
        time_syncher=message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub], queue_size=10, slop=0.1)
        
        time_syncher.registerCallback(self.fusion_callback)
        
    
    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):

        if not self.kalmanInitialized:
            x=np.array([odom_msg.pose.pose.position.x,
                        odom_msg.pose.pose.position.y,
                        euler_from_quaternion(odom_msg.pose.pose.orientation),
                        0,
                        0,
                        0])        
            
            # TODO PART 5 Bonus put the Q and R matrices
            # that you conclude from lab Three
            Q=np.array([0.2, 0.2, 0.05, 0.05, 0.01, 0.02]) * np.eye(6)
            R=np.array([0.05, 0.03, 2.5, 0.6]) * np.eye(4)
            P=np.zeros((6, 6))
                        
            self.kf=kalman_filter(P,Q,R, x)
            
            self.kalmanInitialized = True

        
        dt = time.time() - self.timelast

        self.timelast=time.time()


        z=np.array([odom_msg.twist.twist.linear.x,
                    odom_msg.twist.twist.angular.z,
                    imu_msg.linear_acceleration.x,
                    imu_msg.linear_acceleration.y])
        
        self.kf.predict(dt)
        self.kf.update(z)
        
        xhat=self.kf.get_states()
        
        self.pose=np.array([xhat[0],
                            xhat[1],
                            normalize_angle(xhat[2]),
                            odom_msg.header.stamp])
        
        self.loc_logger.log_values([
                odom_msg.pose.pose.position.x, 
                odom_msg.pose.pose.position.y, 
                euler_from_quaternion(odom_msg.pose.pose.orientation), 
                Time.from_msg(self.pose[3]).nanoseconds
            ])
        
    def odom_callback(self, pose_msg):
        
        self.pose=[ pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    euler_from_quaternion(pose_msg.pose.pose.orientation),
                    pose_msg.header.stamp]
        
        self.loc_logger.log_values([self.pose[0], self.pose[1], self.pose[2], Time.from_msg(self.pose[3]).nanoseconds])

        
    def getPose(self):
        return self.pose


if __name__=="__main__":
    
    init()
    
    LOCALIZER=localization()
    
    
    spin(LOCALIZER)
