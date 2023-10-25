import sys

from utilities import Logger, euler_from_quaternion
from rclpy.time import Time
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry as odom

from rclpy import init, spin

rawSensor = 0
class localization(Node):
    
    def __init__(self, localizationType=rawSensor):

        super().__init__("localizer")
        
        # TODO Part 3: Define the QoS profile variable based on whether you are using the simulation (Turtlebot 3 Burger) or the real robot (Turtlebot 4)
        # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3

        """
        QoS profile:
        Reliability: RELIABLE - remember to change it to BEST_EFFORT in turtlebot4
        History (Depth): 10
        Durability: VOLATILE
        """
        odom_qos=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)
        
        self.loc_logger=Logger("data/robot_pose.csv", ["x [m]", "y [m]", "$\\theta$ [rad]", "stamp"])
        self.pose=None
        
        if localizationType == rawSensor:
            # TODO Part 3: subscribe to the position sensor topic (Odometry)
            self.odom_subscription = self.create_subscription(odom,"/odom",self.odom_callback, qos_profile=odom_qos)
            self.odom_initialized = True
        else:
            print("This type doesn't exist", sys.stderr)
    
    
    def odom_callback(self, pose_msg: odom):
        
        # TODO Part 3: Read x,y, theta, and record the stamp
        th = euler_from_quaternion(pose_msg.pose.pose.orientation)

        self.pose=[pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, th, pose_msg.header.stamp]
        
        # Log the data
        time_ns = Time.from_msg(pose_msg.header.stamp).nanoseconds
        self.loc_logger.log_values([self.pose[0], self.pose[1], self.pose[2], time_ns])
    
    def getPose(self):
        return self.pose

# TODO Part 3
# Here put a guard that makes the node run, ONLY when run as a main thread!
if __name__ == '__main__':
    init()
    localizer = localization()
    spin(localizer)

    
