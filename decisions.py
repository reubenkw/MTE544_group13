# Imports


import sys

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensor

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController

# You may add any other imports you may need/want to use below

from rclpy.qos import ReliabilityPolicy, DurabilityPolicy
from math import pi


class decision_maker(Node):
    
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, goalPoint=None, rate=10, motion_type=POINT_PLANNER, is_sim=False):

        super().__init__("decision_maker")
        self.th_linear = 0.1    # [m]
        self.th_angular = 0.1   # [rad]

        #TODO Part 4: Create a publisher for the topic responsible for robot's motion
        self.publisher=self.create_publisher(publisher_msg, publishing_topic, qos_publisher)

        publishing_period=1/rate
        
        # Instantiate the controller
        # TODO Part 5: Tune your parameters here
    
        if motion_type == POINT_PLANNER:
            self.controller=controller(klp=0.8, klv=0.5, kap=0.8, kav=0.6, is_sim=is_sim)
            self.planner=planner(POINT_PLANNER)    
    
    
        elif motion_type==TRAJECTORY_PLANNER:
            self.controller=trajectoryController(klp=0.2, klv=0.5, kap=0.8, kav=0.6, is_sim=is_sim)
            self.planner=planner(TRAJECTORY_PLANNER)

        else:
            print("Error! you don't have this planner", file=sys.stderr)


        # Instantiate the localization, use rawSensor for now  
        self.localizer=localization(rawSensor)

        # Instantiate the planner
        # NOTE: goalPoint is used only for the pointPlanner
        self.goal=self.planner.plan(goalPoint)

        self.create_timer(publishing_period, self.timerCallback)


    def timerCallback(self):
        
        # TODO Part 3: Run the localization node
        spin_once(self.localizer)   # Remember that this file is already running the decision_maker node.

        if self.localizer.getPose()  is  None:
            print("waiting for odom msgs ....")
            return

        vel_msg=Twist()
        
        # TODO Part 3: Check if you reached the goal
        # Check by comparing actual and threhshold angular and linear error
        
        crnt_pose = self.localizer.getPose()
        if isinstance(self.goal, list): 
            # no angular threshold for trajectory
            reached_goal = calculate_linear_error(crnt_pose, self.goal[-1]) < self.th_linear
        else:
            # must meet angular and linear thresholds
            reached_goal = calculate_linear_error(crnt_pose, self.goal) < self.th_linear
        
        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg)
            
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()
            
            #TODO Part 3: exit the spin
            raise SystemExit()
        
        print(crnt_pose[:3])
        velocity, yaw_rate = self.controller.vel_request(crnt_pose, self.goal, True)
        print(velocity, yaw_rate)


        #TODO Part 4: Publish the velocity to move the robot
        twist = Twist()
        twist.linear.x = velocity
        twist.angular.z = yaw_rate
        
        self.publisher.publish(twist)

import argparse


def main(args=None):
    
    init()

    # TODO Part 3: You migh need to change the QoS profile based on whether you're using the real robot or in simulation.
    # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
    
    """
    QoS profile:
    Reliability: RELIABLE - remember to change it to BEST_EFFORT in turtlebot4
    History (Depth): 10
    Durability: VOLATILE
    """
    odom_qos=QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=1, depth=10)
    

    # TODO Part 3: instantiate the decision_maker with the proper parameters for moving the robot
    if args.motion.lower() == "point":
        # uses arbitrary point
        goalPoint=[1.0, 1.0, 0.0]
        DM=decision_maker(Twist, "/cmd_vel", odom_qos, goalPoint, rate=10, motion_type=POINT_PLANNER, is_sim=args.is_sim)
    elif args.motion.lower() == "trajectory":
        DM=decision_maker(Twist, "/cmd_vel", odom_qos, rate=10, motion_type=TRAJECTORY_PLANNER, is_sim=args.is_sim)
    else:
        print("invalid motion type", file=sys.stderr)        
    
    
    
    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")


if __name__=="__main__":

    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="point")
    argParser.add_argument("--sim", dest="is_sim", default=False, action='store_true')
    args = argParser.parse_args()

    main(args)
