# Imports


import sys

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error, is_sim
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
    
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, goalPoint=None, rate=10, motion_type=POINT_PLANNER):

        super().__init__("decision_maker")
        self.th_linear = 0.1    # [m]
        self.th_angular = 0.1   # [rad]

        #TODO Part 4: Create a publisher for the topic responsible for robot's motion
        self.publisher=self.create_publisher(publisher_msg, publishing_topic, qos_publisher)

        publishing_period=1/rate
        
        # Instantiate the controller
        # TODO Part 5: Tune your parameters here
    
        if motion_type == POINT_PLANNER:
            self.controller=controller(klp=0.2, klv=0.5, kap=0.8, kav=0.6)
            self.planner=planner(POINT_PLANNER)    
    
    
        elif motion_type==TRAJECTORY_PLANNER:
            self.controller=trajectoryController(klp=0.2, klv=0.5, kap=0.8, kav=0.6)
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
        # may not need to explicitly create the node since already running in the decision_maker
        self.localizer.spin()   # Remember that this file is already running the decision_maker node.

        if self.localizer.getPose()  is  None:
            print("waiting for odom msgs ....")
            return

        vel_msg=Twist()
        
        # TODO Part 3: Check if you reached the goal
        # Check by comparing actual and threhshold angular and linear error
        end_goal = self.goal
        if isinstance(self.goal, list): 
            end_goal = self.goal[-1]
            
        # must meet angular and linear thresholds
        crnt_pose = self.localizer.getPose()
        reached_goal = calculate_linear_error(crnt_pose, end_goal) < self.th_linear and abs(calculate_angular_error(crnt_pose, end_goal)) < self.th_angular
        
        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg)
            
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()
            
            #TODO Part 3: exit the spin
            raise SystemExit()
        
        velocity, yaw_rate = self.controller.vel_request(self.localizer.getPose(), self.goal, True)

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
    # Later -- need to self identify if real or simulation robot
    odom_qos=QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=1, depth=10)
    

    # TODO Part 3: instantiate the decision_maker with the proper parameters for moving the robot
    if args.motion.lower() == "point":
        # uses arbitrary point
        DM=decision_maker(Twist(), "/cmd_vel", odom_qos, goalPoint=, rate=10, motion_type=POINT_PLANNER)
    elif args.motion.lower() == "trajectory":
        traj_planner = planner(TRAJECTORY_PLANNER).plan()
        DM=decision_maker(Twist(), "/cmd_vel", odom_qos, traj_planner, rate=10, motion_type=TRAJECTORY_PLANNER)
    else:
        print("invalid motion type", file=sys.stderr)        
    
    
    
    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")


if __name__=="__main__":

    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="point")
    argParser.add_argument("--sim", type=bool, default=False)
    args = argParser.parse_args()

    main(args)
