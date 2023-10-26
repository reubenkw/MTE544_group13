import numpy as np


from pid import PID_ctrl
from utilities import (
    euler_from_quaternion,
    calculate_angular_error,
    calculate_linear_error,
)
from numpy import sign

M_PI = 3.1415926535

P = 0
PD = 1
PI = 2
PID = 3

# turtlebot 3 burger specs
SIM_LINEAR_VEL_TH = 0.22  # [m/s]
SIM_ANGULAR_VEL_TH = 2.84  # [rad/s]

# turtlebot 4 specs in safe mode
REAL_LINEAR_VEL_TH = 0.31  # [m/s]
REAL_ANGULAR_VEL_TH = 1.90  # [rad/s]


class controller:
    # Default gains of the controller for linear and angular motions
    def __init__(
        self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2, is_sim=False
    ):
        # TODO Part 5 and 6: Modify the below lines to test your PD, PI, and PID controller
        self.PID_linear = PID_ctrl(P, klp, klv, kli, filename_="data/linear.csv")
        self.PID_angular = PID_ctrl(P, kap, kav, kai, filename_="data/angular.csv")

        if is_sim:
            self.linear_vel_th = SIM_LINEAR_VEL_TH
            self.angular_vel_th = SIM_ANGULAR_VEL_TH
        else:
            self.linear_vel_th = REAL_LINEAR_VEL_TH
            self.angular_vel_th = REAL_ANGULAR_VEL_TH

    def vel_request(self, pose, goal, status):
        e_lin = calculate_linear_error(pose, goal)
        e_ang = calculate_angular_error(pose, goal)

        print(f"erorr: {e_lin}, {e_ang}")

        linear_vel = self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel = self.PID_angular.update([e_ang, pose[3]], status)

        # TODO Part 4: Add saturation limits for the robot linear and angular velocity
        linear_vel = (
            sign(linear_vel) * self.linear_vel_th
            if abs(linear_vel) > self.linear_vel_th
            else linear_vel
        )
        angular_vel = (
            sign(angular_vel) * self.angular_vel_th
            if abs(angular_vel) > self.angular_vel_th
            else angular_vel
        )

        return linear_vel, angular_vel


class trajectoryController(controller):
    def __init__(
        self,
        klp=0.2,
        klv=0.2,
        kli=0.2,
        kap=0.2,
        kav=0.2,
        kai=0.2,
        lookAhead=1.0,
        is_sim=False,
    ):
        super().__init__(klp, klv, kli, kap, kav, kai, is_sim)
        self.lookAhead = lookAhead

    def vel_request(self, pose, listGoals, status):
        goal = self.lookFarFor(pose, listGoals)

        finalGoal = listGoals[-1]

        e_lin = calculate_linear_error(pose, finalGoal)
        e_ang = calculate_angular_error(pose, goal)

        linear_vel = self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel = self.PID_angular.update([e_ang, pose[3]], status)

        # TODO Part 4: Add saturation limits for the robot linear and angular velocity
        linear_vel = (
            sign(linear_vel) * self.linear_vel_th
            if abs(linear_vel) > self.linear_vel_th
            else linear_vel
        )
        angular_vel = (
            sign(angular_vel) * self.angular_vel_th
            if abs(angular_vel) > self.angular_vel_th
            else angular_vel
        )

        return linear_vel, angular_vel

    def lookFarFor(self, pose, listGoals):
        poseArray = np.array([pose[0], pose[1]])
        listGoalsArray = np.array(listGoals)

        distanceSquared = np.sum((listGoalsArray - poseArray) ** 2, axis=1)
        closestIndex = np.argmin(distanceSquared)

        return listGoals[min(closestIndex + 3, len(listGoals) - 1)]
