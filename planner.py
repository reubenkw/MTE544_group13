from typing import List
from math import exp
import numpy as np
from utilities import Logger

# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1



class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0, 0.0], pose_init=[0,0,0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint, pose_init)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner(pose_init)


    def point_planner(self, goalPoint, pose_init):
        x0 = pose_init[0]
        y0 = pose_init[1]
        theta0 = pose_init[2]

        x = goalPoint[0] + x0
        y = goalPoint[1] + y0
        theta = goalPoint[2] + theta0
        return x, y, theta

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self, pose_init) -> List[tuple[float, float]]:
        x0 = pose_init[0]
        y0 = pose_init[1]
        FUNCTION = "EXPONENTIAL"  # options: POLYNOMIAL, EXPONENTIAL

        xs = np.arange(start=-1, stop=1, step=0.1)
        if FUNCTION == "POLYNOMIAL":
            f = lambda x: x ** 2
        elif FUNCTION == "EXPONENTIAL":
            f = lambda x: 1 / (1+exp(-10*x))

        fvec = np.vectorize(f)

        ys = fvec(xs)

        # make it relative to starting point
        xs += x0 - xs[0]
        ys += y0 - ys[0]

        # print(list(map(tuple, np.vstack((xs, ys)).T)))
        traj = list(map(tuple, np.vstack((xs, ys)).T))

        # save values for plotting
        logger = Logger("data/trajectory.csv", ["x [m]", "y [m]"])
        for x, y in traj:
            logger.log_values([x, y])
        logger.save_log()

        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        return traj

