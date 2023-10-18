from typing import List
from math import exp

# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1



class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0, 0.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        theta = goalPoint[2]
        return x, y, theta

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self) -> List[tuple[float, float]]:
        FUNCTION = "POLYNOMIAL"  # options: POLYNOMIAL, EXPONENTIAL
        STEP_SIZE: float = 0.05
        FINAL_X: float = 2

        trajectory: List[tuple[float, float]] = []
        x = 0

        while x <= FINAL_X:
            if FUNCTION == "POLYNOMIAL":
                trajectory.append((x, x ** 2))
            elif FUNCTION == "EXPONENTIAL":
                trajectory.append((x, 1/(1+exp(-x))))
            x += STEP_SIZE

        print(trajectory)

        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        # return

        return trajectory

