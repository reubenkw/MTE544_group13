
from mapUtilities import *
from a_star import *
from enum import Enum
from a_star import manhattan_cost, euclidean_cost

POINT_PLANNER=0; TRAJECTORY_PLANNER=1

class CostFunctions(Enum):
    EUCLIDEAN = 0,
    MANHATTAN = 1

class planner:
    def __init__(self, type_, mapName="room"):

        self.type=type_
        self.mapName=mapName

    
    def plan(self, startPose, endPose):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)
        
        elif self.type==TRAJECTORY_PLANNER:
            self.costMap=None
            self.initTrajectoryPlanner()
            return self.trajectory_planner(startPose, endPose)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):


        # TODO PART 5 Create the cost-map, the laser_sig is 
        # the standard deviation for the gausiian for which
        # the mean is located on the occupant grid. 
        
        # Tune?
        self.m_utilites=mapManipulator(laser_sig=0.17)
            
        self.costMap=self.m_utilites.make_likelihood_field()
        

    def trajectory_planner(self, startPoseCart, endPoseCart):
        # Set the cost function for astar heuristic
        cost_function_type = CostFunctions.EUCLIDEAN
        cost_function = euclidean_cost if cost_function_type == CostFunctions.EUCLIDEAN else manhattan_cost

        # This is to convert the cartesian coordinates into the 
        # the pixel coordinates of the map image, remmember,
        # the cost-map is in pixels. You can by the way, convert the pixels
        # to the cartesian coordinates and work by that index, the a_star finds
        # the path regardless. 

        startPose=self.m_utilites.position_2_cell(startPoseCart)
        endPose=self.m_utilites.position_2_cell(endPoseCart)

        print("start pose", startPoseCart, startPose)
        print("end pose", endPoseCart, endPose)
        # print("origin", self.m_utilites.getOrigin(), self.m_utilites.position_2_cell(self.m_utilites.getOrigin()))
        # print("res", self.m_utilites.getResolution())
        # print("shape (y, x)", self.costMap.shape)
        # TODO PART 5 convert the cell pixels into the cartesian coordinates
        found_path = search(self.costMap, cost_function, startPose, endPose)
        path_coords = list(map(self.m_utilites.cell_2_position, found_path))
        print(path_coords)

        # print("path", path_coords)


        # TODO PART 5 return the path as list of [x,y]
        return path_coords




if __name__=="__main__":

    m_utilites=mapManipulator()
    
    map_likelihood=m_utilites.make_likelihood_field()

    # you can use this part of the code to test your 
    # search algorithm regardless of the ros2 hassles
    
