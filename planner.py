
import math
from mapUtilities import mapManipulator
from a_star import search
import time
import numpy as np
import matplotlib.pyplot as plt

POINT_PLANNER=0; ASTAR_PLANNER=1

PARABOLA=0; SIGMOID=1

class planner:

    def __init__(self, type_, mapName="room"):

        self.type=type_
        self.mapName=mapName
        ## TODO: Adjust the laser_sig value which decides the safety distance to obstacles
        self.m_utilites = mapManipulator(filename_=self.mapName, laser_sig=0.5)
        self.costMap = self.m_utilites.make_likelihood_field()

    
    def plan(self, startPose, endPose):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)
        
        
        elif self.type==ASTAR_PLANNER:
            return self.trajectory_planner(startPose, endPose)
        
        else:
            print("Error! you don't have this type of planner")
            return None
        

    def point_planner(self, endPose):
        return (endPose[0], endPose[1])


    def trajectory_planner(self, startPoseCart, endPoseCart):
        start_time = time.time()
        startPoseCart = np.array(startPoseCart)[:2]
        endPoseCart = np.array(endPoseCart)[:2]

        # TODO: Convert to pixel coordinates using the m_utilites
        startPose = self.m_utilites.position_2_cell(startPoseCart)
        endPose = self.m_utilites.position_2_cell(endPoseCart)

        # convert to tuple
        startPose = (startPose[0], startPose[1])
        endPose = (endPose[0], endPose[1])
        # TODO: Call the A* search algorithm
        path = search(self.costMap, startPose, endPose)
        if path is None:
            return None
        
        pathCart = self.m_utilites.cell_2_position(path)
        pathCart_list = pathCart.tolist()
        print("Time taken for A* is ", time.time()-start_time)

        # Visualization
        allObstacles = np.array(self.m_utilites.getAllObstacles())
        plt.plot(allObstacles[:, 0], allObstacles[:, 1], 'ko', markersize=4)
        plt.axis('equal')
        plt.title('A* path planning')
        plt.xlabel('x')
        plt.ylabel('y')
        # plot the path
        plt.plot(pathCart[:, 0], pathCart[:, 1], 'b-', linewidth=3)
        # plot the start and end points
        plt.plot(startPoseCart[0], startPoseCart[1], 'g*', markersize=10)
        plt.plot(endPoseCart[0], endPoseCart[1], 'r*', markersize=10)
        plt.text(startPoseCart[0], startPoseCart[1], 'start', fontsize=16)
        plt.text(endPoseCart[0], endPoseCart[1], 'goal', fontsize=16)
        plt.show()

        return pathCart_list
        
    