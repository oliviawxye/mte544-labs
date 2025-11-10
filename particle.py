
from mapUtilities import *
from utilities import *
from numpy import cos, sin
import numpy as np


class particle:

    def __init__(self, pose, weight):
        self.pose = pose
        self.weight = weight

    def motion_model(self, v, w, dt):
        #TODO: Implement the motion model for the particle
        """
        v: linear velocity
        w: angular velocity
        dt: time step
        """
        # See slide 69 Week 8 for reference

        x1 = self.pose[0]
        y1 = self.pose[1]
        th1 = self.pose[2]

        x2 = -v/w * sin(th1) + v/w * sin(th1 + w*dt)
        y2 = v/w * cos(th1) - v/w * cos(th1 + w*dt)
        th2 = w * dt

        self.pose[0] += x2
        self.pose[1] += y2
        self.pose[2] += th2

    # TODO: You need to explain the following function to TA
    def calculateParticleWeight(self, scanOutput: LaserScan, mapManipulatorInstance: mapManipulator, laser_to_ego_transformation: np.array):
        
        # Transform laser scan to map frame: T represents where the scanner’s data would appear in the map given this particle’s pose.
        T = np.matmul(self.__poseToTranslationMatrix(), laser_to_ego_transformation)

        # Convert laser scan to Cartesian coordinates (in scanner frame)
        _, scanCartesianHomo = convertScanToCartesian(scanOutput)

        # Transform scan points to map frame
        scanInMap = np.dot(T, scanCartesianHomo.T).T

        likelihoodField = mapManipulatorInstance.getLikelihoodField()

        # Converts map coordinates to grid cell indices (pixels)
        cellPositions = mapManipulatorInstance.position_2_cell(
            scanInMap[:, 0:2])

        lm_x, lm_y = likelihoodField.shape

        # Keep only cell positions that are inside the map boundaries
        cellPositions = cellPositions[np.logical_and.reduce(
                (cellPositions[:, 0] > 0, -cellPositions[:, 1] > 0, cellPositions[:, 0] < lm_y,  -cellPositions[:, 1] < lm_x))]

        # Calculate the log likelihood for each point and sum them up
        log_weights = np.log(
            likelihoodField[-cellPositions[:, 1], cellPositions[:, 0]])
        log_weight = np.sum(log_weights) # Total log likelihood
        weight = np.exp(log_weight) # Convert back to normal weight
        weight += 1e-10 #  To avoid zero weight

        # Update particle final weight
        self.setWeight(weight)

    def setWeight(self, weight):
        self.weight = weight

    def getWeight(self):
        return self.weight

    def setPose(self, pose):
        self.pose = pose

    def getPose(self):
        return self.pose[0], self.pose[1], self.pose[2]

    def __poseToTranslationMatrix(self):
        x, y, th = self.getPose()

        translation = np.array([[cos(th), -sin(th), x],
                                [sin(th), cos(th), y],
                                [0, 0, 1]])

        return translation
