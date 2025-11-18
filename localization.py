import sys
import time
from utilities import Logger

from rclpy.time import Time

from utilities import *
from rclpy.node import Node
from geometry_msgs.msg import Twist


from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from rclpy import init, spin, spin_once

import numpy as np
import message_filters



rawSensors=0; particlesFilter=1


odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)


class localization(Node):
    
    def __init__(self, type_, loggerName="robotPose.csv", loggerHeaders=["odom_x", "odom_y", "odom_th", "odom_vx", "odom_yawrate","pf_x","pf_y","pf_th","stamp"]):

        super().__init__("localizer")
        
        self.loc_logger=Logger( loggerName , loggerHeaders)
        self.pose=None
        
        
        if type_==rawSensors:
            self.initRawSensors()

        elif type_==particlesFilter:
            self.initParticleFilter()
        else:
            print("We don't have this type for localization", sys.stderr)
            return            
    
        self.timelast=time.time()
    
    def initRawSensors(self):
        self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)

    def initParticleFilter(self):
        self.odom_pose_sub=message_filters.Subscriber(self, odom, "/odom", qos_profile=odom_qos)
        self.pf_pose_sub=message_filters.Subscriber(self, odom, "/pf_pose", qos_profile=odom_qos)
        time_syncher=message_filters.ApproximateTimeSynchronizer([self.odom_pose_sub, self.pf_pose_sub], queue_size=10, slop=0.1)
        time_syncher.registerCallback(self.odom_and_pf_pose_callback)

    def odom_and_pf_pose_callback(self, odom_msg: odom, pf_msg: odom):
        # Use the pf_msg to update the pose of the robot [x, y, theta, stamp]
        self.pose = [pf_msg.pose.pose.position.x,
                     pf_msg.pose.pose.position.y,
                     euler_from_quaternion(pf_msg.pose.pose.orientation),
                     pf_msg.header.stamp]
        
        # log the values from the odom and the particle filter based on the headers
        # odom values: x, y, theta, vx, yawrate
        odom_values_list = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, euler_from_quaternion(
            odom_msg.pose.pose.orientation), odom_msg.twist.twist.linear.x, odom_msg.twist.twist.angular.z]
        # pf values: x, y, theta
        pf_values_list = [pf_msg.pose.pose.position.x, pf_msg.pose.pose.position.y,
                          euler_from_quaternion(pf_msg.pose.pose.orientation)]

        stamp = Time.from_msg(odom_msg.header.stamp).nanoseconds
        # Put all the values in a list
        values_to_log = odom_values_list + pf_values_list + [stamp]
        self.loc_logger.log_values(values_to_log)

        
    
    def odom_callback(self, pose_msg):
        self.pose=[ pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    euler_from_quaternion(pose_msg.pose.pose.orientation),
                    pose_msg.header.stamp]
        
        stamp = Time.from_msg(pose_msg.header.stamp).nanoseconds
        # Put all the values in a list
        values_to_log = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, euler_from_quaternion(pose_msg.pose.pose.orientation), pose_msg.twist.twist.linear.x, pose_msg.twist.twist.angular.z, 0, 0, 0, stamp]
        self.loc_logger.log_values(values_to_log)

        
    def getPose(self):
        return self.pose


if __name__=="__main__":
    
    init()
    
    LOCALIZER=localization()
    
    
    spin(LOCALIZER)
