import sys

from utilities import Logger, euler_from_quaternion
from rclpy.time import Time
from rclpy.node import Node

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from rclpy import init, spin
import rclpy.qos as Qos
import rclpy

rawSensor = 0
class localization(Node):
    
    def __init__(self, localizationType=rawSensor):

        super().__init__("localizer")
        
        # TODO Part 3: Define the QoS profile variable based on whether you are using the simulation (Turtlebot 3 Burger) or the real robot (Turtlebot 4)
        # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
        
        odom_qos=QoSProfile(reliability=Qos.ReliabilityPolicy.RELIABLE, durability=Qos.DurabilityPolicy.VOLATILE, history=Qos.HistoryPolicy.UNKNOWN, depth=10) # For simulation
        #odom_qos=QoSProfile(reliability=1, durability=2, history=1, depth=10) # For real robot. Update once in lab.
        
        self.loc_logger=Logger("robot_pose.csv", ["x", "y", "theta", "stamp"])
        self.pose=None
        
        if localizationType == rawSensor:
        # TODO Part 3: subscribe to the position sensor topic (Odometry)
         self.create_subscription(odom, '/odom', self.odom_callback, odom_qos)

        else:
            print("This type doesn't exist", sys.stderr)
    
    
    def odom_callback(self, pose_msg):
        # TODO Part 3: Read x,y, theta, and record the stamp

        timestamp = Time.from_msg(pose_msg.header.stamp).nanoseconds
        position = pose_msg.pose.pose.position
        orientation = pose_msg.pose.pose.orientation
        yaw = euler_from_quaternion(orientation)

        self.pose=[position.x, position.y, yaw, timestamp]
        
        # Log the data
        self.loc_logger.log_values([self.pose[0], self.pose[1], self.pose[2], Time.from_msg(self.pose[3]).nanoseconds])
    
    def getPose(self):
        return self.pose
    
def main(args=None):
    rclpy.init(args=args)
    node=localization(rawSensor) # Create node.
    spin(node) # Spin the node to keep it alive and processing callbacks.
    node.destroy_node() # Destroy the node explicitly.
    rclpy.shutdown()

# TODO Part 3
# Here put a guard that makes the node run, ONLY when run as a main thread!
# This is to make sure this node functions right before using it in decision.py
if __name__ == "__main__":
    main()
    
