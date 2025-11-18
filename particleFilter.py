

import rclpy
from particle import particle
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from mapUtilities import mapManipulator
import message_filters
import numpy as np

import time

import random

from utilities import *

from rclpy.duration import Duration

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import OccupancyGrid

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time

class particleFilter(Node):

    def __init__(self, mapFilename="your_map/room.yaml", numParticles=500):

        super().__init__("particleFiltering")

        # QoS profile for the subscribers
        qos_profile_odom = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)
        qos_profile_laserScanner = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                              durability=DurabilityPolicy.VOLATILE,
                                              depth=10)
        # Create the odom and laserScan subscribers with ApproximateTimeSynchronizer
        self.odomSub = message_filters.Subscriber(self, Odometry, "/odom",
                                                  qos_profile=qos_profile_odom)
        self.laserScanSub = message_filters.Subscriber(self, LaserScan, "/scan",
                                                       qos_profile=qos_profile_laserScanner)
        self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.odomSub, self.laserScanSub], queue_size=10, slop=0.1)
        # Register the callback
        self.timeSynchronizer.registerCallback(self.filterCallback)

        # Create the initial pose subscriber (used for initializing the particle filter)
        self.initialPoseSubsriber = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.initialPose2Dcallback, 10)

        # Create the publishers
        self.particleMarkerArrayPublisher = self.create_publisher(MarkerArray, "/particles/markers", 10)
        self.mapPublisher = self.create_publisher(OccupancyGrid, "/customMap", 10)
        self.pfPosePublisher = self.create_publisher(Odometry, '/pf_pose', 10)
        self.odomPosePublisher = self.create_publisher(Odometry, '/odom_pose', 10)

        # Create the map utilities object
        # TODO: You can tune your laser_sig here
        self.mapUtilities = mapManipulator(mapFilename, laser_sig=0.1)
        self.mapUtilities.make_likelihood_field()
        self.occ_map = self.mapUtilities.to_message()
        # create a Timer to publish the map every 1 second
        self.create_timer(1.0, self.publishMap)

        # Create the transform broadcaster and listener
        self.br = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Particle Filter Variables
        self.particles = []
        self.numParticles = numParticles
        self.std_particle_x = 0.5
        self.std_particle_y = 0.5
        self.std_particle_theta = 0.5

        # Some flags and variables
        self.initialized = False
        self.championPose = None
        self.ego_odom_frame_id = None
        self.laser_to_ego_transform = None
        self.dt = 0.2  # initialize with the probable dt value
        self.tic = None
        
        print("give me your first estimate on rviz, 2D Pose Estimator")


    def publishMap(self):
        self.mapPublisher.publish(self.occ_map)
        

    def initializeParticleFilter(self, x, y, th):

        numParticles = self.numParticles

        # generate the particles around the initial pose (x, y, th) (you should use the std_particle_x, std_particle_y, std_particle_theta)
        self.particlePoses = np.random.uniform(low=[x-self.std_particle_x, y - self.std_particle_y,  th - 1.0],
                                               high=[x+self.std_particle_x, y + self.std_particle_y, th + 1.0], size=(numParticles, 3))

        self.particles = [particle(particle_, 1/numParticles) for particle_ in
                          self.particlePoses]

        self.weights = [1/numParticles] * numParticles

        self.initialized = True


    def initialPose2Dcallback(self, marker: PoseWithCovarianceStamped):

        x, y, th = marker.pose.pose.position.x, \
            marker.pose.pose.position.y, euler_from_quaternion(
                marker.pose.pose.orientation)

        self.initializeParticleFilter(x, y, th)


    def visualizeParticles(self, particles_, stamp):

        particles = MarkerArray()

        for i, particle_ in enumerate(particles_):

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = stamp

            marker.id = i
            marker.ns = "particles"
            marker.lifetime = Duration(seconds=0.3).to_msg()

            marker.type = marker.ARROW
            marker.action = marker.ADD

            weight = particle_.getWeight()
            weight = np.clip(weight, 0.01, 0.6)


            marker.scale.x = weight
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            marker.pose.position.x = particle_.getPose()[0]
            marker.pose.position.y = particle_.getPose()[1]
            marker.pose.orientation.w = np.cos(particle_.getPose()[2]/2)
            marker.pose.orientation.z = np.sin(particle_.getPose()[2]/2)

            particles.markers.append(marker)

        self.particleMarkerArrayPublisher.publish(particles)


    def normalizeWeights(self):

        sumWeight = np.sum(self.weights)
        self.weights = self.weights / sumWeight

        for particle in self.particles:
            particle.setWeight(particle.getWeight()/sumWeight)


    def resample(self, laser_scan, mapUtilInstance):
        std_noise = 0.05
        generated_particles = []

        particles_weights = np.array([each_particle.getWeight() for each_particle in self.particles])
        # print("Sum of weights: ", np.sum(particles_weights))
        particles_weights = particles_weights / np.sum(particles_weights)
        
        # randoming sampling N particles from the list of particles based on their weights
        sampled_particles = np.random.choice(
            self.particles, self.numParticles, p=particles_weights)

        for bp in sampled_particles:
            x, y, th = bp.getPose()
            # add noise to the x, y, and th, use the same std_noise for x, y, and th
            new_x = x + random.uniform(-std_noise, std_noise)
            new_y = y + random.uniform(-std_noise, std_noise)
            new_th = th + random.uniform(-std_noise, std_noise)

            new_particle = particle([new_x, new_y, new_th], bp.getWeight())

            new_particle.calculateParticleWeight(
                laser_scan, mapUtilInstance, self.laser_to_ego_transform)

            generated_particles.append(new_particle)

        self.particles = generated_particles

        return self.particles


    def filterCallback(self, odomMsg: Odometry, laserMsg: LaserScan):
        start_time = time.time()
        # check if the laser_to_ego_transform is available
        if self.laser_to_ego_transform is None:
            # get the transform from the laser to the ego robot
            try:
                self.ego_odom_frame_id = odomMsg.child_frame_id
                self.laser_to_ego_transform = self.tf_buffer.lookup_transform(
                    odomMsg.child_frame_id, laserMsg.header.frame_id, odomMsg.header.stamp)
                # create the 3x3 matrix considering x y and theta
                laser_to_ego_theta = euler_from_quaternion(
                    self.laser_to_ego_transform.transform.rotation)
                self.laser_to_ego_transform = np.array(
                    [[np.cos(laser_to_ego_theta), -np.sin(laser_to_ego_theta), self.laser_to_ego_transform.transform.translation.x],
                     [np.sin(laser_to_ego_theta), np.cos(laser_to_ego_theta), self.laser_to_ego_transform.transform.translation.y],
                     [0, 0, 1]])
                
                print("Got the transform from the {} to the {}".format(
                    laserMsg.header.frame_id, odomMsg.child_frame_id))
                print(self.laser_to_ego_transform)
            except Exception as e:
                self.get_logger().info(
                    "Waiting for the transform from the laser to the ego robot")
                return

        if not self.initialized:
            return
        
        if self.tic is not None:
            self.dt = Time.from_msg(odomMsg.header.stamp).nanoseconds / 1e9 - self.tic
        self.tic = Time.from_msg(odomMsg.header.stamp).nanoseconds / 1e9

        w = odomMsg.twist.twist.angular.z
        v = odomMsg.twist.twist.linear.x

        if self.dt > 0.3:
            self.get_logger().info("dt {:.3f} is too high, setting it to 0.3".format(self.dt))
        dt = np.min([self.dt, 0.3])
        
        for i, particle_ in enumerate(self.particles):

            particle_.motion_model(v, w, dt)
            particle_.calculateParticleWeight(laserMsg, self.mapUtilities, self.laser_to_ego_transform)

            self.weights[i] = particle_.getWeight()
            self.particlePoses[i] = particle_.getPose()

        particles_to_viz = self.resample(laserMsg, self.mapUtilities)

        weighted_average_translation = np.average(
            self.particlePoses[:, :2], axis=0, weights=self.weights)

        # For the rotation (theta), you might use a mean of circular quantities if the angles are small and don't wrap around
        mean_angle = np.arctan2(np.sum(np.sin(self.particlePoses[:, 2])*self.weights),
                                np.sum(np.cos(self.particlePoses[:, 2])*self.weights))

        weighted_average_pose = np.append(
            weighted_average_translation, mean_angle)
        stamp = odomMsg.header.stamp

        publishTransform(
            self.br, weighted_average_pose[0], weighted_average_pose[1], weighted_average_pose[2], stamp, self.ego_odom_frame_id)

        self.normalizeWeights()

        x, y, th = weighted_average_pose[0], weighted_average_pose[1], weighted_average_pose[2]

        self.championPose = [x, y, th]
        self.publishChampionPose(stamp)
        self.publishOdomPose(odomMsg)

        self.visualizeParticles(particles_to_viz, stamp)

        end_time = time.time()
        self.get_logger().info("Time taken for the filterCallback: {:.3f} seconds".format(end_time - start_time))

    def publishChampionPose(self, odom_timestamp=None):

        msg = Odometry()
        msg.header.frame_id = "map"
        if odom_timestamp is not None:
            msg.header.stamp = odom_timestamp
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.child_frame_id = "pf_pose"
        msg.pose.pose.position.x = self.championPose[0]
        msg.pose.pose.position.y = self.championPose[1]
        msg.pose.pose.orientation = quaternion_from_euler(self.championPose[2])

        self.pfPosePublisher.publish(msg)

    def publishOdomPose(self, odomMsg):
        odomMsg.header.frame_id = "map"
        self.odomPosePublisher.publish(odomMsg)

    def getChampionPose(self):
        return self.championPose


if __name__ == "__main__":

    rclpy.init()

    pf = particleFilter()

    rclpy.spin(pf)
