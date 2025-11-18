import matplotlib.pyplot as plt
import numpy as np 

from sensor_msgs.msg import LaserScan
from math import floor
import math

from math import pi as M_PI
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PointStamped, Quaternion, Point
from utilities import *

class mapManipulator(Node):


    def __init__(self, filename_: str = "room.yaml", laser_sig=0.1):
        
        
        super().__init__('likelihood_field')
        
        filenameYaml=None
        filenamePGM=None
        if ".pgm" in filename_:
            
            filenamePGM=filename_
            filenameYaml=filename_.replace(".pgm", ".yaml")
            
        elif ".yaml" in filename_:
            
            filenameYaml=filename_
            filenamePGM=filename_.replace(".yaml", ".pgm")
        
        else:
            filenameYaml=filename_ + ".yaml"
            filenamePGM=filename_+".pgm"

        self.laser_sig = laser_sig
        self.o_x, self.o_y, self.res, self.thresh = self.read_description(filenameYaml)
        width, height, max_value, pixels = self.read_pgm(filenamePGM)

        self.image_array = np.array(pixels).reshape((height, width))
        self.image_array = self.expand_image(self.image_array)

        self.height, self.width = self.image_array.shape
        
        self.likelihood_msg=None


    def expand_image(self, image_array):
        """
        Expand the image array to avoid the edge effect
        image_array: 2D array of the image
        return: 2D array of the expanded image
        """
        num_pixel_to_expand = np.max([5, floor(5*self.laser_sig/self.res)])
        expanded_image = np.ones((image_array.shape[0] + 2*num_pixel_to_expand, image_array.shape[1] + 2*num_pixel_to_expand)) * 255
        expanded_image[num_pixel_to_expand:-num_pixel_to_expand, num_pixel_to_expand:-num_pixel_to_expand] = image_array
        # shift the origin
        self.o_x -= num_pixel_to_expand * self.res
        self.o_y -= num_pixel_to_expand * self.res
        return expanded_image
        
    def getAllObstacles(self):
        image_array=self.image_array.T
        indices = np.where(image_array < 10)
        indices_arr = np.array([indices[0], indices[1]]).T
        return self.cell_2_position(indices_arr)
        

    def getLikelihoodField(self):
        return self.likelihood_field
    
    def getMetaData(self):
        return self.o_x, self.o_y, self.res, self.thresh
            
    def getMap(self):
        return self.image_array
        
    def read_pgm(self, filename):
        with open(filename, 'rb') as f:
            # Check if it's a PGM file
            header = f.readline().decode().strip()
            
            if header != 'P5':
                raise ValueError('Invalid PGM file format')

            # Skip comments
            line = f.readline().decode().strip()
            while line.startswith('#'):
                line = f.readline().decode().strip()

            # Read width, height, and maximum gray value
            width, height = map(int, line.split())
            max_value = int(f.readline().decode().strip())

            # Read the image data
            image_data = f.read()

        # Convert image data to a list of pixel values
        pixels = [x for x in image_data]

        return width, height, max_value, pixels

    def plot_pgm_image(self, image_array):
        # Plot the image
        plt.imshow(image_array, cmap='gray')
        plt.axis('off')
        plt.title('PGM Image')
        plt.show()



    def read_description(self, filenameYAML):
        import re

        # Open and read the YAML file
        with open(filenameYAML, 'r') as file:
            yaml_content = file.readlines()

            # Extract the desired fields
            threshold = None
            origin_x = None
            origin_y = None
            resolution = None

            for line in yaml_content:
                if 'occupied_thresh' in line:
                    threshold = float(re.findall(r'\d+\.\d+', line)[0])
                elif 'origin' in line:
                    origin_values = re.findall(r'-?\d+\.\d+', line)
                    origin_x = float(origin_values[0])
                    origin_y = float(origin_values[1])
                elif 'resolution' in line:
                    resolution = float(re.findall(r'\d+\.\d+', line)[0])
        return origin_x, origin_y, resolution, threshold


    def getOrigin(self):
        return np.array([self.o_x, self.o_y])
    
    def getResolution(self):
        return self.res
    
    def cell_2_position(self, pix_array):
        """
        Convert a cell index to a position in the map
        pix_array: Nx2 array of cell indices
        return: Nx2 array of positions
        """
        origin = self.getOrigin()
        res = self.getResolution()
        h = self.height
        return (pix_array * np.array([1, -1]) + np.array([0, h])) * res + origin
    
    
    def position_2_cell(self, pos_array):
        """
        Convert a position in the map to a cell index
        pos_array: Nx2 array of positions
        return: Nx2 array of cell indices
        """
        origin = self.getOrigin()
        res = self.getResolution()
        h = self.height
        return (np.array(np.floor((-origin + pos_array)/res) * np.array([1, -1]) + np.array([0, h]), dtype=np.int32))


    def make_likelihood_field(self):
        
        image_array=self.image_array

        from sklearn.neighbors import KDTree
        
        indices = np.where(image_array < 10)
        indices_arr = np.array([indices[0], indices[1]]).T
        
        occupied_points = self.cell_2_position(indices_arr)
        all_indices = np.array([[i, j] for i in range(self.height) for j in range(self.width)])
        all_positions = self.cell_2_position(all_indices)

        kdt=KDTree(occupied_points)

        dists=kdt.query(all_positions, k=1)[0][:]
        probabilities=np.exp( -(dists**2) / (2*self.laser_sig**2))
        
        likelihood_field=probabilities.reshape(image_array.shape)
        
        likelihood_field_img=np.array(255-255*probabilities.reshape(image_array.shape), dtype=np.int32)
        
        self.likelihood_img=likelihood_field_img
        
        self.occ_points=np.array(occupied_points)
        
                
        #self.plot_pgm_image(likelihood_field_img)

        self.likelihood_field = likelihood_field
        
        return likelihood_field
                
    
    def to_message(self):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"

        likelihoodField = self.getLikelihoodField()

        grid.info.resolution = self.getResolution()  # Set the resolution (m/cell)
        grid.info.width = self.height
        grid.info.height = self.width
        grid.info.origin = Pose()  # Set the origin of the map (geometry_msgs/Pose)
        
        grid.info.origin.orientation.w = np.cos(-np.pi/4)
        grid.info.origin.orientation.z = np.sin(-np.pi/4)
        offset = -self.height*self.getResolution()


        grid.info.origin.position.x, grid.info.origin.position.y = self.getOrigin()[0], +self.getOrigin()[1] - offset


        # Flatten the likelihood field and scale it to [0, 100], set unknown as -1
        normalized_likelihood = np.clip(likelihoodField.T * 100, 0, 100)
        
        # Convert to integers and flatten
        grid.data = [int(value) for value in normalized_likelihood.flatten()]
        grid.data = list(grid.data)

        return grid
     



import argparse
if __name__=="__main__":
    


    rclpy.init()

    parser=argparse.ArgumentParser()
    parser.add_argument('--map', type=str, default="./your_map/room.yaml", help='the absolute path to argument')
    parser.add_argument('--std', type=float, help='the std', default=0.01)


    args = parser.parse_args()

    MAP_UTILITIS=mapManipulator(args.map, args.std)

    #rclpy.spin(MAP_UTILITIS)


# Usage example

