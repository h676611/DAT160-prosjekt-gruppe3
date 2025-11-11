import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from visualization_msgs.msg import Marker
import numpy as np
from scipy.ndimage import label
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import ColorRGBA
from typing import List

from bug2_interfaces.srv import SetWallpoints
from bug2_interfaces.msg import PointArray


class MapFilterClass(Node):
    def __init__(self):
        super().__init__('wall_classifier_node')

        self.map_msg = OccupancyGrid()
        self.mask_size = 2*2

        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                        durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
                        depth=5,
        )

        self.create_subscription(OccupancyGrid, '/map', callback=self.clbk_map, qos_profile=qos_profile)
        
        self.pub_filtered_map = self.create_publisher(OccupancyGrid, '/filtered_map', 10)

        self.pub_marker = self.create_publisher(Marker, '/marker_visual', 2)
        # self.cluster_pub = self.create_publisher(PoseArray, '/wall_points', 10)


        self.map_received = False
        self.srv = self.create_service(SetWallpoints, '/wallpoints', self.clbk_wallpoints)

        self.marker_msg = Marker()
        #Defines the transformation frame with which the following data is associated
        self.marker_msg.header.frame_id = "/map"
        #Defines the current time in ros time
        self.marker_msg.header.stamp = self.get_clock().now().to_msg()
        #Assign a unique marker id
        self.marker_msg.id = 0
        #Define the type of oject that is displayed
        self.marker_msg.type = Marker.POINTS
        #Define the action that is taken
        self.marker_msg.action = Marker.ADD
        #Define part of the orientation of the object displayed in rviz
        self.marker_msg.pose.orientation.w =1.0
        # Defines the size of the marker (in meters) displayed in rviz
        self.marker_msg.scale.x=0.1
        self.marker_msg.scale.y=0.1
        # Define the color (red, green and blue from 0-1) and the opacity (alpha from 0-1)
        self.marker_msg.color.r = 0.0/255.0
        self.marker_msg.color.g = 255.0/255.0
        self.marker_msg.color.b = 0.0/255.0
        self.marker_msg.color.a = 1.0
        #Define how long the object should last before being automatically deleted, where 0 indicates forever
        self.marker_msg.lifetime = rclpy.duration.Duration().to_msg()

        self.posted = False
        self.og_map_msg = None

        self.cluster_points = []

        # timer_period = 7.0  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)        


    def clbk_wallpoints(self, request, response):
        if not self.map_received:
            self.get_logger().warn('Wallpoints requested but no map received yet')
            response.success = False
            return response

        self.cluster_points = self.find_wall_clusters(self.map_msg)
        response.points = self.cluster_points
        response.success = True
        return response
    


    def clbk_map(self, msg):
        self.map_received = True
        self.get_logger().info('Map received.')

        if self.map_msg.header == msg.header:
            return
        self.og_map_msg = msg


        self.get_logger().info('Map Size: '+str(len(msg.data)))
        self.get_logger().info('Origin Position: '+str(msg.info.origin.position))

        self.map_msg.header = msg.header
        self.map_msg.header.frame_id = 'map'
        self.map_msg.info.origin = msg.info.origin
        self.map_msg.info.map_load_time = msg.info.map_load_time

        self.reduce_map()


    def get_map_pos(self, map_iter):
        """
        Returns x and y integer values of the map grid given the iteration value of the map list.
        """
        x = int(map_iter/self.map_msg.info.width)
        y = int(map_iter - x*self.map_msg.info.width)

        return [x, y]
    
    def get_world_pos(self, x, y) -> Point:
        """
        Given the x and y map grid values returns the actual position coordinates in the gazebo world.
        """
        map_position = Point()
        map_position.x = self.map_msg.info.origin.position.x + y*self.map_msg.info.resolution
        map_position.y = self.map_msg.info.origin.position.y + x*self.map_msg.info.resolution

        return map_position
    
    def get_map_iter(self, x, y):
        """
        Given the x and y grid values of the map returns the corresponding map list iteration.
        """
        map_iter = x*self.og_map_msg.info.width + y

        return map_iter
    
    def avg_mask(self, x, y, mask_size):
        """
        Calculates the average value of a square around the map grid position defined by x and y.
        The mask_size defines the size of the square. The calculated mean is then thresholded to
        be either 0 or 100.
        """
        max = int(math.sqrt(mask_size)/2)
        value_sum = 0
        counter = 0
        for cur_x in range(int(x-max),int(x+max),1):
            for cur_y in range(int(y-max),int(y+max),1):
                if cur_y < self.og_map_msg.info.width:
                    if cur_x < self.og_map_msg.info.height:
                        value_sum += self.og_map_msg.data[self.get_map_iter(cur_x, cur_y)]
                        counter +=1

        value_avg = value_sum/counter
        if value_avg >= 50:
            value_final = 100
        else:
            value_final = 0

        return value_final

    def reduce_map(self):
        """
        Creates a new map with a reduced resolution. This is done by combining multiple map values 
        into one by calculating the average value of them and thresholding them to either 100 or 0.
        How much the size of the map is reduced is dependent on self.mask_size which defines
        how many values are used for one value in the new map.
        """
        x=int(math.sqrt(self.mask_size))
        y=int(math.sqrt(self.mask_size))
        self.map = []
        height = 1
        width = 1
        y_counter = 1

        while self.get_map_iter(x,y) < len(self.og_map_msg.data):
            new_value = self.avg_mask(x,y,self.mask_size)
            self.map.append(new_value)

            y += math.sqrt(self.mask_size)
            y_counter += 1

            if y >= self.og_map_msg.info.width:
                width = y_counter-1
                y_counter = 1
                y = int(math.sqrt(self.mask_size))
                x += math.sqrt(self.mask_size)
                height += 1

        self.map_msg.info.width = width
        self.map_msg.info.height = height-1
        #Set resolution of new map
        self.map_msg.info.resolution = self.og_map_msg.info.resolution * math.sqrt(self.mask_size)

        #Print out new size of the map
        self.get_logger().info('New Map Size: '+str(len(self.map)))
        self.get_logger().info(f"width={width}, height={height-1}, len(data)={len(self.map)}")


        self.map_msg.data = self.map



    def find_wall_clusters(self, grid_msg: OccupancyGrid) -> List[PointArray]:
        data = self.grid_to_matrix(grid_msg)
        wall_mask = (data == 100).astype(np.uint8)

        structure = np.ones((3, 3), dtype=int)
        labeled, num_features = label(wall_mask, structure=structure)

        self.get_logger().info(f"Found {num_features} wall clusters.")

        cluster_points = []
        for i in range(1, num_features + 1):
            accepted_points = []
            coords = np.argwhere(labeled == i)
            if len(coords) == 0:
                continue

            threshold = 1.0

            for j in range(len(coords)):
                row, col = coords[j]
                world_point = self.get_world_pos(row, col)

                # Always accept the first point
                if not accepted_points:
                    accepted_points.append(world_point)
                    continue

                # Check distance to all previously accepted points
                too_close = False
                for p in accepted_points:
                    dist = np.hypot(world_point.x - p.x, world_point.y - p.y)
                    if dist < threshold:
                        too_close = True
                        break

                if not too_close:
                    accepted_points.append(world_point)
            point_array = PointArray()
            point_array.points = accepted_points
            cluster_points.append(point_array)

        return cluster_points



    def is_near_wall(self, data: np.ndarray, row: int, col: int, radius: int = 3) -> bool:
        """
        Checks if there is any occupied cell (value 100) within a square neighborhood
        centered at (row, col) with a given radius.
        """
        height, width = data.shape
        r_min = max(0, row - radius)
        r_max = min(height, row + radius + 1)
        c_min = max(0, col - radius)
        c_max = min(width, col + radius + 1)
        region = data[r_min:r_max, c_min:c_max]
        return np.any(region == 100)

    

    def grid_to_matrix(self, grid_msg: OccupancyGrid):
        """
        Converts an OccupancyGrid message to a 2D numpy array.
        Each cell corresponds to grid_msg.data[row * width + col].
        """
        width = grid_msg.info.width
        height = grid_msg.info.height
        data = np.array(grid_msg.data, dtype=np.int8).reshape((height, width))
        return data

    def matrix_to_grid(self, matrix: np.ndarray, template_msg: OccupancyGrid):
        """
        Converts a 2D numpy array back to an OccupancyGrid message.
        Uses 'template_msg' to copy metadata (resolution, origin, etc.).
        """
        grid_msg = OccupancyGrid()
        grid_msg.header = template_msg.header
        grid_msg.info = template_msg.info
        grid_msg.data = matrix.flatten().tolist()
        return grid_msg


    # def timer_callback(self):

    #     if not hasattr(self, 'og_map_msg'):
    #         return  # Wait until a map is received
    #     if len(self.map_msg.data) == 0:
    #         return  # Skip if no filtered map yet
        
        
    #     self.cluster_points = self.find_wall_clusters(self.map_msg)
        

    #     points_msg = PoseArray()
    #     points_msg.header.frame_id = "map"

    #     for point in self.cluster_points:
    #         pose = Pose()
    #         pose.position = point
    #         pose.orientation.w = 1.0  # identity orientation
    #         points_msg.poses.append(pose)

    #     self.cluster_pub.publish(points_msg)

        
    #     # self.map_msg.header.stamp = self.get_clock().now().to_msg()
    #     # self.map_msg.info.map_load_time = self.get_clock().now().to_msg()

    #     # self.pub_filtered_map.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)

    map_filter = MapFilterClass()

    rclpy.spin(map_filter)

    map_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()