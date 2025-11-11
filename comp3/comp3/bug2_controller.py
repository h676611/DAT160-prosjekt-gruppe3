import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from bug2_interfaces.srv import Switch 
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
import math
import numpy as np

from bug2_interfaces.action import GoToPoint
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import time
from rclpy.executors import MultiThreadedExecutor

from visualization_msgs.msg import Marker

class Bug2Controller(Node):
    def __init__(self):
        Node.__init__(self, node_name="bug2_controller")

        # Action server
        self._action_server = ActionServer(
            self,
            GoToPoint,
            'gotopoint',
            execute_callback = self.execute_callback,
            cancel_callback = self.cancel_callback,
            goal_callback=self.goal_callback)


        # Service clients
        self.gotopoint_cli = self.create_client(Switch, 'go_to_point')
        self.wallfollow_cli = self.create_client(SetBool, 'wall_follow')
 
        # Subscribers
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.clbk_odom, 10)
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.clbk_laser, 10)

        self.pub_marker = self.create_publisher(Marker, 'goal_marker', 2)
        # Laser readings
        self.front = 100.0
        self.right = 100.0
        self.left = 100.0

        # Robot state
        self.position = Point()
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.hitPoint = None
        self.isWallFollowActive = False
        self.mline_start = Point()
        self.goal = Point()

        self.atGoal = False

        self.marker_msg = Marker()


        # Wait for services
        while not self.wallfollow_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('wall_follow service not available, waiting...')
        self.wallfollow_req = SetBool.Request()

        while not self.gotopoint_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('go_to_point service not available, waiting...')
        self.gotopoint_req = Switch.Request()



    # ------------------- Callbacks -------------------

    def execute_callback(self, goal_handle):
        self.get_logger().info("Starting action execution")
        result = GoToPoint.Result()

        self.goal = goal_handle.request.target_position
        self.get_logger().info(f"Goal: x={self.goal.x:.2f}, y={self.goal.y:.2f}")
        self.create_goal_marker()
        self.pub_marker.publish(self.marker_msg)

        self.mline_start = self.position
        
        self.isWallFollowActive = True
        self.switch_to_gotopoint()

        while not self.atGoal:
            self.timer_callback() 
            
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Cancel requested! Stopping execution...')
                self.stop_robot()
                goal_handle.canceled()
                return result 

            # Publish feedback
            feedback = GoToPoint.Feedback()
            feedback.current_position = self.position
            goal_handle.publish_feedback(feedback)
            time.sleep(0.01)

        goal_handle.succeed()
        result.base_position = self.position
        self.get_logger().info("Goal completed successfully")
        self.stop_robot()
        return result


    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""

        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT


    def clbk_laser(self, msg):
        values = np.concatenate((msg.ranges[350:], msg.ranges[:11]))  # 350–364 + 0–10
        max_val = np.min(values)
        self.front = max_val 
        self.right = msg.ranges[315]
        self.left = msg.ranges[45]

    def clbk_odom(self, msg):
        self.position = msg.pose.pose.position
        self.x = self.position.x
        self.y = self.position.y
        self.z = self.position.z

    # ------------------- Timer callback -------------------
    def timer_callback(self):

        if self.goal == None:
            self.get_logger().info("Waiting for goal point")
            return
        
        if self.distance(self.position, self.goal) < 0.5:
            self.atGoal = True
            return

        if self.isWallFollowActive:
            if self.isOnMline() and self.hitPoint is not None and  self.isCloserToGoal(self.position, self.hitPoint):
                self.switch_to_gotopoint()
        else:
            if self.front < 1.0:
                self.hitPoint = self.position
                self.switch_to_wallfollow()
            else:
                self.switch_to_gotopoint()

    # ------------------- Helper methods -------------------

    def stop_robot(self):
        self.send_request_gotopoint(False, self.goal)
        self.send_request_wallfollow(False)

    def switch_to_gotopoint(self):
        """
        Switches to gotopoint if wallfollowing is active
        
        """
        if self.isWallFollowActive:
            self.stop_robot()
            self.isWallFollowActive = False
            self.send_request_wallfollow(False)
            self.send_request_gotopoint(True, self.goal)
            self.get_logger().info("Switching to gotopoint")

    def switch_to_wallfollow(self):
        """
        Switches to wallfollowing if it isn't active

        """
        if not self.isWallFollowActive:
            self.stop_robot()
            self.isWallFollowActive = True
            self.send_request_gotopoint(False, self.goal)
            self.send_request_wallfollow(True)
            self.get_logger().info("Switching to wall-following")

    def isOnMline(self):
        if self.x is None or self.y is None:
            return False
        
        threshold = 0.20

        if self.goal.x == self.mline_start.x:
            return abs(self.x - self.mline_start.x) <= threshold
        
        m = (self.goal.y - self.mline_start.y) / (self.goal.x - self.mline_start.x)
        expected_y = self.mline_start.y + m * (self.x - self.mline_start.x)
        
        return abs(self.y - expected_y) <= threshold

    def isCloserToGoal(self, point1, point2):
        dx1 = self.goal.x - point1.x
        dy1 = self.goal.y - point1.y
        d1 = math.sqrt(dx1**2 + dy1**2)

        dx2 = self.goal.x - point2.x
        dy2 = self.goal.y - point2.y
        d2 = math.sqrt(dx2**2 + dy2**2)

        return (d1 + 0.04) < d2

    def distance(self, point1, point2):
        dx = point1.x - point2.x
        dy = point1.y - point2.y
        return math.sqrt(dx**2 + dy**2)

    def send_request_gotopoint(self, move_switch, target_position):
        self.gotopoint_req.move_switch = move_switch
        self.gotopoint_req.target_position = target_position
        future = self.gotopoint_cli.call_async(self.gotopoint_req)
        future.add_done_callback(lambda fut: None)
        return future

    def send_request_wallfollow(self, data):
        self.wallfollow_req.data = data
        future = self.wallfollow_cli.call_async(self.wallfollow_req)
        future.add_done_callback(lambda fut: None)
        return future
    
    def create_goal_marker(self):
        self.get_logger().info('Creating goal marker')
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
        self.marker_msg.color.r = 255.0/255.0
        self.marker_msg.color.g = 0.0/255.0
        self.marker_msg.color.b = 0.0/255.0
        self.marker_msg.color.a = 1.0
        #Define how long the object should last before being automatically deleted, where 0 indicates forever
        self.marker_msg.lifetime = rclpy.duration.Duration().to_msg()

        self.marker_msg.points.clear()
        self.marker_msg.points.append(self.goal)
        self.pub_marker.publish(self.marker_msg)

# ------------------- Main -------------------
def main(args=None):

    rclpy.init(args=args)
    controller = Bug2Controller()

    # rclpy.spin(controller)

    executor = MultiThreadedExecutor()
    rclpy.spin(controller, executor=executor)
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
