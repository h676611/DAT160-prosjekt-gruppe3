import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from comp3_interfaces.action import GoToPoint, ExploreWall
from geometry_msgs.msg import Point
from comp3_interfaces.srv import Wallfollow
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA

import time
import threading

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor


# test flytt til ny controller
from nav_msgs.msg import Odometry
import math

class RobotController(Node):
    def __init__(self):
        Node.__init__(self, node_name="robot_controller")
        self._action_client = ActionClient(self, GoToPoint, 'gotopoint')

        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.start_marker_id = 0  # unique id for this marker

        # Events to coordinate asynchronous goal/result callbacks with execute_callback
        self._bug2_goal_accepted_event = threading.Event()
        self._bug2_result_event = threading.Event()
        self._bug2_goal_handle = None
        self._bug2_result = None

        self.wallfollow_cli = self.create_client(Wallfollow, 'wall_follow')

        while not self.wallfollow_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('wall_follow service not available, waiting...')
        self.wallfollow_req = Wallfollow.Request()


        self._action_server = ActionServer(
            self,
            ExploreWall,
            'explore_wall',
            execute_callback = self.execute_callback,
            cancel_callback = self.cancel_callback,
            goal_callback=self.goal_callback)



        # test flytt til en annen controller
        self.position = None
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.clbk_odom, 10)


        self.wall_points = []
        self.target_pos = Point()

        # Define the future variable for the goal sent to the Bug2 action
        self._send_goal_future = None


    # test flytt til ny controller

    def clbk_odom(self, msg):
        self.position = msg.pose.pose.position
        # self.get_logger().info(f"clbk pos: {self.position}")

    def distance(self, point1, point2):
        dx = point1.x - point2.x
        dy = point1.y - point2.y
        return math.sqrt(dx**2 + dy**2)
    

    def publish_start_marker(self, point: Point):
        marker = Marker()
        marker.header.frame_id = 'map'  # or 'odom', depending on your frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'start_point'
        marker.id = self.start_marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point
        marker.pose.orientation.w = 1.0  # neutral orientation
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # green
        marker.lifetime.sec = 0  # 0 = forever
        self.marker_pub.publish(marker)

    


    # ------------------- Action client -------------------

    def execute_callback(self, goal_handle):
        self.get_logger().info("Starting action execution")

        # Reset events for this new goal
        self._bug2_goal_accepted_event.clear()
        self._bug2_result_event.clear()
        self._bug2_goal_handle = None
        self._bug2_result = None

        result = ExploreWall.Result()
        wall = goal_handle.request.wall_points.points
        follow_right = goal_handle.request.follow_right

        # Wait for initial position from odometry
        timeout = 5.0
        start_time = time.time()
        while self.position is None:
            if time.time() - start_time > timeout:
                self.get_logger().error('Timeout waiting for initial odometry position')
                result.success = False
                goal_handle.abort()
                return result
            time.sleep(0.1)
        

        # Pick point in wall
        target_point = wall[0]
        for pt in wall:
            if self.distance(pt, self.position) < self.distance(target_point, self.position):
                target_point = pt

        # self.get_logger().info(f"Target point selected: x={target_point.x:.2f}, y={target_point.y:.2f}")

        # Send goal point to bug2 
        
        self.send_goal(target_point)

        # Wait for bug2 goal acceptance via event (non-blocking for other threads)
        if not self._bug2_goal_accepted_event.wait(timeout=5.0):
            self.get_logger().info("Bug2 goal response timeout.")
            result.success = False
            return result

        if not self._bug2_goal_handle.accepted:
            self.get_logger().info("Bug2 goal rejected.")
            result.success = False
            return result

        self.get_logger().info("Bug2 goal accepted. Waiting for result...")

        # Wait for the actual result via event
        self._bug2_result_event.wait()  # no timeout: block here but other callbacks run in other threads

        self.get_logger().info("Bug2 navigation completed successfully.")

        start_point = self.position
        # self.get_logger().info(f"start_point: {start_point}")
        self.publish_start_marker(start_point)


        # Start wall following 
        self.send_request_wallfollow(activate=True, follow_right=follow_right)

        # wait until robot has left the start point (use short sleeps so other threads run)
        # prefer a short-rate loop to avoid long blocking sleep
        while self.position is not None and self.distance(self.position, start_point) < 1.0:
            time.sleep(1.0)

        self.get_logger().info('Robot left start point, now following wall...')

        # wait until robot returns near start_point
        while self.position is None or self.distance(self.position, start_point) > 0.5:
            # self.get_logger().info(f'distance to startpoint: {self.distance(self.position, start_point):.4f}')
            time.sleep(0.5)

        
        
        self.get_logger().info("Completed wall follow around")
        
        self.send_request_wallfollow(activate=False)

        goal_handle.succeed()
        self.get_logger().info("Goal completed successfully")
        return result


    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""

        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""

        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT


    # ------------------- Service client ------------------

    def send_request_wallfollow(self, activate, follow_right = False):
        self.wallfollow_req.activate = activate
        self.wallfollow_req.follow_right = follow_right
        future = self.wallfollow_cli.call_async(self.wallfollow_req)
        future.add_done_callback(lambda fut: None)
        return future


    # ------------------- Action server -------------------
    
    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

        rclpy.shutdown()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        self._bug2_goal_handle = goal_handle
        # signal execute_callback that response arrived
        self._bug2_goal_accepted_event.set()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self._goal_handle = goal_handle
        self.get_logger().info('Goal accepted')

        # Wait for the result asynchronously
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self._bug2_result = result
        self.get_logger().info(
            f'Goal finished! Final position: ({result.base_position.x:.2f}, {result.base_position.y:.2f})'
        )
        # signal execute_callback that result arrived
        self._bug2_result_event.set()



    def feedback_callback(self, feedback_msg):
        return



    def send_goal(self, point: Point):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = GoToPoint.Goal()

        goal_msg.target_position = point

        self.get_logger().info(f'Sending goal request for: {point.x:.2f}, {point.y:.2f}')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)


def main(args=None):
    rclpy.init(args=args)

    action_client = RobotController()

    # Use a multi-threaded executor (at least 2 threads) so clbk_odom runs while execute_callback waits.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(action_client)
    try:
        executor.spin()
    finally:
        executor.remove_node(action_client)
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

