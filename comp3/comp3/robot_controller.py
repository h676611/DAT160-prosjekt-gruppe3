import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from bug2_interfaces.action import GoToPoint, ExploreWall
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool

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

        # Events to coordinate asynchronous goal/result callbacks with execute_callback
        self._bug2_goal_accepted_event = threading.Event()
        self._bug2_result_event = threading.Event()
        self._bug2_goal_handle = None
        self._bug2_result = None

        self.wallfollow_cli = self.create_client(SetBool, 'wall_follow')

        while not self.wallfollow_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('wall_follow service not available, waiting...')
        self.wallfollow_req = SetBool.Request()


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
    


    # ------------------- Action client -------------------

    def execute_callback(self, goal_handle):
        self.get_logger().info("Starting action execution")
        result = ExploreWall.Result()
        wall = goal_handle.request.wall_points.points

        # Pick point in wall

        target_point = wall[len(wall)//2]
        # for pt in wall:
        #     if self.distance(pt, self.position) < self.distance(target_point, self.position):
        #         target_point = pt

        self.get_logger().info(f"Target point selected: x={target_point.x:.2f}, y={target_point.y:.2f}")

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
        self.get_logger().info(f"start_point: {start_point}")

        # Start wall following 
        self.send_request_wallfollow(True)

        # wait until robot has left the start point (use short sleeps so other threads run)
        # prefer a short-rate loop to avoid long blocking sleep
        while self.position is not None and self.distance(self.position, start_point) < 1.0:
            time.sleep(1.0)

        self.get_logger().info('Robot left start point, now following wall...')

        # wait until robot returns near start_point
        while self.position is None or self.distance(self.position, start_point) > 0.5:
            self.get_logger().info(f'distance to startpoint: {self.distance(self.position, start_point):.4f}')
            time.sleep(0.5)

        self.get_logger().info("Completed wall follow around")
        self.send_request_wallfollow(False)

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

    def send_request_wallfollow(self, data: bool):
        self.wallfollow_req.data = data
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

