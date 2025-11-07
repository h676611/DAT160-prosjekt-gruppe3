import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from bug2_interfaces.action import GoToPoint, ExploreWall
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool

import time

from rclpy.action import ActionServer, CancelResponse, GoalResponse
import time
from rclpy.executors import MultiThreadedExecutor


# test flytt til ny controller
from nav_msgs.msg import Odometry
import math

class RobotController(Node):
    def __init__(self):
        Node.__init__(self, node_name="robot_controller")
        self._action_client = ActionClient(self, GoToPoint, 'gotopoint')

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
        target_point = wall[0]
        # Send goal point to bug2 
        self.send_goal(target_point)

        # Wait for bug2 to finish 
        
        # Wait until the goal is accepted
        rclpy.spin_until_future_complete(self, self._send_goal_future)
        bug2_goal_handle  = self._send_goal_future.result()

        if not bug2_goal_handle.accepted:
            self.get_logger().info("Bug2 goal rejected.")
            result.success = False
            bug2_goal_handle.abort()
            return result

        self.get_logger().info("Bug2 goal accepted. Waiting for result...")

        # Wait for the actual result (i.e., when Bug2Controller reaches goal)
        get_result_future = bug2_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)


        self.get_logger().info("Bug2 navigation completed successfully.")

        start_point = self.position
        self.get_logger().info(f"start_point: {start_point}")

        # Start wall following 
        self.send_request_wallfollow(True)

        time.sleep(10)
        self.get_logger().info(f'distance to startpoint: {self.distance(self.position, start_point):.4f}')

        while (self.distance(self.position, start_point) > 0.5):
            self.get_logger().info(f'distance to startpoint: {self.distance(self.position, start_point):.4f}')
            time.sleep(0.5)

        self.get_logger().info("Completed wall follow around")
        self.send_request_wallfollow(False)
        

        # # Publish feedback (you may want to update current_position if needed)
        # feedback = ExploreWall.Feedback()
        # goal_handle.publish_feedback(feedback)

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
        self.get_logger().info(
            f'Goal finished! Final position: ({result.base_position.x:.2f}, {result.base_position.y:.2f})'
        )



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

    rclpy.spin(action_client)




if __name__ == '__main__':
    main()
        
