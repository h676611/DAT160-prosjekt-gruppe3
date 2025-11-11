import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from bug2_interfaces.action import ExploreWall
from bug2_interfaces.srv import SetWallpoints 
from bug2_interfaces.msg import PointArray
from geometry_msgs.msg import Point, PointStamped
from typing import List
import time

class LeaderClass(Node):
    def __init__(self):
        super().__init__('leader')

        self.robot_namespaces = ['tb3_0', 'tb3_1']



        self.action_clients = {
            ns: ActionClient(self, ExploreWall, f'/{ns}/explore_wall')
            for ns in self.robot_namespaces
        }

        self.wall_segments_cli = self.create_client(SetWallpoints, '/wallpoints')

        self.click_sub = self.create_subscription(PointStamped, '/clicked_point', self.point_clbck, 10)

        self.wall_segments = []


        while not self.wall_segments_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('wallpoints service not available, waiting...')
        self.wall_segments_req = SetWallpoints.Request()

        self.send_request_wallpoints()

        self.send_goals_to_robots()

    
    def send_goals_to_robots(self):
        # self.wall_segments is a list of PointArray objects
        if len(self.wall_segments) >= 2:
            self.send_goal(self.wall_segments[0], 'tb3_0')
            self.send_goal(self.wall_segments[1], 'tb3_1')
        else:
            self.get_logger().error(f'Expected 2 wall segments, got {len(self.wall_segments)}')

    def send_request_wallpoints(self):
        max_retries = 10
        retry_delay = 1.0  # seconds
        
        for attempt in range(max_retries):
            future = self.wall_segments_cli.call_async(self.wall_segments_req)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            
            if response is not None and response.success:
                self.wall_segments = response.points
                self.get_logger().info(f'Received {len(self.wall_segments)} wall segments on attempt {attempt + 1}')
                return  # success, exit
            else:
                self.get_logger().warn(
                    f'Wallpoints service returned success=False (attempt {attempt + 1}/{max_retries}). '
                    f'Retrying in {retry_delay}s...'
                )
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                else:
                    self.get_logger().error('Failed to get wallpoints after all retries')


    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

        rclpy.shutdown()

    def point_clbck(self, msg):
        new_target = msg.point
        self.get_logger().info(f"New goal received: x={new_target.x:.2f}, y={new_target.y:.2f}")

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
        self.get_logger().info('Goal completed')
        # result = future.result().result
        # rclpy.shutdown()


    def feedback_callback(self, feedback_msg):
        return


    def send_goal(self, wall: PointArray, robot_ns: str):
        action_client = self.action_clients[robot_ns]

        self.get_logger().info(f'Waiting for action server in {robot_ns}...')
        action_client.wait_for_server()

        goal_msg = ExploreWall.Goal()
        goal_msg.wall_points = wall
        goal_msg.wall_id = 0

        self.get_logger().info(f'Sending goal to {robot_ns}...')
        send_future = action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_future.add_done_callback(self.goal_response_callback)


def main(args=None):
    rclpy.init(args=args)

    robot_leader = LeaderClass()

    rclpy.spin(robot_leader)  # This will now handle sending the goal after segments are received

    robot_leader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
