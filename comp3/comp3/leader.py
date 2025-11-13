import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from comp3_interfaces.action import ExploreWall
from comp3_interfaces.srv import SetWallpoints 
from comp3_interfaces.msg import PointArray
import time
from comp3_interfaces.srv import SetID4pos
from comp3_interfaces.action import GoToPoint
from geometry_msgs.msg import Point
import math

class LeaderClass(Node):
    def __init__(self):
        super().__init__('leader')

        self.robot_namespaces = ['tb3_0', 'tb3_1', 'tb3_2', 'tb3_3']
        self.backup_namespaces = ['tb3_2', 'tb3_3']

        # ExploreWall action clients
        self.action_clients = {
            ns: ActionClient(self, ExploreWall, f'/{ns}/explore_wall')
            for ns in self.robot_namespaces
        }

        # GoToPoint action clients for backup robots
        self.gotopoint_action_clients = {
            ns: ActionClient(self, GoToPoint, f'/{ns}/gotopoint')
            for ns in self.backup_namespaces
        }

        self.ID4pos_service = self.create_service(SetID4pos,'set_id_4_pos', self.clbk_set_id_4_pos)
        self.ID4pos = None
        self.visited_ID4 = False

        self.wall_segments_cli = self.create_client(SetWallpoints, '/wallpoints')
        self.wall_segments = []
        self.robot_wall_progress = {ns: 0 for ns in self.robot_namespaces}

        while not self.wall_segments_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('wallpoints service not available, waiting...')
        self.wall_segments_req = SetWallpoints.Request()

        if self.send_request_wallpoints():
            self.send_goal(self.wall_segments[0], 'tb3_0', follow_right=False)
            self.send_goal(self.wall_segments[1], 'tb3_1', follow_right=False)
        else:
            self.get_logger().error('Unable to fetch wall segments; goals will not be sent.')

    # ----------------- General GoToPoint -----------------
    def send_goal_gotopoint(self, point: Point, ns: str):
        client = self.gotopoint_action_clients[ns]
        while not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {ns} GoToPoint action server...')
        
        goal_msg = GoToPoint.Goal()
        goal_msg.target_position = point

        self.get_logger().info(f'Sending GoToPoint goal for {ns} -> ({point.x:.2f}, {point.y:.2f})')
        send_future = client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_future.add_done_callback(lambda fut, ns=ns: self.gotopoint_goal_response_callback(fut, ns))

    def gotopoint_goal_response_callback(self, future, ns):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f'GoToPoint goal rejected for {ns}')
            return

        self.get_logger().info(f'GoToPoint goal accepted for {ns}')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(lambda fut, ns=ns: self.gotopoint_result_callback(fut, ns))

    def gotopoint_result_callback(self, future, ns):
        result = future.result().result
        self.get_logger().info(f'{ns} reached target position (ID4)')

        self.visited_ID4 = True
        # destroy the action client to free resources
        self.gotopoint_action_clients[ns].destroy()
        



    # ----------------- SetID4pos service -----------------
    def clbk_set_id_4_pos(self, request, response):
        self.get_logger().info(f"SetID4pos request received: {request.point}")

        self.ID4pos = request.point
        response.success = True

        # Send GoToPoint goal to all backup robots
        if not self.visited_ID4:
            self.get_logger().info("Sending backup robots to ID4 position.")
            for robot_ns in self.backup_namespaces:
                self.send_goal_gotopoint(self.ID4pos, robot_ns)

        return response


    # ----------------- ExploreWall actions -----------------
    def goal_response_callback(self, future, robot_ns):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f'Goal rejected for {robot_ns}')
            return
        self.get_logger().info(f'Goal accepted for {robot_ns}')
        goal_handle.get_result_async().add_done_callback(lambda fut, ns=robot_ns: self.get_result_callback(fut, ns))

    def get_result_callback(self, future, robot_ns):
        self.get_logger().info(f'{robot_ns} finished exploring its wall.')
        current_index = self.robot_wall_progress[robot_ns]
        follow_right = True  # Always follow left wall
        next_index = 0

        if current_index == 0:
            next_index = 1
        elif current_index == 1:
            next_index = 0

        self.send_goal(self.wall_segments[next_index], robot_ns, follow_right)

    def feedback_callback(self, feedback_msg):
        pass

    def send_goal(self, wall: PointArray, robot_ns: str, follow_right: bool):
        client = self.action_clients[robot_ns]
        client.wait_for_server()
        goal_msg = ExploreWall.Goal()
        goal_msg.wall_points = wall
        goal_msg.wall_id = 0
        goal_msg.follow_right = follow_right

        wall_index = self.wall_segments.index(wall)
        self.robot_wall_progress[robot_ns] = wall_index

        self.get_logger().info(f'Sending ExploreWall goal to {robot_ns} (segment={wall_index}, follow_right={follow_right})')
        send_future = client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_future.add_done_callback(lambda fut, ns=robot_ns: self.goal_response_callback(fut, ns))


    # ----------------- Wallpoints service -----------------
    def send_request_wallpoints(self):
        max_retries = 10
        retry_delay = 1.0
        
        for attempt in range(max_retries):
            future = self.wall_segments_cli.call_async(self.wall_segments_req)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            
            if response is not None and response.points:
                self.wall_segments = list(response.points)
                self.get_logger().info(f'Received {len(self.wall_segments)} wall segments on attempt {attempt+1}')
                for i, segment in enumerate(self.wall_segments):
                    self.get_logger().info(f'Wall segment {i} size: {len(segment.points)} points')
                return True

            self.get_logger().warn(f'Wallpoints service returned no segments (attempt {attempt+1}/{max_retries}). Retrying...')
            if attempt < max_retries - 1:
                time.sleep(retry_delay)

        self.get_logger().error('Failed to get wall segments after all retries')
        return False


def main(args=None):
    rclpy.init(args=args)
    leader = LeaderClass()
    rclpy.spin(leader)
    leader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
