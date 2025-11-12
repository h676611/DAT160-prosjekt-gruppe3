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

class LeaderClass(Node):
    def __init__(self):
        super().__init__('leader')

        self.robot_namespaces = ['tb3_0', 'tb3_1']

        self.action_clients = {
            ns: ActionClient(self, ExploreWall, f'/{ns}/explore_wall')
            for ns in self.robot_namespaces
        }


        self.gotopoint_action_client = ActionClient(self, GoToPoint, '/tb3_2/gotopoint')

        self.ID4pos_service = self.create_service(SetID4pos,'set_id_4_pos', self.clbk_set_id_4_pos)

        self.ID4pos = None

        self.wall_segments_cli = self.create_client(SetWallpoints, '/wallpoints')


        self.wall_segments = []

        self.robot_wall_progress = {ns: 0 for ns in self.robot_namespaces}

        self.goal_handles = {}


        while not self.wall_segments_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('wallpoints service not available, waiting...')
        self.wall_segments_req = SetWallpoints.Request()

        if self.send_request_wallpoints():
            self.send_goals_to_robots()
        else:
            self.get_logger().error('Unable to fetch wall segments; goals will not be sent.')

    
    def send_goal_gotopoint(self, point: Point):
        while not self.gotopoint_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for tb3_2 GoToPoint action server...')
        
        goal_msg = GoToPoint.Goal()
        goal_msg.target_position = point

        self.get_logger().info(f'Sending goal request for: {point.x:.2f}, {point.y:.2f}')
        send_future = self.gotopoint_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_future.add_done_callback(self.gotopoint_goal_response_callback)


    def gotopoint_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('GoToPoint goal rejected for tb3_2')
            return

        self.get_logger().info('GoToPoint goal accepted for tb3_2')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.gotopoint_result_callback)

    def gotopoint_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('tb3_2 reached ID4 position')


    
    def clbk_set_id_4_pos(self, request, response):
        self.get_logger().info(f"SetID4pos request received: id={request.point}, robot_number={request.robot_number}")
        self.ID4pos = request.point
        response.success = True

        self.send_goal_gotopoint(self.ID4pos)

        return response


    
    def send_goals_to_robots(self):
        num_walls = len(self.wall_segments)
        num_robots = len(self.robot_namespaces)

        if num_walls == 0:
            self.get_logger().error('No wall segments received. Cannot send goals.')
            return

        # Assign walls to robots, repeat last wall if not enough
        self.current_walls = {}
        for i, ns in enumerate(self.robot_namespaces):
            wall_index = min(i, num_walls - 1)  # if fewer walls than robots, repeat last wall
            self.current_walls[ns] = self.wall_segments[wall_index]

        self.get_logger().info('Sending initial goals to robots...')
        for ns, wall in self.current_walls.items():
            self.send_goal(wall, ns, follow_right=ns == 'tb3_0')



    def send_request_wallpoints(self):
        max_retries = 10
        retry_delay = 1.0  # seconds
        
        for attempt in range(max_retries):
            future = self.wall_segments_cli.call_async(self.wall_segments_req)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            
            if response is not None and response.points:
                self.wall_segments = list(response.points)
                self.get_logger().info(
                    f'Received {len(self.wall_segments)} wall segments on attempt {attempt + 1}'
                )

                # drop segments under threshold
                # threshold = 8
                # self.wall_segments = [
                #     segment for segment in self.wall_segments
                #     if len(segment.points) >= threshold
                # ]

                # log wall segment sizes
                for i, segment in enumerate(self.wall_segments):
                    self.get_logger().info(f'Wall segment {i} size: {len(segment.points)} points')

                return True

            self.get_logger().warn(
                f'Wallpoints service returned no segments (attempt {attempt + 1}/{max_retries}). '
                f'Retrying in {retry_delay}s...'
            )
            if attempt < max_retries - 1:
                time.sleep(retry_delay)

        self.get_logger().error('Failed to get wallpoints after all retries')
        return False


    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

        rclpy.shutdown()


    def goal_response_callback(self, future, robot_ns):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f'Goal rejected for {robot_ns}')
            return

        self.get_logger().info(f'Goal accepted for {robot_ns}')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            lambda future, ns=robot_ns: self.get_result_callback(future, ns)
        )


    def get_result_callback(self, future, robot_ns):
        self.get_logger().info(f'{robot_ns} finished exploring its wall.')

        # Determine the other robot's namespace
        other_robot = [r for r in self.robot_namespaces if r != robot_ns][0]
        other_wall = self.current_walls[other_robot]

        # Flip direction for this new wall
        follow_right =  False

        # Send new goal to this robot
        self.get_logger().info(f'{robot_ns} now exploring {other_robot}\'s wall (follow_right={follow_right})')
        self.send_goal(other_wall, robot_ns, follow_right)

        # Update the tracking dictionary
        self.current_walls[robot_ns] = other_wall



    def feedback_callback(self, feedback_msg):
        return


    def send_goal(self, wall: PointArray, robot_ns: str, follow_right: bool):
        action_client = self.action_clients[robot_ns]

        self.get_logger().info(f'Waiting for action server in {robot_ns}...')
        action_client.wait_for_server()

        goal_msg = ExploreWall.Goal()
        goal_msg.wall_points = wall
        goal_msg.wall_id = 0
        goal_msg.follow_right = follow_right

        self.get_logger().info(f'Sending goal to {robot_ns} (follow_right={follow_right})...')
        send_future = action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_future.add_done_callback(
            lambda future, ns=robot_ns: self.goal_response_callback(future, ns)
        )


def main(args=None):
    rclpy.init(args=args)

    robot_leader = LeaderClass()

    rclpy.spin(robot_leader)  # This will now handle sending the goal after segments are received

    robot_leader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
