import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose, Point
from scoring_interfaces.srv import SetMarkerPosition
from comp3_interfaces.srv import SetID4pos

class MarkerDetection(Node):
    def __init__(self):
        super().__init__('marker_detection')

        # service to set marker position
        self.setID4pos_srv = self.create_client(SetID4pos,'set_id_4_pos')
        while not self.setID4pos_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.setID4pos_req = SetID4pos.Request()

        #------------------ Subscriptions for tb3_0 ------------------
        self.sub_marker_pose_0 = self.create_subscription(
            Pose, '/tb3_0/marker_map_pose', self.clbk_marker_map_pose_0, 10
        )
        self.sub_marker_id_0 = self.create_subscription(
            Int64, '/tb3_0/marker_id', self.clbk_marker_id_0, 10
        )

        #------------------ Subscriptions for tb3_1 ------------------
        self.sub_marker_pose_1 = self.create_subscription(
            Pose, '/tb3_1/marker_map_pose', self.clbk_marker_map_pose_1, 10
        )
        self.sub_marker_id_1 = self.create_subscription(
            Int64, '/tb3_1/marker_id', self.clbk_marker_id_1, 10
        )

        #------------------ Service Client scoring------------------
        self.cli_set_marker_position = self.create_client(SetMarkerPosition,'/set_marker_position')
        # wait for the service to be available
        while not self.cli_set_marker_position.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        #------------------ Variables for tb3_0 ------------------
        self.prev_marker_id_0 = -1
        self.marker_id_0 = 1000
        self.marker_position_0 = Point()
        self.prev_marker_position_0 = Point()

        #------------------ Variables for tb3_1 ------------------
        self.prev_marker_id_1 = -1
        self.marker_id_1 = 1000
        self.marker_position_1 = Point()
        self.prev_marker_position_1 = Point()

        self.id4_buffer_0 = []         # Buffer for tb3_0 ID4
        self.id4_buffer_1 = []         # Buffer for tb3_1 ID4
        self.buffer_size = 5            # Number of frames to average over
        self.stable_id_threshold = 3    # Minimum number of detections to consider stable
        self.hasSentID4pos_0 = False
        self.hasSentID4pos_1 = False
        self.prev_id4_position_0 = Point()
        self.prev_id4_position_1 = Point()

        # Timer to check markers once per second
        timer_period = 0.1  # seconds
        self.timer_tb3_0 = self.create_timer(timer_period, self.timer_callback_tb3_0)
        self.timer_tb3_1 = self.create_timer(timer_period, self.timer_callback_tb3_1)


    #------------------ Callbacks for tb3_0 ------------------
    def clbk_marker_map_pose_0(self, msg):
        self.marker_position_0 = msg.position

    def clbk_marker_id_0(self, msg):
        self.marker_id_0 = msg.data

    #------------------ Callbacks for tb3_1 ------------------
    def clbk_marker_map_pose_1(self, msg):
        self.marker_position_1 = msg.position

    def clbk_marker_id_1(self, msg):
        self.marker_id_1 = msg.data

    #------------------ Timer callback ------------------
    def timer_callback_tb3_0(self):
        if self.marker_id_0 <= 4:
            # Always report to scoring service
            request = SetMarkerPosition.Request()
            request.marker_id = self.marker_id_0
            request.marker_position = self.marker_position_0
            self.cli_set_marker_position.call_async(request)

        # Only buffer ID4 for leader
        if self.marker_id_0 == 4:
            self.id4_buffer_0.append(self.marker_position_0)
            if len(self.id4_buffer_0) > self.buffer_size:
                self.id4_buffer_0.pop(0)

            # Check if we have enough frames for stable position
            if len(self.id4_buffer_0) >= self.stable_id_threshold:
                avg_pose = Point()
                avg_pose.x = sum(p.x for p in self.id4_buffer_0) / len(self.id4_buffer_0)
                avg_pose.y = sum(p.y for p in self.id4_buffer_0) / len(self.id4_buffer_0)
                avg_pose.z = sum(p.z for p in self.id4_buffer_0) / len(self.id4_buffer_0)

                # Only send if moved enough from previous
                delta_sq = (avg_pose.x - self.prev_id4_position_0.x)**2 + (avg_pose.y - self.prev_id4_position_0.y)**2
                if delta_sq > 0.0025 and not self.hasSentID4pos_0:
                    self.get_logger().info("Sending stable SetID4pos for tb3_0")
                    req = SetID4pos.Request()
                    req.point = avg_pose
                    req.robot_number = 0
                    self.setID4pos_srv.call_async(req)
                    self.hasSentID4pos_0 = True

                self.prev_id4_position_0 = avg_pose

    def timer_callback_tb3_1(self):
        if self.marker_id_1 <= 4:
            # Always report to scoring service
            request = SetMarkerPosition.Request()
            request.marker_id = self.marker_id_1
            request.marker_position = self.marker_position_1
            self.cli_set_marker_position.call_async(request)

        # Only buffer ID4 for leader
        if self.marker_id_1 == 4:
            self.id4_buffer_1.append(self.marker_position_1)
            if len(self.id4_buffer_1) > self.buffer_size:
                self.id4_buffer_1.pop(0)

            # Check if we have enough frames for stable position
            if len(self.id4_buffer_1) >= self.stable_id_threshold:
                avg_pose = Point()
                avg_pose.x = sum(p.x for p in self.id4_buffer_1) / len(self.id4_buffer_1)
                avg_pose.y = sum(p.y for p in self.id4_buffer_1) / len(self.id4_buffer_1)
                avg_pose.z = sum(p.z for p in self.id4_buffer_1) / len(self.id4_buffer_1)

                # Only send if moved enough from previous
                delta_sq = (avg_pose.x - self.prev_id4_position_1.x)**2 + (avg_pose.y - self.prev_id4_position_1.y)**2
                if delta_sq > 0.0025 and not self.hasSentID4pos_1:
                    self.get_logger().info("Sending stable SetID4pos for tb3_1")
                    req = SetID4pos.Request()
                    req.point = avg_pose
                    req.robot_number = 0
                    self.setID4pos_srv.call_async(req)
                    self.hasSentID4pos_1 = True

                self.prev_id4_position_1 = avg_pose

        


def main(args=None):
    rclpy.init(args=args)
    marker_detection = MarkerDetection()
    rclpy.spin(marker_detection)
    marker_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
