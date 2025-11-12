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


        self.hasSentID4pos = False

        # Timer to check markers once per second
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback_tb3_0)
        self.timer = self.create_timer(timer_period, self.timer_callback_tb3_1)

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
        # Check tb3_0
        if self.marker_id_0 > 4 or abs(self.marker_position_0.x-self.prev_marker_position_0.x) < 0.1 or abs(self.marker_position_0.y - self.prev_marker_position_0.y) < 0.1:
            return

        if not self.hasSentID4pos:
            self.get_logger().info("Sending SetID4pos for tb3_0")
            req = SetID4pos.Request()
            req.point = self.marker_position_0
            req.robot_number = 0
            self.setID4pos_srv.call_async(req)
            self.hasSentID4pos = True

        
        # send request to set marker position service
        request = SetMarkerPosition.Request()
        request.marker_id = self.marker_id_0
        request.marker_position = self.marker_position_0
        self.cli_set_marker_position.call_async(request)

        self.prev_marker_position_0 = self.marker_position_0

        self.get_logger().info(f"[tb3_0] marker_id: {self.marker_id_0}")
        self.get_logger().info(f"[tb3_0] marker_position: {self.marker_position_0}")

    def timer_callback_tb3_1(self):
        # Check tb3_1
        if self.marker_id_1 > 4 or abs(self.marker_position_1.x - self.prev_marker_position_1.x) < 0.1 or abs(self.marker_position_1.y - self.prev_marker_position_1.y) < 0.1:
            return

        if not self.hasSentID4pos:
            self.get_logger().info("Sending SetID4pos for tb3_1")
            req = SetID4pos.Request()
            req.point = self.marker_position_1
            req.robot_number = 1
            self.setID4pos_srv.call_async(req)
            self.hasSentID4pos = True

        # send request to set marker position service
        request = SetMarkerPosition.Request()
        request.marker_id = self.marker_id_1
        request.marker_position = self.marker_position_1
        self.cli_set_marker_position.call_async(request)

        self.prev_marker_position_1 = self.marker_position_1

        self.get_logger().info(f"[tb3_1] marker_id: {self.marker_id_1}")
        self.get_logger().info(f"[tb3_1] marker_position: {self.marker_position_1}")

        
        # send request to set marker position service
        request = SetMarkerPosition.Request()
        request.marker_id = self.marker_id_1
        request.marker_position = self.marker_position_1
        self.cli_set_marker_position.call_async(request)

        self.prev_marker_position_1 = self.marker_position_1

        self.get_logger().info(f"[tb3_1] marker_id: {self.marker_id_1}")
        self.get_logger().info(f"[tb3_1] marker_position: {self.marker_position_1}")

        


def main(args=None):
    rclpy.init(args=args)
    marker_detection = MarkerDetection()
    rclpy.spin(marker_detection)
    marker_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
