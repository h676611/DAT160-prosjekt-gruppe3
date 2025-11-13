import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose, Point
from scoring_interfaces.srv import SetMarkerPosition
from comp3_interfaces.srv import SetID4pos


class MarkerDetection(Node):
    def __init__(self):
        super().__init__('marker_detection')

        # ----------- Service clients -----------
        self.setID4pos_srv = self.create_client(SetID4pos, 'set_id_4_pos')
        while not self.setID4pos_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_id_4_pos service...')
        self.cli_set_marker_position = self.create_client(SetMarkerPosition, '/set_marker_position')
        while not self.cli_set_marker_position.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_marker_position service...')


        self.marker_dict = {
            0: False,
            1: False,
            2: False,
            3: False,
            4: False
        }

        # ----------- Configuration -----------
        self.buffer_size = 5
        self.stable_id_threshold = 3
        self.radius = 0.1  # for majority clustering
        self.robots = ['tb3_0', 'tb3_1', 'tb3_2', 'tb3_3']

        # ----------- State tracking for each robot -----------
        self.marker_id = {ns: 1000 for ns in self.robots}
        self.marker_position = {ns: Point() for ns in self.robots}
        self.prev_marker_position = {ns: Point() for ns in self.robots}
        self.id4_buffer = {ns: [] for ns in self.robots}
        self.prev_id4_position = {ns: Point() for ns in self.robots}

        self.hasSentID4pos = False
        # ----------- Create subscriptions and timers -----------
        for ns in self.robots:
            self.create_subscription(Pose, f'/{ns}/marker_map_pose',
                                     lambda msg, ns=ns: self.clbk_marker_map_pose(msg, ns), 10)
            self.create_subscription(Int64, f'/{ns}/marker_id',
                                     lambda msg, ns=ns: self.clbk_marker_id(msg, ns), 10)
            self.create_timer(0.1, lambda ns=ns: self.timer_callback(ns))
        
        self.marker_last_seen_time = {ns: 0.0 for ns in self.robots}

    # ----------- Callback functions -----------
    def clbk_marker_map_pose(self, msg, ns):
        self.marker_position[ns] = msg.position
        self.marker_last_seen_time[ns] = self.get_clock().now().nanoseconds / 1e9  # seconds


    def clbk_marker_id(self, msg, ns):
        self.marker_id[ns] = msg.data

    # ----------- Timer callback for all robots -----------
    def timer_callback(self, ns):
        current_id = self.marker_id[ns]
        current_pos = self.marker_position[ns]

        current_time = self.get_clock().now().nanoseconds / 1e9


        if current_id <= 4 and not self.marker_dict[current_id] and current_time - self.marker_last_seen_time[ns] < 1.0:
            req = SetMarkerPosition.Request()
            req.marker_id = current_id
            req.marker_position = current_pos
            future = self.cli_set_marker_position.call_async(req)

            def callback(fut, ns=ns, marker_id=current_id):
                try:
                    result = fut.result()
                    if result.accepted:
                        self.get_logger().info(f"[{ns}] Marker ID {marker_id} accepted by scoring system.")
                        self.marker_dict[marker_id] = True  # only update on acceptance
                    else:
                        self.get_logger().warn(f"[{ns}] Marker ID {marker_id} rejected by scoring system.")
                except Exception as e:
                    self.get_logger().error(f"[{ns}] Service call failed: {e}")

            future.add_done_callback(callback)



        # Buffer ID4 positions for stability
        if not self.hasSentID4pos and current_id == 4 and self.marker_dict[4]:
            self.get_logger().info(f"[{ns}] Sending stable SetID4pos")
            req = SetID4pos.Request()
            req.point = current_pos
            self.setID4pos_srv.call_async(req)
            self.hasSentID4pos = True
            


    # ----------- Helper: majority filtering -----------
    def get_majority_point(self, points):
        clusters = []
        for p in points:
            placed = False
            for cluster in clusters:
                cx = sum(pt.x for pt in cluster) / len(cluster)
                cy = sum(pt.y for pt in cluster) / len(cluster)
                if (p.x - cx)**2 + (p.y - cy)**2 < self.radius**2:
                    cluster.append(p)
                    placed = True
                    break
            if not placed:
                clusters.append([p])

        largest_cluster = max(clusters, key=len)
        mean_point = Point()
        mean_point.x = sum(p.x for p in largest_cluster) / len(largest_cluster)
        mean_point.y = sum(p.y for p in largest_cluster) / len(largest_cluster)
        mean_point.z = sum(p.z for p in largest_cluster) / len(largest_cluster)
        return mean_point


def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
