import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
from comp3_interfaces.srv import Wallfollow
from nav_msgs.msg import Odometry
import numpy as np


class WallfollowerController(Node):
    def __init__(self):
        super().__init__("wallfollower_controller")

        # --- Subscriptions & publishers ---
        self.sub = self.create_subscription(LaserScan, 'scan', self.clbk_laser, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.srv = self.create_service(Wallfollow, 'wall_follow', self.clbk_wall_follow)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.clbk_odom, 10)

        self.position = None
        self.other_robot_positions = {}

        # Detect current namespace (e.g., "/tb3_0")
        self.current_namespace = self.get_namespace().strip('/') or 'tb3_0'

        # Subscribe to all robots’ odometry except self
        for ns in ['tb3_0', 'tb3_1', 'tb3_2']:
            if ns != self.current_namespace:
                self.create_subscription(Odometry, f'/{ns}/odom',
                                         lambda msg, ns=ns: self.clbk_other_odom(msg, ns), 10)

        # --- Behavior state ---
        self.active = False
        self.follow_right = False  # If False, follow left wall

        # --- Lidar values ---
        self.front = 100.0
        self.right = 100.0
        self.left = 100.0

        # --- Control parameters ---
        self.desired_distance = 0.6
        self.wall_distance_target = 0.55
        self.distance_threshold = 0.02
        self.collision_distance = 0.4

        # --- Timer ---
        self.timer = self.create_timer(0.05, self.timer_callback)

    # -------------------------------------------------------
    # ROS CALLBACKS
    # -------------------------------------------------------
    def clbk_odom(self, msg):
        self.position = msg.pose.pose.position

    def clbk_other_odom(self, msg, ns):
        self.other_robot_positions[ns] = msg.pose.pose.position

    def clbk_wall_follow(self, request, response):
        self.active = request.activate
        self.follow_right = request.follow_right

        if not request.activate:
            self.pub.publish(Twist())  # Stop
        response.success = True
        return response

    def clbk_laser(self, msg):
        # Average readings in small sectors to smooth noise
        front_sector = np.concatenate((msg.ranges[345:], msg.ranges[:16]))
        self.front = np.min(front_sector)

        right_sector = msg.ranges[300:330]
        self.right = np.min(right_sector)
        
        left_sector = msg.ranges[30:60]
        self.left = np.min(left_sector)

    # -------------------------------------------------------
    # HELPER METHODS
    # -------------------------------------------------------
    def distance(self, pos1, pos2):
        dx, dy = pos1.x - pos2.x, pos1.y - pos2.y
        return math.sqrt(dx**2 + dy**2)
    
    def too_close_to_other_robot(self):
        """Return (namespace, distance) of closest robot if within threshold, else (None, None)."""
        if self.position is None or not self.other_robot_positions:
            return None, None

        closest_ns = None
        closest_dist = float('inf')

        for ns, pos in self.other_robot_positions.items():
            if pos is None:
                continue
            d = self.distance(self.position, pos)
            if d < closest_dist:
                closest_dist = d
                closest_ns = ns

        if closest_ns and closest_dist < self.collision_distance:
            return closest_ns, closest_dist
        else:
            return None, None


    # -------------------------------------------------------
    # MAIN CONTROL LOGIC
    # -------------------------------------------------------
    def wall_follow_logic(self):
        """Unified wall-following logic for both left/right sides."""
        vel = Twist()

        # Choose side: right = 1, left = -1 (flips angular direction)
        side = 1 if self.follow_right else -1
        wall_distance = self.right if self.follow_right else self.left

        # Stop if too close to obstacle in front
        if self.front < self.desired_distance + 0.1:
            vel.angular.z = 0.8 * side  # turn away from wall
            vel.linear.x = -0.01
            return vel

        # Turn toward wall if no wall detected
        if wall_distance > 5.0:
            vel.angular.z = -0.5 * side
            vel.linear.x = 0.1
            return vel

        # Compute proportional correction based on wall distance
        error = wall_distance - self.wall_distance_target
        if abs(error) > self.distance_threshold:
            vel.angular.z = -side * min(0.7, 1.3 * abs(error)) * math.copysign(1, error)
            vel.linear.x = max(0.4, 0.15 * (1 - abs(vel.angular.z) / 0.7))
        else:
            vel.angular.z = 0.0
            vel.linear.x = 0.5

        return vel

    # -------------------------------------------------------
    # TIMER LOOP
    # -------------------------------------------------------
    def timer_callback(self):
        if not self.active or self.right == 100.0 or self.position is None:
            return

        # Collision avoidance
        ns, too_close = self.too_close_to_other_robot()
        if too_close:
            if self.get_namespace().strip('/') < ns:
                self.get_logger().info(f"Too close to {ns}, yielding.")
                vel = Twist()
                vel.angular.z = 0.8  # turn away
                self.pub.publish(vel)
                return
            else:
                self.get_logger().info(f"Too close to {ns}, proceeding.")

        # Wall following
        vel = self.wall_follow_logic()
        self.pub.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    controller = WallfollowerController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
