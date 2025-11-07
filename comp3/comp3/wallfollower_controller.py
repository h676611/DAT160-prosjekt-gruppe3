import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from rclpy.qos import qos_profile_sensor_data

import math
from typing import List, Optional, Tuple


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class WallfollowerController(Node):
    """
    Wall follower with the richer state-machine logic from wall_follower.py.
    Modes:
        SEARCH -> drive forward + slight right yaw to find a left wall
        FOLLOW -> maintain distance to left wall via P controller
        AVOID  -> obstacle ahead, turn left until clear
        BRIDGE -> left wall disappeared at convex corner, steer right to reacquire
    """

    def __init__(self):
        super().__init__('wallfollower_controller')

        # Parameters mirrored from wall_follower.py so behavior stays consistent.
        self.declare_parameter('desired_distance', 0.55)
        self.declare_parameter('front_clearance', 0.60)
        self.declare_parameter('max_linear', 0.28)
        self.declare_parameter('max_angular', 1.2)
        self.declare_parameter('kp_dist', 1.8)
        self.declare_parameter('kp_ang', 1.1)
        self.declare_parameter('search_yaw', -0.1)
        self.declare_parameter('scan_left_deg', 290)
        self.declare_parameter('scan_left_spread', 10)
        self.declare_parameter('scan_front_spread', 20)
        self.declare_parameter('nan_substitute', 10.0)
        self.declare_parameter('gap_factor', 1.5)
        self.declare_parameter('gap_clear_factor', 1.2)
        self.declare_parameter('gap_linear', 0.10)
        self.declare_parameter('gap_turn', 0.60)

        gp = self.get_parameter
        self.desired_distance = float(gp('desired_distance').value)
        self.front_clearance = float(gp('front_clearance').value)
        self.v_max = float(gp('max_linear').value)
        self.w_max = float(gp('max_angular').value)
        self.kp_dist = float(gp('kp_dist').value)
        self.kp_ang = float(gp('kp_ang').value)
        self.search_yaw = float(gp('search_yaw').value)
        self.left_deg = int(gp('scan_left_deg').value)
        self.left_spread = int(gp('scan_left_spread').value)
        self.front_spread = int(gp('scan_front_spread').value)
        self.nan_sub = float(gp('nan_substitute').value)
        self.gap_factor = float(gp('gap_factor').value)
        self.gap_clear_factor = float(gp('gap_clear_factor').value)
        self.gap_linear = float(gp('gap_linear').value)
        self.gap_turn = float(gp('gap_turn').value)

        # ROS interfaces
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.clbk_laser, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.srv = self.create_service(SetBool, 'wall_follow', self.clbk_wall_follow)

        # Internal state
        self.active = False
        self.mode = 'SEARCH'
        self.latest_scan: Optional[LaserScan] = None

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz like wall_follower.py
        self.get_logger().info('wallfollower_controller ready. Use /wall_follow to toggle.')

    def clbk_wall_follow(self, request: SetBool.Request, response: SetBool.Response):
        self.active = bool(request.data)
        if self.active:
            self.mode = 'SEARCH'
            response.message = 'Wall following enabled'
        else:
            self._stop(n=3)
            response.message = 'Wall following disabled'
        response.success = True
        return response

    def clbk_laser(self, msg: LaserScan):
        self.latest_scan = msg

    # ----- Helpers copied from wall_follower.py -----
    def _window(self, arr: List[float], center_deg: int, spread: int,
                angle_min: float, angle_inc: float) -> List[float]:
        rad = math.radians(center_deg)
        i_center = int(round((rad - angle_min) / angle_inc))
        lo = max(0, i_center - spread)
        hi = min(len(arr) - 1, i_center + spread)
        values = []
        for i in range(lo, hi + 1):
            v = arr[i]
            values.append(v if math.isfinite(v) and v > 0.0 else self.nan_sub)
        return values if values else [self.nan_sub]

    def _front_min(self, scan: LaserScan) -> float:
        vals_pos = self._window(scan.ranges, 0, self.front_spread, scan.angle_min, scan.angle_increment)
        vals_neg = self._window(scan.ranges, 360, self.front_spread, scan.angle_min, scan.angle_increment)
        return min(min(vals_pos), min(vals_neg))

    def _frontleft_min(self, scan: LaserScan) -> float:
        vals = self._window(scan.ranges, 45, 8, scan.angle_min, scan.angle_increment)
        return min(vals)

    def _left_distance_and_slope(self, scan: LaserScan) -> Tuple[float, float]:
        win = self._window(scan.ranges, self.left_deg, self.left_spread, scan.angle_min, scan.angle_increment)
        d_avg = sum(win) / len(win)
        slope = (win[-1] - win[0]) / max(1, len(win) - 1)
        return d_avg, slope

    def _stop(self, n: int = 1):
        for _ in range(n):
            self.cmd_pub.publish(Twist())

    # ----- Control loop -----
    def timer_callback(self):
        if not self.active or self.latest_scan is None:
            return

        scan = self.latest_scan
        front = self._front_min(scan)
        front_left = self._frontleft_min(scan)
        left_dist, left_slope = self._left_distance_and_slope(scan)

        lost_left = left_dist > self.desired_distance * self.gap_factor

        if front < self.front_clearance or front_left < (self.front_clearance * 0.95):
            self.mode = 'AVOID'
        else:
            self.mode = 'BRIDGE' if lost_left else 'FOLLOW'

        twist = Twist()

        if self.mode == 'SEARCH':
            twist.linear.x = 0.12
            twist.angular.z = self.search_yaw

        elif self.mode == 'AVOID':
            twist.linear.x = 0.06
            twist.angular.z = min(0.9, self.w_max)
            if (front >= self.front_clearance * 1.15) and (front_left >= self.front_clearance * 1.10):
                self.mode = 'BRIDGE' if lost_left else 'FOLLOW'

        elif self.mode == 'BRIDGE':
            twist.linear.x = self.gap_linear
            twist.angular.z = clamp(-self.gap_turn, -self.w_max, self.w_max)
            if left_dist <= self.desired_distance * self.gap_factor * self.gap_clear_factor:
                self.mode = 'FOLLOW'
            elif front < self.front_clearance:
                self.mode = 'AVOID'

        else:  # FOLLOW
            err = self.desired_distance - left_dist
            w = self.kp_dist * err + self.kp_ang * (-left_slope)
            if front < self.front_clearance * 1.10:
                w += 0.20
            v = self.v_max * (1.0 - clamp(abs(err) / max(self.desired_distance, 1e-3), 0.0, 0.9))
            twist.linear.x = clamp(v, 0.08, self.v_max)
            twist.angular.z = clamp(w, -self.w_max, self.w_max)

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    controller = WallfollowerController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
