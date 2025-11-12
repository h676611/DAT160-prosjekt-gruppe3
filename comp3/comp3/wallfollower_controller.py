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
        Node.__init__(self, node_name="wallfollower_controller")

        self.sub = self.create_subscription(LaserScan, 'scan', self.clbk_laser, 10)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.srv = self.create_service(Wallfollow, 'wall_follow', self.clbk_wall_follow)


        self.odom_sub = self.create_subscription(Odometry, 'odom', self.clbk_odom, 10)
        self.position = None

        current_namespace = self.get_namespace()[-1]
        if current_namespace == '1':
            other_namespace = 'tb3_0'
        else:
            other_namespace = 'tb3_1'

        self.other_odom_sub = self.create_subscription(Odometry, f'/{other_namespace}/odom', self.clbk_other_odom, 10)

        self.other_robot_position = None

        self.active = False
        self.follow_right = False

        self.front = 100.0
        self.right = 100.0
        self.left = 100.0

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) 

    def clbk_odom(self, msg):
        self.position = msg.pose.pose.position

    def clbk_other_odom(self, msg):
        self.other_robot_position = msg.pose.pose.position
        

    def clbk_wall_follow(self, request, response):

        if request.activate == False:
            vel_msg = Twist()
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.pub.publish(vel_msg)

        self.active = request.activate
        self.follow_right = request.follow_right
        response.success = True
        return response


    #Callback function for the Turtlebots Lidar topic /scan
    def clbk_laser(self, msg):
        values = np.concatenate((msg.ranges[350:], msg.ranges[:11]))  # 350–364 + 0–10
        max_val = np.min(values)
        self.front = max_val 
        self.right = msg.ranges[315] # 45 degrees right of front
        self.left = msg.ranges[45]

    def distance(self, pos1, pos2):
        dx = pos1.x - pos2.x
        dy = pos1.y - pos2.y
        return math.sqrt(dx**2 + dy**2)

       
    def timer_callback(self):

        if not self.active:
            return

        if self.right == 100.0:
            return
        
        if self.position is None or self.other_robot_position is None:
            return
        
        if self.distance(self.position, self.other_robot_position) < 0.3:
            vel_msg = Twist()
            vel_msg.linear.x = -0.1
            vel_msg.angular.z = 0.2
            self.pub.publish(vel_msg)
            return

        vel_msg = Twist()

        desired_distance = 0.5
        hyp = 0.43

        if self.follow_right:
            error = abs(self.right - hyp)
            if self.front < desired_distance + 0.1:
                # wall ahead turning left
                vel_msg.angular.z = 0.8
                vel_msg.linear.x = -0.01
            elif self.right > 5.0:
                # no wall on right turning right
                vel_msg.angular.z = -0.55
                vel_msg.linear.x = 0.1 
            else:
                vel_msg.linear.x = 0.3
                if error > 0.02:
                    if self.right < hyp:
                        # too close to wall turning left
                        vel_msg.angular.z = min(0.7, 1.3 * error)
                    else:
                        # too far away from wall turning right
                        vel_msg.angular.z = -min( 0.7, 1.3 * error)
                    vel_msg.linear.x = max(0.3, 0.1 * (1 - abs(vel_msg.angular.z)/0.7))
                else:
                    # desired distance to wall go forward
                    vel_msg.angular.z = 0.0
                    vel_msg.linear.x = 0.5
        else:
            error = abs(self.left - hyp)
            if self.front < desired_distance + 0.1:
                # wall ahead turning left
                vel_msg.angular.z = -0.8
                vel_msg.linear.x = -0.01
            elif self.left > 5.0:
                # no wall on right turning right
                vel_msg.angular.z = 0.55
                vel_msg.linear.x = 0.1
            else:
                vel_msg.linear.x = 0.3
                if error > 0.02:
                    if self.left < hyp:
                        # too close to wall turning left
                        vel_msg.angular.z = -min(0.7, 1.3 * error)
                    else:
                        # too far away from wall turning right
                        vel_msg.angular.z = min( 0.7, 1.3 * error)
                    vel_msg.linear.x = max(0.3, 0.1 * (1 - abs(vel_msg.angular.z)/0.7))
                else:
                    # desired distance to wall go forward
                    vel_msg.angular.z = 0.0
                    vel_msg.linear.x = 0.5



        self.pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)

    controller = WallfollowerController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()