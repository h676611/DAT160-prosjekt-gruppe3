import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from comp3_interfaces.srv import Switch 
import math

from tf_transformations import euler_from_quaternion


class Gotopointcontroller(Node):
    def __init__(self):
        Node.__init__(self, node_name="gotopoint_controller")

        self.sub_odom = self.create_subscription(Odometry, 'odom', self.clbk_odom, 10)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.srv = self.create_service(Switch, 'go_to_point', self.clbk_switch) 

        self.pose = 0.0
        self.yaw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.goal = Point()

        self.active = False

        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) 

    def clbk_switch(self, request, response) :

        if request.move_switch == False:
            vel_msg = Twist()
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.pub.publish(vel_msg)

        self.goal = request.target_position
        self.active = request.move_switch
        response.success = True
        return response


    def clbk_odom(self, msg):
        position = msg.pose.pose.position
        self.x = position.x
        self.y = position.y
        self.z = position.z
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)

        self.yaw = yaw


    def timer_callback(self):

        if not self.active:
            return
        
        vel_msg = Twist()

        dx = self.goal.x - self.x
        dy = self.goal.y - self.y

        distance = math.sqrt(dx**2 + dy**2)

        k = 0.7
        boundary = 0.01 * distance

        # Desired heading
        degree_to_goal = math.atan2(dy, dx)

        # Heading error normalized to [-pi, pi]
        error = (degree_to_goal - self.yaw + math.pi) % (2 * math.pi) - math.pi

        if abs(error) > boundary:
            vel_msg.angular.z = max(-1.0, min(1.0, k * error))
            vel_msg.linear.x = 0.5 * math.cos(error)
        else:
            vel_msg.linear.x = min(0.4, 0.2 + 0.4 * distance)

        self.pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)

    controller = Gotopointcontroller()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()