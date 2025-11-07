import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
from std_srvs.srv import SetBool
import numpy as np
from rclpy.qos import qos_profile_sensor_data



class WallfollowerController(Node):
    def __init__(self):
        Node.__init__(self, node_name="wallfollower_controller")

        self.sub = self.create_subscription(LaserScan, 'scan', self.clbk_laser, qos_profile_sensor_data)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.srv = self.create_service(SetBool, 'wall_follow', self.clbk_wall_follow)

        self.active = False

        self.front = 100.0
        self.right = 100.0
        self.left = 100.0

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) 


    def clbk_wall_follow(self, request, response):

        if request.data == False:
            vel_msg = Twist()
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.pub.publish(vel_msg)

        self.active = request.data
        response.success = True
        return response


    #Callback function for the Turtlebots Lidar topic /scan
    def clbk_laser(self, msg):
        values = np.concatenate((msg.ranges[355:], msg.ranges[:6]))  # 350–364 + 0–10
        max_val = np.min(values)
        self.front = max_val
        self.right = msg.ranges[310]
        self.left = msg.ranges[55]

       
    def timer_callback(self):

        if not self.active:
            return

        if self.right == 100.0:
            return
        
        vel_msg = Twist()

        desired_distance = 0.6
        hyp = 0.4

        error = abs(self.right - hyp)

        if self.front < desired_distance:
            # wall ahead turning left  
            # self.get_logger().info("wall ahead turning left") 
            vel_msg.angular.z = 0.8
            vel_msg.linear.x = -0.05
        elif self.right > 5.0:
            # no wall on right turning right
            # self.get_logger().info("no wall on right turning right") 
            vel_msg.angular.z = -0.5
            vel_msg.linear.x = 0.2
        else:
            vel_msg.linear.x = 0.4
            if error > 0.05:
                if self.right < hyp:
                    # self.get_logger().info("too close to wall turning left") 
                    # too close to wall turning left
                    vel_msg.angular.z = min(0.6, 1.4 * error)
                else:
                    # self.get_logger().info("too far away from wall turning right") 
                    # too far away from wall turning right
                    vel_msg.angular.z = -min(0.6, 1.4 * error)
            else:
                # desired distance to wall go forward
                vel_msg.angular.z = 0.0

        self.pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)

    controller = WallfollowerController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()