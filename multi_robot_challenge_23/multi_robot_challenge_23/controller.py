import rclpy
from rclpy.node import Node

#TODO: Import the message types that you need for your publishers and subscribers here:
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class BraitenbergController(Node):
    def __init__(self):
        #TODO: Initialze a ros node
        Node.__init__(self, node_name="controller")

        #TODO: Create a subscriber to the /scan topic using, as a callback function, the existing function self.clbk_laser and queue size of 10
        self.sub = self.create_subscription(LaserScan, 'tb3_0/scan', self.clbk_laser, 10)

        #TODO: Create a publisher to the /cmd_vel topic with a queue size of 10
        self.pub = self.create_publisher(Twist, 'tb3_0/cmd_vel', 10)

        #Default values for the lidar variables as a placeholder until the actual sensor values are recieved through from the ros topic
        self.lidar_left_front = 100
        self.lidar_right_front = 100

        #Creates a timer definition -> during rclpy.spin() the function self.timer_callback() will be executed every 0.1 seconds
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) 

    #Callback function for the Turtlebots Lidar topic /scan
    def clbk_laser(self, msg):
        self.lidar_left_front = msg.ranges[12]
        self.lidar_right_front = msg.ranges[348]

    def timer_callback(self):
        #Creates a message from type Twist
        vel_msg = Twist()
        #Defines the speed at which the robot will move forward (value between 0 and 1)
        vel_msg.linear.x = 0.7
        #Defines the speed at which the robot will turn around its own axis (value between -1 and 1)
        vel_msg.angular.z = 0.0
        
        #TODO: Set vel_msg.linear.x and vel_msg.angular.z depending on the values from self.lidar_left_front and self.lidar_right_front    
        
        threshold = 1.8

        if self.lidar_left_front < threshold and self.lidar_right_front < threshold:
            print("Both sensors detect an obstacle -> stop")
            vel_msg.angular.z = 0.0
            vel_msg.linear.x = 0.0
        elif self.lidar_left_front < threshold:
            print("Only the left sensor detects an obstacle -> turn right")
            vel_msg.angular.z = -1.0
        elif self.lidar_right_front < threshold:
            print("Only the right sensor detects an obstacle -> turn left")
            vel_msg.angular.z = 1.0
        else:
            vel_msg.linear.x = 1.0
            vel_msg.angular.z = 0.0
            print("No obstacles are detected -> move forward")
        
        #TODO: Publish vel_msg using the previously defined publisher
        self.pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)

    controller = BraitenbergController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()