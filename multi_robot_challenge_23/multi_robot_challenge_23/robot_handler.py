import rclpy
from rclpy.node import Node


class RobotHandler(Node):
    def __init__(self):
        #TODO: Initialze a ros node
        Node.__init__(self, node_name="robot_handler")
        




def main(args=None):
    rclpy.init(args=args)

    robot_handler = RobotHandler()

    rclpy.spin(robot_handler)

    robot_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()