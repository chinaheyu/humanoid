import rclpy
import rclpy.qos
from rclpy.node import Node


class HumanoidControlNode(Node):
    def __init__(self):
        # Initialize node
        super().__init__('humanoid_control')


def main(args=None):
    rclpy.init(args=args)
    humanoid_control_node = HumanoidControlNode()
    rclpy.spin(humanoid_control_node)


if __name__ == '__main__':
    main()
