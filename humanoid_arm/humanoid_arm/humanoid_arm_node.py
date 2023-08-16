import rclpy
from rclpy.node import Node
import moteus


class HumanoidArmNode(Node):
    def __init__(self):
        super().__init__('humanoid_arm')

def main(args=None):
    rclpy.init(args=args)
    humanoid_arm_node = HumanoidArmNode()
    rclpy.spin(humanoid_arm_node)


if __name__ == '__main__':
    main()
