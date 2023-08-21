import rclpy
import rclpy.qos
from rclpy.node import Node
from humanoid_interface.msg import MotorControl, MotorFeedback


class HumanoidControlNode(Node):
    def __init__(self):
        # Initialize node
        super().__init__('humanoid_control')
        
        # Create ros interface
        self._motor_control_publisher = self.create_publisher(MotorControl, 'motor_control', rclpy.qos.QoSPresetProfiles.get_from_short_key("SYSTEM_DEFAULT"))
        self._motor_feedback_subscription = self.create_subscription(MotorFeedback, 'motor_feedback', self._motor_feedback_callback, rclpy.qos.QoSPresetProfiles.get_from_short_key("SENSOR_DATA"))

    def _motor_feedback_callback(self, msg: MotorFeedback):
        pass
    
    def control_motor(self, msg: MotorControl):
        self._motor_control_publisher(msg)


def main(args=None):
    rclpy.init(args=args)
    humanoid_control_node = HumanoidControlNode()
    rclpy.spin(humanoid_control_node)


if __name__ == '__main__':
    main()
