import rclpy
from rclpy.node import Node
import rclpy.qos
import moteus
from dataclasses import dataclass
import threading
import asyncio
from humanoid_interface.msg import MotorControl, MotorFeedback


@dataclass
class MotorDataClass:
    id: int
    controller: moteus.Controller
    target_position: float = 0


class HumanoidArmNode(Node):
    def __init__(self):
        super().__init__('humanoid_arm')
        self._motors = {}
        for i in range(14, 15):
            self._motors[i] = MotorDataClass(id=i, controller=moteus.Controller(id=i))
        
        self._motor_feedback_publisher = self.create_publisher(MotorFeedback, "motor_feedback", rclpy.qos.QoSPresetProfiles.get_from_short_key("SENSOR_DATA"))
        self._motor_control_subscription = self.create_subscription(MotorControl, "motor_control", self._motor_control_callback, rclpy.qos.QoSPresetProfiles.get_from_short_key("SYSTEM_DEFAULT"))
        
        self._control_thread = threading.Thread(target=asyncio.run, args=[self._control_loop()])
        self._control_thread.start()
    
    def _motor_control_callback(self, msg: MotorControl):
        if msg.id in self._motors and msg.control_type == MotorControl.MOTOR_POSITION_CONTROL:
            self._motors[msg.id].target_position = msg.position

    async def _control_loop(self):
        transport = moteus.Fdcanusb()
        await transport.cycle([c.controller.make_stop() for c in self._motors.values()])
        while True:
            states = await transport.cycle([c.controller.make_position(position=c.target_position, query=True) for c in self._motors.values()])
            for state in states:
                self._motor_feedback_publisher.publish(
                    MotorFeedback(
                        stamp=self.get_clock().now().to_msg(),
                        id=state.id,
                        position=state.values[moteus.Register.POSITION],
                        velocity=state.values[moteus.Register.VELOCITY],
                        torque=state.values[moteus.Register.TORQUE]
                    )
                )
            await asyncio.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    humanoid_arm_node = HumanoidArmNode()
    rclpy.spin(humanoid_arm_node)


if __name__ == '__main__':
    main()
