import rclpy
import rclpy.qos
import moteus
from .humanoid_arm_node import HumanoidArmNode
import asyncio
from humanoid_interface.msg import MotorFeedback
import os
from humanoid_interface.srv import TeachArm
import json
import numpy as np


class ArmTeachNode(HumanoidArmNode):
    def __init__(self):
        super().__init__()
        
        self._teach_service = self.create_service(TeachArm, "arm/teach", self._teach_callback)
    
    def _save_frame(self, frame_name) -> bool:
        frame_dict = {m.id: m.feedback[0] for m in self._motors.values()}
        with open(os.path.join(self._frames_data_path, f'{frame_name}.json'), 'w') as fp:
            fp.write(json.dumps(frame_dict, indent=4))
        return True
    
    def _teach_callback(self, request: TeachArm.Request, response: TeachArm.Response) -> TeachArm.Response:
        response.result = self._save_frame(request.frame_name)
        return response

    async def _control_loop(self) -> None:
        # Wait motors online
        while rclpy.ok():
            for m in self._motors.values():
                if not await self._detect_motor(m):
                    self.get_logger().error(f'Motor {m.id} not found.')
                    break
            else:
                break
            asyncio.sleep(1)

        # Enter control loop
        transport = moteus.Fdcanusb()
        await transport.cycle([c.controller.make_stop() for c in self._motors.values()])
        while rclpy.ok():
             # control motors
            states = await transport.cycle([c.controller.make_brake(query=True) for c in self._motors.values()])
            for state in states:
                # mapping motor position
                position = state.values[moteus.Register.POSITION] * 2 * np.pi - self._motors[state.id].offset
                if self._motors[state.id].reverse:
                    position = -position
                
                # update feedback
                self._motors[state.id].feedback[0] = position
                self._motors[state.id].feedback[1] = state.values[moteus.Register.VELOCITY] * 2 * np.pi
                
                # publish feedback
                self._motor_feedback_publisher.publish(
                    MotorFeedback(
                        stamp=self.get_clock().now().to_msg(),
                        id=state.id,
                        position=self._motors[state.id].feedback[0],
                        velocity=self._motors[state.id].feedback[1],
                        torque=state.values[moteus.Register.TORQUE]
                    )
                )
            await asyncio.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    arm_teach_node = ArmTeachNode()
    rclpy.spin(arm_teach_node)


if __name__ == '__main__':
    main()
