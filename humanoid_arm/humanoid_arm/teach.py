import rclpy
from rclpy.node import Node
import rclpy.qos
import moteus
from dataclasses import dataclass
import threading
import asyncio
from typing import Dict
from ament_index_python.packages import get_package_share_directory
import os
from humanoid_interface.srv import TeachArm
import json


@dataclass
class MotorDataClass:
    id: int
    controller: moteus.Controller
    position: float


class ArmTeachNode(Node):
    def __init__(self):
        super().__init__('arm_teach')
        
        self._frames_data_path = os.path.join(get_package_share_directory('humanoid_arm'), 'frames')
        self._teach_service = self.create_service(TeachArm, "arm/teach", self._teach_callback)
        
        self._motors: Dict[int, MotorDataClass] = {}
        for i in range(14, 23):
            self._motors[i] = MotorDataClass(id=i, controller=moteus.Controller(id=i))
        
        self._control_thread = threading.Thread(target=asyncio.run, args=[self._control_loop()])
        self._control_thread.start()
    
    def _save_frame(self, frame_name) -> bool:
        frame_dict = {m.id: m.position for m in self._motors.values()}
        with open(os.path.join(self._frames_data_path, f'{frame_name}.json'), 'w') as fp:
            fp.write(json.dumps(frame_dict, indent=4))
        return True
    
    def _teach_callback(self, request: TeachArm.Request, response: TeachArm.Response) -> TeachArm.Response:
        response.result = self._save_frame(request.frame_name)
        return response
    
    async def _detect_motor(self, motor: MotorDataClass) -> bool:
        s = moteus.Stream(motor.controller)
        try:
            response = await asyncio.wait_for(s.command(b'conf get id.id', allow_any_response=True), 0.1)
            if int(response.decode('utf-8')) == motor.id:
                return True
        except (TimeoutError, ValueError):
            pass
        return False

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
            states = await transport.cycle([c.controller.make_position(velocity=0.0, query=True) for c in self._motors.values()])
            for state in states:
                self._motors[state.id].position = state.values[moteus.Register.POSITION]
            await asyncio.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    arm_teach_node = ArmTeachNode()
    rclpy.spin(arm_teach_node)


if __name__ == '__main__':
    main()
