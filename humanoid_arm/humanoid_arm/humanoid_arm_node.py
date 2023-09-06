import rclpy
from rclpy.node import Node
import rclpy.qos
import moteus
from dataclasses import dataclass
import threading
import asyncio
from humanoid_interface.msg import MotorControl, MotorFeedback
from typing import Dict
from humanoid_interface.srv import PlayArm
from ament_index_python.packages import get_package_share_directory
import os
from joint_trajectory_planner import FifthOrderTrajectory
import numpy as np
import json
import time


@dataclass
class MotorDataClass:
    id: int
    controller: moteus.Controller
    target: np.ndarray = np.zeros(3)
    feedback: np.ndarray = np.zeros(3)


class HumanoidArmNode(Node):
    def __init__(self):
        super().__init__('humanoid_arm')
        
        self._frames_data_path = os.path.join(get_package_share_directory('humanoid_arm'), 'frames')
        self._play_service = self.create_service(PlayArm, "arm/play", self._play_callback)
        
        self._motors: Dict[int, MotorDataClass] = {}
        for i in range(14, 23):
            self._motors[i] = MotorDataClass(id=i, controller=moteus.Controller(id=i))
        
        self._motor_feedback_publisher = self.create_publisher(MotorFeedback, "motor_feedback", rclpy.qos.QoSPresetProfiles.get_from_short_key("SENSOR_DATA"))
        self._motor_control_subscription = self.create_subscription(MotorControl, "motor_control", self._motor_control_callback, rclpy.qos.QoSPresetProfiles.get_from_short_key("SYSTEM_DEFAULT"))
        
        self._control_thread = threading.Thread(target=asyncio.run, args=[self._control_loop()])
        self._control_thread.start()
    
    def _play_callback(self, request: PlayArm.Request, response: PlayArm.Response) -> PlayArm.Response:
        try:
            with open(os.path.join(self._frames_data_path, f'{request.frame_name}.json'), 'r') as fp:
                frame_dict = json.load(fp)
        except RuntimeError:
            response.result = False
            return response
        motor_id = self._motors.keys()
        traj = FifthOrderTrajectory(
            np.column_stack([self._motors[i].feedback for i in motor_id]),
            np.array(
                [frame_dict[i] for i in motor_id],
                np.zeros(2, len(motor_id))
            ),
            request.duration
        )
        st = time.time()
        while (t := time.time()) < request.duration:
            p = traj.plan(t - st)
            for i in range(len(motor_id)):
                self._motors[motor_id[i]].target = p[:, i]
            time.sleep(0.02)
        response.result = True
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

    def _motor_control_callback(self, msg: MotorControl) -> None:
        if msg.id in self._motors and msg.control_type == MotorControl.MOTOR_POSITION_CONTROL:
            self._motors[msg.id].target[0] = msg.position

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
            states = await transport.cycle([c.controller.make_position(position=c.target[0], velocity=c.target[1], query=True) for c in self._motors.values()])
            for state in states:
                self._motors[state.id].feedback[0] = state.values[moteus.Register.POSITION]
                self._motors[state.id].feedback[1] = state.values[moteus.Register.VELOCITY]
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
