import rclpy
from rclpy.node import Node
import rclpy.qos
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
import moteus
from dataclasses import dataclass
import threading
import asyncio
from humanoid_interface.msg import MotorControl, MotorFeedback
from typing import Dict, List
from humanoid_interface.srv import PlayArm
from ament_index_python.packages import get_package_share_directory
import os
from .joint_trajectory_planner import FifthOrderTrajectory
import numpy as np
import json
import time


@dataclass
class MotorDataClass:
    id: int
    controller: moteus.Controller
    target: np.ndarray
    feedback: np.ndarray
    reverse: bool = False
    offset: float = 0.0


class HumanoidArmNode(Node):
    def __init__(self):
        super().__init__('humanoid_arm', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        
        self._frames_data_path = os.path.join(get_package_share_directory('humanoid_arm'), 'frames')
        self._play_service = self.create_service(PlayArm, "arm/play", self._play_callback)
        
        self._motors: Dict[int, MotorDataClass] = {}
        for i in range(14, 24):
            self._motors[i] = MotorDataClass(id=i, controller=moteus.Controller(id=i), target=np.zeros(3), feedback=np.zeros(3))
        
        self.add_on_set_parameters_callback(self._parameter_callback)
        self._load_parameter()
        
        self._motor_feedback_publisher = self.create_publisher(MotorFeedback, "motor_feedback", rclpy.qos.QoSPresetProfiles.get_from_short_key("SENSOR_DATA"))
        self._motor_control_subscription = self.create_subscription(MotorControl, "motor_control", self._motor_control_callback, rclpy.qos.QoSPresetProfiles.get_from_short_key("SYSTEM_DEFAULT"))
        
        self._control_thread = threading.Thread(target=asyncio.run, args=[self._control_loop()])
        self._control_thread.start()
    
    def _load_parameter(self):
        for param_name in self.get_parameters_by_prefix('motors'):
            param = self.get_parameter(f'motors.{param_name}')
            self.get_logger().info(f'Load parameter {param.name} = {param.value}')
            motor_id = int(param.name.split('.')[1].split('_')[1])
            if motor_id in self._motors:
                if param.name.endswith('offset'):
                    self._motors[motor_id].offset = param.value
                if param.name.endswith('reverse'):
                    self._motors[motor_id].reverse = param.value
        
    def _parameter_callback(self, params: List[Parameter]) -> SetParametersResult:
        for param in params:
            if param.name.startswith('motors'):
                motor_id = int(param.name.split('.')[1].split('_')[1])
                if motor_id in self._motors:
                    if param.name.endswith('offset'):
                        self._motors[motor_id].offset = param.value
                    if param.name.endswith('reverse'):
                        self._motors[motor_id].reverse = param.value
        return SetParametersResult(successful=True)
    
    def _play_callback(self, request: PlayArm.Request, response: PlayArm.Response) -> PlayArm.Response:
        try:
            with open(os.path.join(self._frames_data_path, f'{request.frame_name}.json'), 'r') as fp:
                frame_dict = json.load(fp)
        except RuntimeError:
            response.result = False
            return response
        motor_id = list(self._motors.keys())
        traj = FifthOrderTrajectory(
            np.column_stack([self._motors[i].feedback for i in motor_id]),
            np.vstack([
                [frame_dict[str(i)] for i in motor_id],
                np.zeros((2, len(motor_id)))
            ]),
            request.duration
        )
        st = time.time()
        while (t := (time.time() - st)) < request.duration:
            p = traj.plan(t)
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
        except (asyncio.exceptions.TimeoutError, ValueError):
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
            try:
                # Send command
                states = await asyncio.wait_for(transport.cycle([
                    c.controller.make_position(
                        position=((-c.target[0] if c.reverse else c.target[0]) + c.offset) / (2 * np.pi),
                        velocity=c.target[1] / (2 * np.pi),
                        maximum_torque=4.0,
                        query=True
                    )
                    for c in self._motors.values()
                ]), 0.1)
            except asyncio.exceptions.TimeoutError:
                self.get_logger().error('Moteus send command timeout.')
            else:
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

            # wait for 20ms
            await asyncio.sleep(0.02)

        # Stop motors
        states = transport.cycle([
            c.controller.make_stop(
                query=True
            )
            for c in self._motors.values()
        ])


def main(args=None):
    rclpy.init(args=args)
    humanoid_arm_node = HumanoidArmNode()
    rclpy.spin(humanoid_arm_node)


if __name__ == '__main__':
    main()
