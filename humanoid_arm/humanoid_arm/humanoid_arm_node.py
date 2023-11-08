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
from humanoid_interface.srv import PlayArm, GetArmFrameList, TeachArm, PlayArmSequence
from std_srvs.srv import SetBool, Empty
from ament_index_python.packages import get_package_share_directory
import os
from .joint_trajectory_planner import FifthOrderTrajectory
import numpy as np
import json
import time
import math


@dataclass
class MotorDataClass:
    id: int
    controller: moteus.Controller
    target: np.ndarray
    feedback: np.ndarray
    reverse: bool = False
    offset: float = 0.0
    initialized: bool = False


class HumanoidArmNode(Node):
    def __init__(self):
        super().__init__('humanoid_arm', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        
        self._teach_mode = True
        
        self._frames_data_path = os.path.join(get_package_share_directory('humanoid_arm'), 'frames')
        self._play_service = self.create_service(PlayArm, "arm/play", self._play_callback)
        self._play_sequence_service = self.create_service(PlayArmSequence, "arm/play_sequence", self._play_sequence_callback)
        self._teach_mode_service = self.create_service(SetBool, "arm/teach_mode", self._teach_mode_callback)
        self._teach_service = self.create_service(TeachArm, "arm/teach", self._teach_callback)
        self._get_frame_list_service = self.create_service(GetArmFrameList, "arm/get_frame_list", self._get_frame_list_callback)
        self._calibration_service = self.create_service(Empty, "arm/calibration", self._calibration_callback)
        
        self._motors: Dict[int, MotorDataClass] = {}
        for i in range(14, 24):
            self._motors[i] = MotorDataClass(id=i, controller=moteus.Controller(id=i), target=np.zeros(3), feedback=np.zeros(3))
        
        self._motor_feedback_publisher = self.create_publisher(MotorFeedback, "motor_feedback", rclpy.qos.QoSPresetProfiles.get_from_short_key("SENSOR_DATA"))
        self._motor_control_subscription = self.create_subscription(MotorControl, "motor_control", self._motor_control_callback, rclpy.qos.QoSPresetProfiles.get_from_short_key("SYSTEM_DEFAULT"))
        
        self._control_thread = threading.Thread(target=asyncio.run, args=[self._control_loop()])
        self._control_thread.start()
    
    def _calibration_callback(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:
        for motor in self._motors.values():
            motor.offset += motor.feedback[0]
        frame_dict = {m.id: m.offset for m in self._motors.values()}
        with open(os.path.join(self._frames_data_path, 'calibration.json'), 'w') as fp:
            fp.write(json.dumps(frame_dict, indent=4))
        return response

    def _save_frame(self, frame_name) -> bool:
        frame_dict = {m.id: m.feedback[0] for m in self._motors.values()}
        with open(os.path.join(self._frames_data_path, f'{frame_name}.json'), 'w') as fp:
            fp.write(json.dumps(frame_dict, indent=4))
        return True
    
    def _teach_callback(self, request: TeachArm.Request, response: TeachArm.Response) -> TeachArm.Response:
        response.result = self._save_frame(request.frame_name)
        return response
    
    def _teach_mode_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self._teach_mode = request.data
        response.success = True
        return response
    
    def _get_frame_list_callback(self, request: GetArmFrameList.Request, response: GetArmFrameList.Response) -> GetArmFrameList.Response:
        response.frames = []
        for f in os.listdir(self._frames_data_path):
            if f.endswith('.json'):
                response.frames.append(f[:-5])
        return response
    
    def _play_to_frame(self, frame_name: str, duration: float) -> bool:
        try:
            with open(os.path.join(self._frames_data_path, f'{frame_name}.json'), 'r') as fp:
                frame_dict = json.load(fp)
        except RuntimeError:
            return False
        motor_id = list(self._motors.keys())
        traj = FifthOrderTrajectory(
            np.column_stack([self._motors[i].feedback for i in motor_id]),
            np.vstack([
                [frame_dict[str(i)] for i in motor_id],
                np.zeros((2, len(motor_id)))
            ]),
            duration
        )
        st = time.time()
        while (t := (time.time() - st)) < duration:
            p = traj.plan(t)
            for i in range(len(motor_id)):
                self._motors[motor_id[i]].target = p[:, i]
            time.sleep(0.02)
        return True

    def _play_sequence_callback(self, request: PlayArmSequence.Request, response: PlayArmSequence.Response) -> PlayArmSequence.Response:
        for frame_name, duration in zip(request.frame_name, request.duration):
            if not self._play_to_frame(frame_name, duration):
                response.result = False
                return response
        response.result = True
        return response
    
    def _play_callback(self, request: PlayArm.Request, response: PlayArm.Response) -> PlayArm.Response:
        response.result = self._play_to_frame(request.frame_name, request.duration)
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

        # Create transport
        transport = moteus.Fdcanusb()
        
        # initialize motors
        while rclpy.ok():
            try:
                states = await asyncio.wait_for(transport.cycle([c.controller.make_stop(query=True) for c in self._motors.values()]), 0.1)
            except asyncio.exceptions.TimeoutError:
                self.get_logger().warning('Moteus send command timeout.')
            else:
                for state in states:
                    self._motors[state.id].initialized = True
                if all([m.initialized for m in self._motors.values()]):
                    break

        # control loop
        timeout_counter = 0
        while rclpy.ok():
            try:
                # Send command
                if self._teach_mode:
                    states = await asyncio.wait_for(transport.cycle([c.controller.make_brake(query=True) for c in self._motors.values()]), 0.1)
                else:
                    states = await asyncio.wait_for(transport.cycle([
                        c.controller.make_position(
                            position=((-c.target[0] if c.reverse else c.target[0]) + c.offset) / (2 * np.pi),
                            velocity=c.target[1] / (2 * np.pi),
                            maximum_torque=8.0,
                            query=True
                        )
                        for c in self._motors.values()
                    ]), 0.1)
            except asyncio.exceptions.TimeoutError:
                timeout_counter += 1
                self.get_logger().warning(f'Moteus send command timeout {timeout_counter}.')
                if timeout_counter > 10:
                    self.get_logger().error('Moteus timeout too many times, exit.')
                    break
            else:
                timeout_counter = 0
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
