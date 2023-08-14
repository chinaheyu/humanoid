from typing import Union
import rclpy
import rclpy.qos
from rclpy.node import Node
from rclpy.time import Time
import uvicorn
import threading
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from .api_types import *
from humanoid_interface.msg import MotorControl, MotorFeedback, FaceControl, NeckControl, HeadFeedback


class HumanoidWebNode(Node):
    face_components = [
        "chin_up_down",
        "eyes_up_down",
        "eyes_left_right",
        "left_eyelid_up_down",
        "right_eyelid_up_down",
        "left_eyebrow_up_down",
        "right_eyebrow_up_down",
        "left_cheek_up_down",
        "right_cheek_up_down",
        "left_lip_corner_push_pull",
        "right_lip_corner_push_pull"
    ]
    
    def __init__(self):
        # Initialize node
        super().__init__('humanoid_web')
        
        # Motor feedback
        self.motor_feedback: dict[int, MotorFeedback] = {}
        
        # Head feedback
        self.head_feedback: Union[HeadFeedback, None] = None
        
        # Create ros subsctiption
        self._motor_feedback_subscription = self.create_subscription(MotorFeedback, "motor_feedback", self._motor_feedback_callback, rclpy.qos.QoSPresetProfiles.get_from_short_key("SENSOR_DATA"))
        self._head_feedback_subscription = self.create_subscription(HeadFeedback, "head_feedback", self._head_feedback_callback, rclpy.qos.QoSPresetProfiles.get_from_short_key("SENSOR_DATA"))
        
        # Create ros publisher
        self._motor_control_publisher = self.create_publisher(MotorControl, "motor_control", rclpy.qos.QoSPresetProfiles.get_from_short_key("SYSTEM_DEFAULT"))
        self._face_control_publisher = self.create_publisher(FaceControl, "face_control", rclpy.qos.QoSPresetProfiles.get_from_short_key("SYSTEM_DEFAULT"))
        self._neck_control_publisher = self.create_publisher(NeckControl, "neck_control", rclpy.qos.QoSPresetProfiles.get_from_short_key("SYSTEM_DEFAULT"))
    
    def _motor_feedback_callback(self, msg: MotorFeedback) -> None:
        self.motor_feedback[msg.id] = msg
    
    def _head_feedback_callback(self, msg: HeadFeedback) -> None:
        self.head_feedback = msg
    
    def control_motor(self, msg: MotorControl):
        self._motor_control_publisher.publish(msg)
    
    def control_face(self, msg: FaceControl):
        self._face_control_publisher.publish(msg)
    
    def control_neck(self, msg: NeckControl):
        self._neck_control_publisher.publish(msg)


humanoid_web_node = None
app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/motor")
def list_motors() -> ApiListMotorResponse:
    response = ApiListMotorResponse()
    if humanoid_web_node is not None:
        for motor_feedback in humanoid_web_node.motor_feedback.values():
            response.motors.append(
                ApiMotorFeedbackResponse(
                    timestamp=Time.from_msg(motor_feedback.stamp).nanoseconds / 1000,
                    id=motor_feedback.id,
                    position=motor_feedback.position,
                    velocity=motor_feedback.velocity,
                    torque=motor_feedback.torque
                )
            )
    return response


@app.get("/motor/feedback")
def get_motor_feedback(id: int) -> ApiMotorFeedbackResponse:
    motor_feedback = humanoid_web_node.motor_feedback.get(id)
    if motor_feedback is None:
        raise HTTPException(status_code=404, detail="Motor not found")
    response = ApiMotorFeedbackResponse(
        timestamp=Time.from_msg(motor_feedback.stamp).nanoseconds / 1000,
        id=motor_feedback.id,
        position=motor_feedback.position,
        velocity=motor_feedback.velocity,
        torque=motor_feedback.torque
    )
    return response


@app.put("/motor/control")
def control_motor(id: int, command: ApiControlMotorRequest):
    msg = MotorControl()
    msg.id = command.id
    if command.control_type == ApiControlType.MOTOR_MIT_CONTROL:
        msg.control_type = MotorControl.MOTOR_MIT_CONTROL
    if command.control_type == ApiControlType.MOTOR_POSITION_CONTROL:
        msg.control_type = MotorControl.MOTOR_POSITION_CONTROL
    msg.position = command.position
    msg.velocity = command.velocity
    msg.torque = command.torque
    msg.kp = command.kp
    msg.kd = command.kd
    humanoid_web_node.control_motor(msg)
    return {"message": "Success"}


@app.put("/head/face")
def control_face(command: ApiControlFaceRequest):
    msg = FaceControl()
    for i in HumanoidWebNode.face_components:
        msg.pulse_width[getattr(FaceControl, f'SERVO_{i.upper()}')] = getattr(command, i)
    humanoid_web_node.control_face(msg)
    return {"message": "Success"}


@app.put("/head/neck")
def control_neck(command: ApiControlNeckRequest):
    msg = NeckControl()
    msg.pitch_velocity = command.pitch_velocity
    msg.yaw_angle = command.yaw_angle
    msg.yaw_max_velocity = command.yaw_max_velocity
    humanoid_web_node.control_neck(msg)
    return {"message": "Success"}


@app.get("/head")
def get_head_feedback() -> ApiHeadFeedbackResponse:
    response = ApiHeadFeedbackResponse()
    if humanoid_web_node.head_feedback is None:
        raise HTTPException(status_code=404, detail="Head not found")
    for i in HumanoidWebNode.face_components:
        setattr(response.face, i, int(humanoid_web_node.head_feedback.pulse_width[getattr(FaceControl, f'SERVO_{i.upper()}')]))
    response.neck.pitch_velocity = humanoid_web_node.head_feedback.pitch_velocity
    response.neck.yaw_angle = humanoid_web_node.head_feedback.yaw_angle
    return response


def main(args=None):
    global humanoid_web_node
    rclpy.init(args=args)
    unicorn_thread = threading.Thread(target=uvicorn.run, args=[app], kwargs={"host": "0.0.0.0", "port": 5000, "log_level": "info"})
    unicorn_thread.start()
    humanoid_web_node = HumanoidWebNode()
    rclpy.spin(humanoid_web_node)


if __name__ == '__main__':
    main()
