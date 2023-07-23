from typing import Union
import rclpy
import rclpy.qos
from rclpy.node import Node
import uvicorn
import threading
from fastapi import FastAPI, HTTPException
from .api_types import *
from humanoid_interface.msg import MotorControl, MotorFeedback


class HumanoidWebNode(Node):
    def __init__(self):
        # Initialize node
        super().__init__('humanoid_web')
        
        # Motor feedback
        self.motor_feedback: dict[int, MotorFeedback] = {}
        
        # Create ros subsctiption
        self._motor_feedback_subscription = self.create_subscription(MotorFeedback, "motor_feedback", self._motor_feedback_callback, rclpy.qos.qos_profile_sensor_data)
        
        # Create ros publisher
        self._motor_control_publisher = self.create_publisher(MotorControl, "motor_control", rclpy.qos.qos_profile_system_default)
    
    def _motor_feedback_callback(self, msg: MotorFeedback) -> None:
        self._motor_feedback[msg.id] = msg
    
    def control_motor(self, msg: MotorControl):
        self._motor_control_publisher.publish(msg)


humanoid_web_node = None
app = FastAPI()


@app.get("/motor")
def list_motors() -> ApiListMotorResponse:
    response = ApiListMotorResponse()
    if humanoid_web_node is not None:
        for motor_feedback in humanoid_web_node.motor_feedback.values():
            response.motors.append(
                ApiMotorFeedbackResponse(
                    timestamp=motor_feedback.stamp,
                    id=motor_feedback.id,
                    position=motor_feedback.position,
                    velocity=motor_feedback.velocity,
                    torque=motor_feedback.torque
                )
            )
    return response


@app.get("/motor/{id}")
def get_motor_feedback(id: int):
    motor_feedback = humanoid_web_node.motor_feedback.get(id)
    if motor_feedback is None:
        raise HTTPException(status_code=404, detail="Motor not found")
    response = ApiMotorFeedbackResponse(
        timestamp=motor_feedback.stamp,
        id=motor_feedback.id,
        position=motor_feedback.position,
        velocity=motor_feedback.velocity,
        torque=motor_feedback.torque
    )
    return response


@app.put("/motor/{id}")
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


def main(args=None):
    global humanoid_web_node
    rclpy.init(args=args)
    unicorn_thread = threading.Thread(target=uvicorn.run, args=[app], kwargs={"port": 5000, "log_level": "info"})
    unicorn_thread.start()
    humanoid_web_node = HumanoidWebNode()
    rclpy.spin(humanoid_web_node)


if __name__ == '__main__':
    main()


