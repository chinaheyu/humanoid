from pydantic import BaseModel
from typing import Union, List
from enum import Enum


class ApiMotorFeedbackResponse(BaseModel):
    timestamp: int
    id: int
    position: float
    velocity: float
    torque: float
    
    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "timestamp": 1691299234198467,
                    "id": 1,
                    "position": 1.5707963267948966,
                    "velocity": 3.141592653589793,
                    "torque": 6.283185307179586
                }
            ]
        }
    }


class ApiListMotorResponse(BaseModel):
    motors: List[ApiMotorFeedbackResponse] = []
    
    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "motors": [
                        {
                            "timestamp": 1691299234198467,
                            "id": 1,
                            "position": 1.5707963267948966,
                            "velocity": 3.141592653589793,
                            "torque": 6.283185307179586
                        },
                        {
                            "timestamp": 1691299234198467,
                            "id": 2,
                            "position": 1.5707963267948966,
                            "velocity": 3.141592653589793,
                            "torque": 6.283185307179586
                        }
                    ]
                }
            ]
        }
    }


class ApiControlType(str, Enum):
    MOTOR_MIT_CONTROL = "MOTOR_MIT_CONTROL"
    MOTOR_POSITION_CONTROL = "MOTOR_POSITION_CONTROL"


class ApiControlMotorRequest(BaseModel):
    id: int
    control_type: ApiControlType
    position: float
    velocity: float
    torque: float
    kp: float
    kd: float

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "id": 1,
                    "control_type": ApiControlType.MOTOR_MIT_CONTROL,
                    "position":  1.5707963267948966,
                    "velocity": 3.141592653589793,
                    "torque": 6.283185307179586,
                    "kp": 3.0,
                    "kd": 1.0
                }
            ]
        }
    }


class ApiFaceComponents(BaseModel):
    chin_up_down: int = 1000
    eyes_up_down: int = 1500
    eyes_left_right: int = 1500
    left_eyelid_up_down: int = 1500
    right_eyelid_up_down: int = 1500
    left_eyebrow_up_down: int = 1500
    right_eyebrow_up_down: int = 1500
    left_cheek_up_down: int = 1500
    right_cheek_up_down: int = 1500
    left_lip_corner_push_pull: int = 1500
    right_lip_corner_push_pull: int = 1500


class ApiNeckComponents(BaseModel):
    pitch_velocity: float = 0.0
    yaw_angle: float = 0.0


class ApiControlFaceRequest(ApiFaceComponents):
    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "chin_up_down": 1000,
                    "eyes_up_down": 1500,
                    "eyes_left_right": 1500,
                    "left_eyelid_up_down": 1500,
                    "right_eyelid_up_down": 1500,
                    "left_eyebrow_up_down": 1500,
                    "right_eyebrow_up_down": 1500,
                    "left_cheek_up_down": 1500,
                    "right_cheek_up_down": 1500,
                    "left_lip_corner_push_pull": 1500,
                    "right_lip_corner_push_pull": 1500
                }
            ]
        }
    }


class ApiControlNeckRequest(ApiNeckComponents):
    yaw_max_velocity: float
    
    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "pitch_velocity": 0.0,
                    "yaw_angle": 0.0,
                    "yaw_max_velocity": 3.14
                }
            ]
        }
    }


class ApiHeadFeedbackResponse(BaseModel):
    face: ApiFaceComponents = ApiFaceComponents()
    neck: ApiNeckComponents = ApiNeckComponents()
    
    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "face": {
                        "chin_up_down": 1000,
                        "eyes_up_down": 1500,
                        "eyes_left_right": 1500,
                        "left_eyelid_up_down": 1500,
                        "right_eyelid_up_down": 1500,
                        "left_eyebrow_up_down": 1500,
                        "right_eyebrow_up_down": 1500,
                        "left_cheek_up_down": 1500,
                        "right_cheek_up_down": 1500,
                        "left_lip_corner_push_pull": 1500,
                        "right_lip_corner_push_pull": 1500
                    },
                    "neck": {
                        "pitch_velocity": 0.0,
                        "yaw_angle": 0.0
                    }
                }
            ]
        }
    }


class ApiPlayArmRequest(BaseModel):
    frame_name: str
    duration: float
    
    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "frame_name": 'home',
                    "duration": 1.0
                }
            ]
        }
    }


class ApiGetArmFramesResponse(BaseModel):
    frames: List[str]
    
    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "frames": ["home", "frame1", "frame2"]
                }
            ]
        }
    }
