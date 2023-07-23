uint8 MOTOR_MIT_CONTROL=0
uint8 MOTOR_POSITION_CONTROL=1

uint8 id                # motor global id
uint8 control_type      # MOTOR_MIT_CONTROL or MOTOR_POSITION_CONTROL
float32 position        # motor position (rad)
float32 velocity        # motor velocity (rad / s)
float32 torque          # motor torque (Nm)
float32 kp              # position pid gain
float32 kd              # velocity pid gain