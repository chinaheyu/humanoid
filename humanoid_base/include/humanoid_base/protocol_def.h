#ifndef PROTOCOL_PROTOCOL_DEF_H
#define PROTOCOL_PROTOCOL_DEF_H

#define PROTOCOL_DYNAMIC_BUFFER 1
#define PROTOCOL_USING_STDLIB 1
#define PROTOCOL_USING_STRING 1

#pragma pack(push)
#pragma pack(1)

#define CMD_SYNC (0x0001)
typedef struct 
{
    long long timestamp;    // microseconds
} cmd_sync_t;

#define CMD_RESET (0x0002)

#define CMD_READ_APP_ID (0x0003)

#define CMD_INITIALIZE_MOTOR (0x0004)

#define CMD_GYRO_FEEDBACK (0x0101)
typedef struct 
{
    long long timestamp;
    short roll;
    short pitch;
    short yaw;
} cmd_gyro_feedback_t;

#define CMD_MOTOR_FEEDBACK (0x0102)
typedef struct 
{
    long long timestamp;
    unsigned char id;
    short position;
    short velocity;
    short torque;
} cmd_motor_feedback_t;

#define CMD_HEAD_FEEDBACK (0x0103)
typedef struct 
{
    long long timestamp;
    unsigned short pulse_width[11];
    short pitch_velocity;
    short yaw_angle;
} cmd_head_feedback_t;

#define CMD_READ_APP_ID_FEEDBACK (0x0104)
typedef struct 
{
    long long timestamp;
    unsigned char app_id;   // 0: TEST, 1: HEAD_CONTROL, 2: LEFT_LEG_CONTROL, 3: RIGHT_LEG_CONTROL, 4: WAIST_CONTROL
} cmd_read_app_id_feedback_t;

#define CMD_MOTOR_MIT (0x0201)
typedef struct 
{
    unsigned char id;
    short position;
    short velocity;
    short kp;
    short kd;
    short torque;
} cmd_motor_mit_t;

#define CMD_MOTOR_POSITION (0x0202)
typedef struct 
{
    unsigned char id;
    short position;
} cmd_motor_position_t;

#define CMD_HEAD_SERVO (0x0203)
typedef struct 
{
    unsigned short pulse_width[11];     // 500 ~ 2500
} cmd_head_servo_t;

#define CMD_NECK_MOTOR (0x0204)
typedef struct 
{
    short pitch_velocity;
    short yaw_angle;
    short yaw_max_velocity;
} cmd_neck_motor_t;

#pragma pack(pop)


#endif //PROTOCOL_PROTOCOL_DEF_H
