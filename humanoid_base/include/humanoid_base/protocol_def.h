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

#pragma pack(pop)


#endif //PROTOCOL_PROTOCOL_DEF_H
