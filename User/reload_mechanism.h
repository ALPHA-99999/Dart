/**
 ******************************************************************************
 * @file           : reload_mechanism.h
 * @brief          : 换弹机构控制模块 - 头文件
 ******************************************************************************
 */

#ifndef RELOAD_MECHANISM_H_
#define RELOAD_MECHANISM_H_

#include <stdint.h>

#include "motor.h"
#include "pid.h"
#include "driver_can.h"

typedef enum
{
    RELOAD_2006_STOP = 0,
    RELOAD_2006_DOWN,
    RELOAD_2006_UP
} Reload2006State_e;

typedef enum
{
    RELOAD_6020_POS_0 = 0,
    RELOAD_6020_POS_P90,
    RELOAD_6020_POS_N90
} Reload6020Pose_e;

typedef enum
{
    RELOAD_SERVO_CLOSE = 0,
    RELOAD_SERVO_OPEN
} ReloadServoPose_e;

typedef struct
{
    Motor motor_2006;
    Motor motor_6020;

    BasePID_Object pid_2006_speed;
    BasePID_Object pid_2006_angle;
    BasePID_Object pid_6020_speed;
    BasePID_Object pid_6020_angle;

    float target_angle_2006;
    float target_angle_6020;

    float base_angle_2006;
    float base_angle_6020;
    float down_offset_2006;
    float up_offset_2006;

    uint8_t servo_id;
    uint16_t servo_pos_close;
    uint16_t servo_pos_open;
    uint16_t servo_target_pos;
    uint16_t servo_move_time;
    uint8_t servo_pending;

    Reload2006State_e state_2006;
    Reload6020Pose_e pose_6020;
    ReloadServoPose_e pose_servo;

    uint8_t sync_enable;
    uint8_t sync_stage;
    uint32_t sync_tick_ms;

    uint8_t enabled;
    int32_t output_limit_2006;
    int32_t output_limit_6020;
} ReloadMechanism_t;

extern ReloadMechanism_t g_reload;

void ReloadMechanism_Init(void);
void ReloadMechanism_Reset(void);
void ReloadMechanism_Control(void);
void ReloadMechanism_Enable(uint8_t enable);

void ReloadMechanism_Config2006Motor(CanNumber canx, uint16_t motor_id, uint16_t ecd_offset);
void ReloadMechanism_Config6020Motor(CanNumber canx, uint16_t motor_id, uint16_t ecd_offset);

void ReloadMechanism_Config2006PID(float speed_kp, float speed_ki, float speed_kd,
                                   float angle_kp, float angle_ki, float angle_kd);
void ReloadMechanism_Config6020PID(float speed_kp, float speed_ki, float speed_kd,
                                   float angle_kp, float angle_ki, float angle_kd);

void ReloadMechanism_Set2006State(Reload2006State_e state);
void ReloadMechanism_Set6020Pose(Reload6020Pose_e pose);
void ReloadMechanism_SetServoPose(ReloadServoPose_e pose);

void ReloadMechanism_Set2006Offset(float down_offset, float up_offset);
void ReloadMechanism_SetServoPosition(uint16_t close_pos, uint16_t open_pos);

void ReloadMechanism_StartCoopSequence(void);
void ReloadMechanism_StopCoopSequence(void);
/* 业务动作API: 2006三状态 */
void ReloadMechanism_2006Stop(void);
void ReloadMechanism_2006Down(void);
void ReloadMechanism_2006Up(void);

/* 业务动作API: 6020三档位 */
void ReloadMechanism_6020ToZero(void);
void ReloadMechanism_6020ToPos90(void);
void ReloadMechanism_6020ToNeg90(void);

/* 业务动作API: 舵机两档位 */
void ReloadMechanism_ServoClose(void);
void ReloadMechanism_ServoOpen(void);

/* 协同动作API: 一键启动换弹时序骨架 */
void ReloadMechanism_TriggerReload(void);

float ReloadMechanism_Get2006Angle(void);
float ReloadMechanism_Get6020Angle(void);

#endif /* RELOAD_MECHANISM_H_ */

