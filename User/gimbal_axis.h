/**
 ******************************************************************************
 * @file           : gimbal_axis.h
 * @brief          : 云台Pitch/Yaw/Stretch三轴丝杆控制模块 - 头文件
 ******************************************************************************
 */

#ifndef GIMBAL_AXIS_H_
#define GIMBAL_AXIS_H_

#include <stdint.h>

#include "motor.h"
#include "pid.h"
#include "driver_can.h"

typedef enum
{
    GIMBAL_AXIS_PITCH = 0,
    GIMBAL_AXIS_YAW,
    GIMBAL_AXIS_STRETCH
} GimbalAxis_e;

typedef enum
{
    GIMBAL_CTRL_SPEED = 0,
    GIMBAL_CTRL_ANGLE
} GimbalCtrlMode_e;

typedef enum
{
    GIMBAL_STATE_IDLE = 0,
    GIMBAL_STATE_MOVING,
    GIMBAL_STATE_HOLDING,
    GIMBAL_STATE_ERROR
} GimbalAxisState_e;

typedef struct
{
    Motor motor;
    BasePID_Object speed_pid;
    BasePID_Object angle_pid;

    GimbalCtrlMode_e mode;
    GimbalAxisState_e state;

    int16_t target_speed;
    float target_angle;

    uint8_t enabled;
    uint16_t jam_count;
    float jam_speed_diff;
    uint16_t jam_threshold;
    int32_t output_limit;
} GimbalAxisMotor_t;

typedef struct
{
    GimbalAxisMotor_t pitch;
    GimbalAxisMotor_t yaw;
    GimbalAxisMotor_t stretch;
    uint8_t enabled;
} GimbalAxisController_t;

extern GimbalAxisController_t g_gimbal_axis;

void GimbalAxis_Init(void);
void GimbalAxis_Reset(void);
void GimbalAxis_Control(void);
void GimbalAxis_Enable(uint8_t enable);
void GimbalAxis_EnableSingle(GimbalAxis_e axis, uint8_t enable);

void GimbalAxis_ConfigMotor(GimbalAxis_e axis, CanNumber canx, uint16_t motor_id, uint16_t ecd_offset);
void GimbalAxis_ConfigPID(GimbalAxis_e axis,
                          float speed_kp, float speed_ki, float speed_kd,
                          float angle_kp, float angle_ki, float angle_kd);
void GimbalAxis_SetJamThreshold(GimbalAxis_e axis, float speed_diff, uint16_t count_threshold);
void GimbalAxis_SetOutputLimit(GimbalAxis_e axis, int32_t output_limit);

void GimbalAxis_SetMode(GimbalAxis_e axis, GimbalCtrlMode_e mode);
void GimbalAxis_SetSpeed(GimbalAxis_e axis, int16_t speed);
void GimbalAxis_SetAngle(GimbalAxis_e axis, float angle);
void GimbalAxis_Stop(GimbalAxis_e axis);

void GimbalAxis_MovePitch(float angle);
void GimbalAxis_MoveYaw(float angle);
void GimbalAxis_MoveStretch(float angle);
void GimbalAxis_StopPitch(void);
void GimbalAxis_StopYaw(void);
void GimbalAxis_StopStretch(void);

float GimbalAxis_GetAngle(GimbalAxis_e axis);
int16_t GimbalAxis_GetSpeed(GimbalAxis_e axis);
GimbalAxisState_e GimbalAxis_GetState(GimbalAxis_e axis);

#endif /* GIMBAL_AXIS_H_ */
