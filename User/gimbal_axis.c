/**
 ******************************************************************************
 * @file           : gimbal_axis.c
 * @brief          : 云台Pitch/Yaw/Stretch三轴丝杆控制模块 - 实现文件
 ******************************************************************************
 */

#include "gimbal_axis.h"

#ifndef LIMIT
#define LIMIT(x, max, min) (((x) > (max)) ? (max) : (((x) < (min)) ? (min) : (x)))
#endif

#ifndef ABS
#define ABS(x) ((x) > 0 ? (x) : (-(x)))
#endif

GimbalAxisController_t g_gimbal_axis;

static GimbalAxisMotor_t *GimbalAxis_GetAxis(GimbalAxis_e axis)
{
    if (axis == GIMBAL_AXIS_YAW)
    {
        return &g_gimbal_axis.yaw;
    }
    if (axis == GIMBAL_AXIS_STRETCH)
    {
        return &g_gimbal_axis.stretch;
    }
    return &g_gimbal_axis.pitch;
}

static void GimbalAxis_SpeedControl(GimbalAxisMotor_t *axis_obj)
{
    axis_obj->motor.Data.Target = axis_obj->target_speed;
    axis_obj->motor.Data.Output = BasePID_SpeedControl(&axis_obj->speed_pid,
                                                       axis_obj->motor.Data.Target,
                                                       axis_obj->motor.Data.SpeedRPM);

    axis_obj->motor.Data.Output = LIMIT(axis_obj->motor.Data.Output,
                                        axis_obj->output_limit,
                                        -axis_obj->output_limit);
}

static void GimbalAxis_AngleControl(GimbalAxisMotor_t *axis_obj)
{
    int32_t speed_out = BasePID_AngleControl(&axis_obj->angle_pid,
                                             axis_obj->target_angle,
                                             axis_obj->motor.Data.TotalAngle);

    axis_obj->motor.Data.Target = speed_out;
    axis_obj->motor.Data.Output = BasePID_SpeedControl(&axis_obj->speed_pid,
                                                       axis_obj->motor.Data.Target,
                                                       axis_obj->motor.Data.SpeedRPM);

    axis_obj->motor.Data.Output = LIMIT(axis_obj->motor.Data.Output,
                                        axis_obj->output_limit,
                                        -axis_obj->output_limit);
}

static void GimbalAxis_CheckJam(GimbalAxisMotor_t *axis_obj)
{
    if (axis_obj->state != GIMBAL_STATE_MOVING)
    {
        axis_obj->jam_count = 0;
        return;
    }

    if (ABS(axis_obj->motor.Data.Target - axis_obj->motor.Data.SpeedRPM) > axis_obj->jam_speed_diff)
    {
        axis_obj->jam_count++;
    }
    else
    {
        axis_obj->jam_count = 0;
    }

    if (axis_obj->jam_count >= axis_obj->jam_threshold)
    {
        axis_obj->jam_count = 0;
        axis_obj->state = GIMBAL_STATE_ERROR;
        axis_obj->motor.Data.Output = 0;
    }
}

static void GimbalAxis_ControlSingle(GimbalAxisMotor_t *axis_obj)
{
    if (!axis_obj->enabled)
    {
        axis_obj->motor.Data.Output = 0;
        axis_obj->state = GIMBAL_STATE_IDLE;
        return;
    }

    if (axis_obj->state == GIMBAL_STATE_ERROR)
    {
        axis_obj->motor.Data.Output = 0;
        return;
    }

    if (axis_obj->mode == GIMBAL_CTRL_SPEED)
    {
        axis_obj->state = (axis_obj->target_speed != 0) ? GIMBAL_STATE_MOVING : GIMBAL_STATE_IDLE;
        GimbalAxis_SpeedControl(axis_obj);
    }
    else
    {
        axis_obj->state = GIMBAL_STATE_MOVING;
        GimbalAxis_AngleControl(axis_obj);
    }

    GimbalAxis_CheckJam(axis_obj);
}

void GimbalAxis_Init(void)
{
    MotorInit(&g_gimbal_axis.pitch.motor, 0, Motor2006, CAN2, 0x204);
    MotorInit(&g_gimbal_axis.yaw.motor, 0, Motor2006, CAN2, 0x202);
    MotorInit(&g_gimbal_axis.stretch.motor, 0, Motor2006, CAN2, 0x201);

    BasePID_Init(&g_gimbal_axis.pitch.speed_pid, 15.0f, 0.0f, 0.0f, 0.0f);
    BasePID_Init(&g_gimbal_axis.pitch.angle_pid, 1.0f, 0.0f, 0.0f, 0.0f);
    BasePID_Init(&g_gimbal_axis.yaw.speed_pid, 15.0f, 0.0f, 0.0f, 0.0f);
    BasePID_Init(&g_gimbal_axis.yaw.angle_pid, 1.0f, 0.0f, 0.0f, 0.0f);
    BasePID_Init(&g_gimbal_axis.stretch.speed_pid, 15.0f, 0.0f, 0.0f, 0.0f);
    BasePID_Init(&g_gimbal_axis.stretch.angle_pid, 1.0f, 0.0f, 0.0f, 0.0f);

    g_gimbal_axis.enabled = 1;

    g_gimbal_axis.pitch.mode = GIMBAL_CTRL_ANGLE;
    g_gimbal_axis.yaw.mode = GIMBAL_CTRL_ANGLE;
    g_gimbal_axis.stretch.mode = GIMBAL_CTRL_ANGLE;

    g_gimbal_axis.pitch.state = GIMBAL_STATE_IDLE;
    g_gimbal_axis.yaw.state = GIMBAL_STATE_IDLE;
    g_gimbal_axis.stretch.state = GIMBAL_STATE_IDLE;

    g_gimbal_axis.pitch.target_speed = 0;
    g_gimbal_axis.yaw.target_speed = 0;
    g_gimbal_axis.stretch.target_speed = 0;

    g_gimbal_axis.pitch.target_angle = 0.0f;
    g_gimbal_axis.yaw.target_angle = 0.0f;
    g_gimbal_axis.stretch.target_angle = 0.0f;

    g_gimbal_axis.pitch.enabled = 1;
    g_gimbal_axis.yaw.enabled = 1;
    g_gimbal_axis.stretch.enabled = 1;

    g_gimbal_axis.pitch.jam_count = 0;
    g_gimbal_axis.yaw.jam_count = 0;
    g_gimbal_axis.stretch.jam_count = 0;

    g_gimbal_axis.pitch.jam_speed_diff = 250.0f;
    g_gimbal_axis.yaw.jam_speed_diff = 250.0f;
    g_gimbal_axis.stretch.jam_speed_diff = 250.0f;

    g_gimbal_axis.pitch.jam_threshold = 120;
    g_gimbal_axis.yaw.jam_threshold = 120;
    g_gimbal_axis.stretch.jam_threshold = 120;

    g_gimbal_axis.pitch.output_limit = CURRENT_LIMIT_FOR_2006 / 2;
    g_gimbal_axis.yaw.output_limit = CURRENT_LIMIT_FOR_2006 / 2;
    g_gimbal_axis.stretch.output_limit = CURRENT_LIMIT_FOR_2006 / 2;
}

void GimbalAxis_Reset(void)
{
    g_gimbal_axis.pitch.state = GIMBAL_STATE_IDLE;
    g_gimbal_axis.yaw.state = GIMBAL_STATE_IDLE;
    g_gimbal_axis.stretch.state = GIMBAL_STATE_IDLE;

    g_gimbal_axis.pitch.target_speed = 0;
    g_gimbal_axis.yaw.target_speed = 0;
    g_gimbal_axis.stretch.target_speed = 0;

    g_gimbal_axis.pitch.target_angle = g_gimbal_axis.pitch.motor.Data.TotalAngle;
    g_gimbal_axis.yaw.target_angle = g_gimbal_axis.yaw.motor.Data.TotalAngle;
    g_gimbal_axis.stretch.target_angle = g_gimbal_axis.stretch.motor.Data.TotalAngle;

    g_gimbal_axis.pitch.jam_count = 0;
    g_gimbal_axis.yaw.jam_count = 0;
    g_gimbal_axis.stretch.jam_count = 0;

    g_gimbal_axis.pitch.motor.Data.Output = 0;
    g_gimbal_axis.yaw.motor.Data.Output = 0;
    g_gimbal_axis.stretch.motor.Data.Output = 0;
}

void GimbalAxis_Control(void)
{
    if (!g_gimbal_axis.enabled)
    {
        g_gimbal_axis.pitch.motor.Data.Output = 0;
        g_gimbal_axis.yaw.motor.Data.Output = 0;
        g_gimbal_axis.stretch.motor.Data.Output = 0;
        return;
    }

    GimbalAxis_ControlSingle(&g_gimbal_axis.pitch);
    GimbalAxis_ControlSingle(&g_gimbal_axis.yaw);
    GimbalAxis_ControlSingle(&g_gimbal_axis.stretch);
}

void GimbalAxis_Enable(uint8_t enable)
{
    g_gimbal_axis.enabled = enable;
    if (!enable)
    {
        GimbalAxis_Reset();
    }
}

void GimbalAxis_EnableSingle(GimbalAxis_e axis, uint8_t enable)
{
    GimbalAxisMotor_t *axis_obj = GimbalAxis_GetAxis(axis);
    axis_obj->enabled = enable;
    if (!enable)
    {
        axis_obj->motor.Data.Output = 0;
        axis_obj->state = GIMBAL_STATE_IDLE;
    }
}

void GimbalAxis_ConfigMotor(GimbalAxis_e axis, CanNumber canx, uint16_t motor_id, uint16_t ecd_offset)
{
    GimbalAxisMotor_t *axis_obj = GimbalAxis_GetAxis(axis);
    MotorInit(&axis_obj->motor, ecd_offset, Motor2006, canx, motor_id);
}

void GimbalAxis_ConfigPID(GimbalAxis_e axis,
                          float speed_kp, float speed_ki, float speed_kd,
                          float angle_kp, float angle_ki, float angle_kd)
{
    GimbalAxisMotor_t *axis_obj = GimbalAxis_GetAxis(axis);
    BasePID_Init(&axis_obj->speed_pid, speed_kp, speed_ki, speed_kd, 0.0f);
    BasePID_Init(&axis_obj->angle_pid, angle_kp, angle_ki, angle_kd, 0.0f);
}

void GimbalAxis_SetJamThreshold(GimbalAxis_e axis, float speed_diff, uint16_t count_threshold)
{
    GimbalAxisMotor_t *axis_obj = GimbalAxis_GetAxis(axis);
    axis_obj->jam_speed_diff = speed_diff;
    axis_obj->jam_threshold = count_threshold;
}

void GimbalAxis_SetOutputLimit(GimbalAxis_e axis, int32_t output_limit)
{
    GimbalAxisMotor_t *axis_obj = GimbalAxis_GetAxis(axis);
    if (output_limit < 0)
    {
        output_limit = -output_limit;
    }
    axis_obj->output_limit = output_limit;
}

void GimbalAxis_SetMode(GimbalAxis_e axis, GimbalCtrlMode_e mode)
{
    GimbalAxisMotor_t *axis_obj = GimbalAxis_GetAxis(axis);
    axis_obj->mode = mode;
}

void GimbalAxis_SetSpeed(GimbalAxis_e axis, int16_t speed)
{
    GimbalAxisMotor_t *axis_obj = GimbalAxis_GetAxis(axis);
    axis_obj->mode = GIMBAL_CTRL_SPEED;
    axis_obj->target_speed = speed;
}

void GimbalAxis_SetAngle(GimbalAxis_e axis, float angle)
{
    GimbalAxisMotor_t *axis_obj = GimbalAxis_GetAxis(axis);
    axis_obj->mode = GIMBAL_CTRL_ANGLE;
    axis_obj->target_angle = angle;
}

void GimbalAxis_Stop(GimbalAxis_e axis)
{
    GimbalAxisMotor_t *axis_obj = GimbalAxis_GetAxis(axis);
    axis_obj->target_speed = 0;
    axis_obj->target_angle = axis_obj->motor.Data.TotalAngle;
    axis_obj->mode = GIMBAL_CTRL_ANGLE;
    axis_obj->state = GIMBAL_STATE_HOLDING;
}

void GimbalAxis_MovePitch(float angle)
{
    GimbalAxis_SetAngle(GIMBAL_AXIS_PITCH, angle);
}

void GimbalAxis_MoveYaw(float angle)
{
    GimbalAxis_SetAngle(GIMBAL_AXIS_YAW, angle);
}

void GimbalAxis_MoveStretch(float angle)
{
    GimbalAxis_SetAngle(GIMBAL_AXIS_STRETCH, angle);
}

void GimbalAxis_StopPitch(void)
{
    GimbalAxis_Stop(GIMBAL_AXIS_PITCH);
}

void GimbalAxis_StopYaw(void)
{
    GimbalAxis_Stop(GIMBAL_AXIS_YAW);
}

void GimbalAxis_StopStretch(void)
{
    GimbalAxis_Stop(GIMBAL_AXIS_STRETCH);
}

float GimbalAxis_GetAngle(GimbalAxis_e axis)
{
    GimbalAxisMotor_t *axis_obj = GimbalAxis_GetAxis(axis);
    return axis_obj->motor.Data.TotalAngle;
}

int16_t GimbalAxis_GetSpeed(GimbalAxis_e axis)
{
    GimbalAxisMotor_t *axis_obj = GimbalAxis_GetAxis(axis);
    return axis_obj->motor.Data.SpeedRPM;
}

GimbalAxisState_e GimbalAxis_GetState(GimbalAxis_e axis)
{
    GimbalAxisMotor_t *axis_obj = GimbalAxis_GetAxis(axis);
    return axis_obj->state;
}
