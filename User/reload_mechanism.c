/**
 ******************************************************************************
 * @file           : reload_mechanism.c
 * @brief          : 换弹机构控制模块 - 实现文件
 ******************************************************************************
 */

#include "reload_mechanism.h"

#include "usart.h"

#ifndef LIMIT
#define LIMIT(x, max, min) (((x) > (max)) ? (max) : (((x) < (min)) ? (min) : (x)))
#endif

#ifndef GET_LOW_BYTE
#define GET_LOW_BYTE(x) ((uint8_t)(x))
#endif

#ifndef GET_HIGH_BYTE
#define GET_HIGH_BYTE(x) ((uint8_t)((x) >> 8))
#endif

ReloadMechanism_t g_reload;

static uint8_t ReloadMechanism_Checksum(const uint8_t *buf)
{
    uint8_t i;
    uint16_t sum = 0;

    for (i = 2; i < (uint8_t)(buf[3] + 2); i++)
    {
        sum += buf[i];
    }

    return (uint8_t)(~sum);
}

static void ReloadMechanism_SendServoCmd(void)
{
    uint8_t frame[10];

    if (!g_reload.servo_pending)
    {
        return;
    }

    frame[0] = 0x55;
    frame[1] = 0x55;
    frame[2] = g_reload.servo_id;
    frame[3] = 7;
    frame[4] = 1;
    frame[5] = GET_LOW_BYTE(g_reload.servo_target_pos);
    frame[6] = GET_HIGH_BYTE(g_reload.servo_target_pos);
    frame[7] = GET_LOW_BYTE(g_reload.servo_move_time);
    frame[8] = GET_HIGH_BYTE(g_reload.servo_move_time);
    frame[9] = ReloadMechanism_Checksum(frame);

    HAL_UART_Transmit_DMA(&huart5, frame, 10);
    g_reload.servo_pending = 0;
}

static void ReloadMechanism_ApplyDiscreteTargets(void)
{
    switch (g_reload.state_2006)
    {
    case RELOAD_2006_STOP:
        g_reload.target_angle_2006 = g_reload.motor_2006.Data.TotalAngle;
        break;
    case RELOAD_2006_DOWN:
        g_reload.target_angle_2006 = g_reload.base_angle_2006 + g_reload.down_offset_2006;
        break;
    case RELOAD_2006_UP:
    default:
        g_reload.target_angle_2006 = g_reload.base_angle_2006 + g_reload.up_offset_2006;
        break;
    }

    switch (g_reload.pose_6020)
    {
    case RELOAD_6020_POS_P90:
        g_reload.target_angle_6020 = g_reload.base_angle_6020 + 90.0f;
        break;
    case RELOAD_6020_POS_N90:
        g_reload.target_angle_6020 = g_reload.base_angle_6020 - 90.0f;
        break;
    case RELOAD_6020_POS_0:
    default:
        g_reload.target_angle_6020 = g_reload.base_angle_6020;
        break;
    }

    if (g_reload.pose_servo == RELOAD_SERVO_OPEN)
    {
        g_reload.servo_target_pos = g_reload.servo_pos_open;
    }
    else
    {
        g_reload.servo_target_pos = g_reload.servo_pos_close;
    }
}

static void ReloadMechanism_Control2006(void)
{
    int32_t speed_out = BasePID_AngleControl(&g_reload.pid_2006_angle,
                                             g_reload.target_angle_2006,
                                             g_reload.motor_2006.Data.TotalAngle);

    g_reload.motor_2006.Data.Target = speed_out;
    g_reload.motor_2006.Data.Output = BasePID_SpeedControl(&g_reload.pid_2006_speed,
                                                           g_reload.motor_2006.Data.Target,
                                                           g_reload.motor_2006.Data.SpeedRPM);

    g_reload.motor_2006.Data.Output = LIMIT(g_reload.motor_2006.Data.Output,
                                            g_reload.output_limit_2006,
                                            -g_reload.output_limit_2006);
}

static void ReloadMechanism_Control6020(void)
{
    int32_t speed_out = BasePID_AngleControl(&g_reload.pid_6020_angle,
                                             g_reload.target_angle_6020,
                                             g_reload.motor_6020.Data.TotalAngle);

    g_reload.motor_6020.Data.Target = speed_out;
    g_reload.motor_6020.Data.Output = BasePID_SpeedControl(&g_reload.pid_6020_speed,
                                                           g_reload.motor_6020.Data.Target,
                                                           g_reload.motor_6020.Data.SpeedRPM);

    g_reload.motor_6020.Data.Output = LIMIT(g_reload.motor_6020.Data.Output,
                                            g_reload.output_limit_6020,
                                            -g_reload.output_limit_6020);
}

static void ReloadMechanism_RunCoopSequence(void)
{
    uint32_t elapsed_ms;

    if (!g_reload.sync_enable)
    {
        return;
    }

    elapsed_ms = HAL_GetTick() - g_reload.sync_tick_ms;

    switch (g_reload.sync_stage)
    {
    case 0:
        /* 预留: 阶段0可先执行舵机闭合或6020归零动作 */
        if (elapsed_ms > 80U)
        {
            g_reload.sync_stage = 1;
            g_reload.sync_tick_ms = HAL_GetTick();
        }
        break;

    case 1:
        /* 预留: 阶段1可执行2006下压 + 6020切换 */
        if (elapsed_ms > 120U)
        {
            g_reload.sync_stage = 2;
            g_reload.sync_tick_ms = HAL_GetTick();
        }
        break;

    case 2:
        /* 预留: 阶段2可执行2006回位 + 舵机张开 */
        if (elapsed_ms > 120U)
        {
            g_reload.sync_stage = 0;
            g_reload.sync_tick_ms = HAL_GetTick();
            g_reload.sync_enable = 0;
        }
        break;

    default:
        g_reload.sync_stage = 0;
        g_reload.sync_tick_ms = HAL_GetTick();
        g_reload.sync_enable = 0;
        break;
    }
}

void ReloadMechanism_Init(void)
{
    MotorInit(&g_reload.motor_2006, 0, Motor2006, CAN2, 0x203);
    MotorInit(&g_reload.motor_6020, 0, Motor6020, CAN1, 0x205);

    BasePID_Init(&g_reload.pid_2006_speed, 10.0f, 0.0f, 0.0f, 0.0f);
    BasePID_Init(&g_reload.pid_2006_angle, 1.0f, 0.0f, 0.0f, 0.0f);
    BasePID_Init(&g_reload.pid_6020_speed, 8.0f, 0.0f, 0.0f, 0.0f);
    BasePID_Init(&g_reload.pid_6020_angle, 1.0f, 0.0f, 0.0f, 0.0f);

    g_reload.base_angle_2006 = 0.0f;
    g_reload.base_angle_6020 = 0.0f;
    g_reload.down_offset_2006 = -45.0f;
    g_reload.up_offset_2006 = 0.0f;

    g_reload.target_angle_2006 = 0.0f;
    g_reload.target_angle_6020 = 0.0f;

    g_reload.servo_id = 1;
    g_reload.servo_pos_close = 650;
    g_reload.servo_pos_open = 900;
    g_reload.servo_target_pos = g_reload.servo_pos_close;
    g_reload.servo_move_time = 20;
    g_reload.servo_pending = 1;

    g_reload.state_2006 = RELOAD_2006_STOP;
    g_reload.pose_6020 = RELOAD_6020_POS_0;
    g_reload.pose_servo = RELOAD_SERVO_CLOSE;

    g_reload.sync_enable = 0;
    g_reload.sync_stage = 0;
    g_reload.sync_tick_ms = HAL_GetTick();

    g_reload.enabled = 1;
    g_reload.output_limit_2006 = CURRENT_LIMIT_FOR_2006;
    g_reload.output_limit_6020 = CURRENT_LIMIT_FOR_6020;
}

void ReloadMechanism_Reset(void)
{
    g_reload.base_angle_2006 = g_reload.motor_2006.Data.TotalAngle;
    g_reload.base_angle_6020 = g_reload.motor_6020.Data.TotalAngle;

    g_reload.state_2006 = RELOAD_2006_STOP;
    g_reload.pose_6020 = RELOAD_6020_POS_0;
    g_reload.pose_servo = RELOAD_SERVO_CLOSE;

    g_reload.servo_pending = 1;
    g_reload.sync_enable = 0;
    g_reload.sync_stage = 0;
    g_reload.sync_tick_ms = HAL_GetTick();

    g_reload.motor_2006.Data.Output = 0;
    g_reload.motor_6020.Data.Output = 0;
}

void ReloadMechanism_Control(void)
{
    if (!g_reload.enabled)
    {
        g_reload.motor_2006.Data.Output = 0;
        g_reload.motor_6020.Data.Output = 0;
        return;
    }

    /* 预留: 这里是三执行单元协同控制入口，可按时间窗切换档位 */
    ReloadMechanism_RunCoopSequence();

    ReloadMechanism_ApplyDiscreteTargets();
    ReloadMechanism_Control2006();
    ReloadMechanism_Control6020();
    ReloadMechanism_SendServoCmd();
}

void ReloadMechanism_Enable(uint8_t enable)
{
    g_reload.enabled = enable;
    if (!enable)
    {
        ReloadMechanism_Reset();
    }
}

void ReloadMechanism_Config2006Motor(CanNumber canx, uint16_t motor_id, uint16_t ecd_offset)
{
    MotorInit(&g_reload.motor_2006, ecd_offset, Motor2006, canx, motor_id);
}

void ReloadMechanism_Config6020Motor(CanNumber canx, uint16_t motor_id, uint16_t ecd_offset)
{
    MotorInit(&g_reload.motor_6020, ecd_offset, Motor6020, canx, motor_id);
}

void ReloadMechanism_Config2006PID(float speed_kp, float speed_ki, float speed_kd,
                                   float angle_kp, float angle_ki, float angle_kd)
{
    BasePID_Init(&g_reload.pid_2006_speed, speed_kp, speed_ki, speed_kd, 0.0f);
    BasePID_Init(&g_reload.pid_2006_angle, angle_kp, angle_ki, angle_kd, 0.0f);
}

void ReloadMechanism_Config6020PID(float speed_kp, float speed_ki, float speed_kd,
                                   float angle_kp, float angle_ki, float angle_kd)
{
    BasePID_Init(&g_reload.pid_6020_speed, speed_kp, speed_ki, speed_kd, 0.0f);
    BasePID_Init(&g_reload.pid_6020_angle, angle_kp, angle_ki, angle_kd, 0.0f);
}

void ReloadMechanism_Set2006State(Reload2006State_e state)
{
    g_reload.state_2006 = state;
}

void ReloadMechanism_Set6020Pose(Reload6020Pose_e pose)
{
    g_reload.pose_6020 = pose;
}

void ReloadMechanism_SetServoPose(ReloadServoPose_e pose)
{
    if (g_reload.pose_servo != pose)
    {
        g_reload.pose_servo = pose;
        g_reload.servo_pending = 1;
    }
}

void ReloadMechanism_Set2006Offset(float down_offset, float up_offset)
{
    g_reload.down_offset_2006 = down_offset;
    g_reload.up_offset_2006 = up_offset;
}

void ReloadMechanism_SetServoPosition(uint16_t close_pos, uint16_t open_pos)
{
    g_reload.servo_pos_close = close_pos;
    g_reload.servo_pos_open = open_pos;
    g_reload.servo_pending = 1;
}

void ReloadMechanism_StartCoopSequence(void)
{
    g_reload.sync_enable = 1;
    g_reload.sync_stage = 0;
    g_reload.sync_tick_ms = HAL_GetTick();
}

void ReloadMechanism_StopCoopSequence(void)
{
    g_reload.sync_enable = 0;
    g_reload.sync_stage = 0;
}

float ReloadMechanism_Get2006Angle(void)
{
    return g_reload.motor_2006.Data.TotalAngle;
}

float ReloadMechanism_Get6020Angle(void)
{
    return g_reload.motor_6020.Data.TotalAngle;
}

void ReloadMechanism_2006Stop(void)
{
    ReloadMechanism_Set2006State(RELOAD_2006_STOP);
}

void ReloadMechanism_2006Down(void)
{
    ReloadMechanism_Set2006State(RELOAD_2006_DOWN);
}

void ReloadMechanism_2006Up(void)
{
    ReloadMechanism_Set2006State(RELOAD_2006_UP);
}

void ReloadMechanism_6020ToZero(void)
{
    ReloadMechanism_Set6020Pose(RELOAD_6020_POS_0);
}

void ReloadMechanism_6020ToPos90(void)
{
    ReloadMechanism_Set6020Pose(RELOAD_6020_POS_P90);
}

void ReloadMechanism_6020ToNeg90(void)
{
    ReloadMechanism_Set6020Pose(RELOAD_6020_POS_N90);
}

void ReloadMechanism_ServoClose(void)
{
    ReloadMechanism_SetServoPose(RELOAD_SERVO_CLOSE);
}

void ReloadMechanism_ServoOpen(void)
{
    ReloadMechanism_SetServoPose(RELOAD_SERVO_OPEN);
}

void ReloadMechanism_TriggerReload(void)
{
    ReloadMechanism_StartCoopSequence();
}
