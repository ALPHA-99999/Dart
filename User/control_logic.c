#include <driver_timer.h>
#include "control_logic.h"
#include "hardware_config.h"
#include "slider.h"
#include "gimbal_axis.h"

typedef enum
{
	MASTER_CTRL_IDLE = 0,
	MASTER_CTRL_RESET,
	MASTER_CTRL_SLIDER_DOWN,
	MASTER_CTRL_TRIGGER_CLOSE,
	MASTER_CTRL_SLIDER_UP,
	MASTER_CTRL_TRIGGER_OPEN,
	MASTER_CTRL_FIRE,
	MASTER_CTRL_DONE
} MasterCtrlState_e;

static MasterCtrlState_e s_master_state = MASTER_CTRL_IDLE;
static uint32_t s_master_state_tick = 0;
static uint8_t s_last_s1 = 3;

#define MASTER_RESET_TICKS          300U
#define MASTER_TRIGGER_CLOSE_TICKS  350U
#define MASTER_TRIGGER_OPEN_TICKS   250U
#define MASTER_FIRE_CLOSE_TICKS     120U

static void MasterControl_EnterState(MasterCtrlState_e state)
{
	s_master_state = state;
	s_master_state_tick = tim14.ClockTime;
}

void MasterControl_Start(void)
{
	MasterControl_EnterState(MASTER_CTRL_RESET);
}

void MasterControl_Stop(void)
{
	MasterControl_EnterState(MASTER_CTRL_IDLE);
	Slider_Reset();
	Slider_TriggerOpen();
	GimbalAxis_Reset();
}

void MasterControl_Task(void)
{
	switch (s_master_state)
	{
	case MASTER_CTRL_IDLE:
		break;

	case MASTER_CTRL_RESET:
		/* yaw/pitch + 皮筋机构复位 */
		GimbalAxis_MoveYaw(0.0f);
		GimbalAxis_MovePitch(0.0f);
		Slider_Reset();
		Slider_TriggerOpen();
		if ((tim14.ClockTime - s_master_state_tick) >= MASTER_RESET_TICKS)
		{
			MasterControl_EnterState(MASTER_CTRL_SLIDER_DOWN);
		}
		break;

	case MASTER_CTRL_SLIDER_DOWN:
		if (g_slider.state == SLIDER_STATE_IDLE)
		{
			Slider_MoveDown(-500);
		}
		if (g_slider.state == SLIDER_STATE_STOP_DOWN)
		{
			MasterControl_EnterState(MASTER_CTRL_TRIGGER_CLOSE);
		}
		break;

	case MASTER_CTRL_TRIGGER_CLOSE:
		Slider_TriggerClose();
		if ((tim14.ClockTime - s_master_state_tick) >= MASTER_TRIGGER_CLOSE_TICKS)
		{
			Slider_Reset();
			MasterControl_EnterState(MASTER_CTRL_SLIDER_UP);
		}
		break;

	case MASTER_CTRL_SLIDER_UP:
		if (g_slider.state == SLIDER_STATE_IDLE)
		{
			Slider_MoveUp(500);
		}
		if (g_slider.state == SLIDER_STATE_STOP_UP)
		{
			MasterControl_EnterState(MASTER_CTRL_TRIGGER_OPEN);
		}
		break;

	case MASTER_CTRL_TRIGGER_OPEN:
		Slider_TriggerOpen();
		if ((tim14.ClockTime - s_master_state_tick) >= MASTER_TRIGGER_OPEN_TICKS)
		{
			MasterControl_EnterState(MASTER_CTRL_FIRE);
		}
		break;

	case MASTER_CTRL_FIRE:
		/* 开合一次完成发射 */
		if ((tim14.ClockTime - s_master_state_tick) < MASTER_FIRE_CLOSE_TICKS)
		{
			Slider_TriggerClose();
		}
		else
		{
			Slider_TriggerOpen();
			MasterControl_EnterState(MASTER_CTRL_DONE);
		}
		break;

	case MASTER_CTRL_DONE:
		break;
	}
}

void TIM14_Task(void)
{
	tim14.ClockTime++;
	RobotOnlineState(&check_robot_state, &referee2022, &rc_Ctrl_et);

	if (rc_Ctrl_et.isOnline == 0)
	{
		MasterControl_Stop();
		g_slider.motor.Data.Output = 0;
		g_gimbal_axis.pitch.motor.Data.Output = 0;
		g_gimbal_axis.yaw.motor.Data.Output = 0;
	}
	else
	{
		if (rc_Ctrl_et.rc.s1 == 1 && s_last_s1 != 1)
		{
			MasterControl_Start();
		}
		else if (rc_Ctrl_et.rc.s1 == 3 && s_last_s1 != 3)
		{
			MasterControl_Stop();
		}
		s_last_s1 = rc_Ctrl_et.rc.s1;

		MasterControl_Task();
		Slider_Control();
		GimbalAxis_Control();
	}

	MotorFillData(&g_slider.motor, g_slider.motor.Data.Output);
	MotorFillData(&g_gimbal_axis.pitch.motor, g_gimbal_axis.pitch.motor.Data.Output);
	MotorFillData(&g_gimbal_axis.yaw.motor, g_gimbal_axis.yaw.motor.Data.Output);

	MotorCanOutput(can2, 0x200);
}

/**
 * @brief  CAN1接收中断回调
 */
uint8_t CAN1_rxCallBack(CAN_RxBuffer *rxBuffer)
{
	MotorRxCallback(can1, (*rxBuffer));
	return 0;
}

/**
 * @brief  CAN2接收中断回调
 */
uint8_t CAN2_rxCallBack(CAN_RxBuffer *rxBuffer)
{
	MotorRxCallback(can2, (*rxBuffer));
	return 0;
}
void TIM13_Task(void)
{
	Slider_TriggerControl();
}
