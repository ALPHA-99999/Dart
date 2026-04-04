#include <driver_timer.h>
#include "control_logic.h"
#include "hardware_config.h"
#include "slider.h"
#include "gimbal_axis.h"
void TIM14_Task(void)
{

	tim14.ClockTime++;
	RobotOnlineState(&check_robot_state, &referee2022, &rc_Ctrl_et);

//	if (rc_Ctrl_et.isOnline==0) g_slider.motor.Data.Output=0;
//	else {
//		Slider_Control();
//		if (rc_Ctrl_et.rc.s1==2 && g_slider.state==SLIDER_STATE_IDLE) Slider_MoveDown(-500);
//		else if (rc_Ctrl_et.rc.s1==1 && g_slider.state==SLIDER_STATE_IDLE) Slider_MoveUp(500);
//		else if (rc_Ctrl_et.rc.s1==3)g_slider.state=SLIDER_STATE_IDLE;
//	}
//		MotorFillData(&g_slider.motor,g_slider.motor.Data.Output);
	
	
	
//		if (rc_Ctrl_et.isOnline==0) {g_gimbal_axis.pitch.motor.Data.Output=0;g_gimbal_axis.yaw.motor.Data.Output=0;}
//	else {
//		GimbalAxis_Control();
//		if (rc_Ctrl_et.rc.s1==2 ) GimbalAxis_MovePitch(-5000);
//		else if (rc_Ctrl_et.rc.s1==1 ) GimbalAxis_MovePitch(5000);
//		else if (rc_Ctrl_et.rc.s1==3) g_gimbal_axis.pitch.state=GIMBAL_STATE_IDLE;
//	}
//	MotorFillData(&g_gimbal_axis.pitch.motor,g_gimbal_axis.pitch.motor.Data.Output);
//	MotorFillData(&g_gimbal_axis.yaw.motor,g_gimbal_axis.yaw.motor.Data.Output);

//if (rc_Ctrl_et.rc.s1==2)  Slider_TriggerOpen();
//else Slider_TriggerClose();



//	MotorCanOutput(can2, 0x1FF);
	//MotorCanOutput(can1, 0x1FF);
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

