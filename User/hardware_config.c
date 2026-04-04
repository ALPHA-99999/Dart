#include "hardware_config.h"
#include "driver_usart.h"
#include <dr16.h>
#include <control_logic.h>
#include <driver_can.h>
#include <driver_timer.h>
#include <attitude.h>
#include "protract.h"
#include "sideway.h"
#include <brain.h>
#include <et08.h>
#include "mecanum_chassis.h"
#include "Gyro.h"
#include "mpu6050.h"
#include "referee.h"
#include "driver_flash.h"
#include "slider.h"
#include "gimbal_axis.h"
// #include "referee.h"

void HardwareConfig()
{
	check_robot_state.usart_state.Check_receiver = 30;
	
ET08Init(&rc_Ctrl_et);
   Slider_Init();  
    GimbalAxis_Init();
	Slider_TriggerConfig(13,20);
	
		UARTx_Init(&huart1,ET08_callback);
	UART_ENABLE_IT(&uart1,&uart1_buffer);
	UART_Receive_DMA(&uart1, &uart1_buffer);
	
	CANx_Init(&hfdcan1, CAN1_rxCallBack);
	CAN_Open(&can1);

	CANx_Init(&hfdcan2, CAN2_rxCallBack);
	CAN_Open(&can2);

	TIMx_Init(&htim14, TIM14_Task);
	TIM_Open(&tim14);
	
	TIMx_Init(&htim13, TIM13_Task);
	TIM_Open(&tim13);
}

