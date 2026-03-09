#ifndef _ATTITUDE_H
#define _ATTITUDE_H
#include "stm32h7xx_hal.h"
#include "pid.h"
#include "motor.h"
#include<dr16.h>
#include "driver_usart.h"
struct Attitude_Motor_Info
{
  float Target_Angle;
	float Can_Angle;
	float Last_Can_Angle;
	float Can_Angle_speed;
  float	Raw_angle;
	float	Raw_angle_last;
  float	Pitch_angle;
	float	Pitch_angle_last;
	int16_t Can_Speed_Feedback;
	float circle_num;
	float Sensitivity;
};
typedef struct
{
  struct Attitude_Motor_Info Pitch;
  struct Attitude_Motor_Info Yaw;
	struct Attitude_Motor_Info Roll;
	struct 
	{
		float speed;
		float pos_yaw;
		float pos_pitch;
		float yaw_target;
		float pitch_target;
	}Motors_DM;
	struct 
	{
		Motor motor[1];						//< 姿态电机结构体
		BasePID_Object turnPID[3];
		float FeedbackAngle[2];
		float HolderCanAngle[2];		
		float angle_last;
		float angle;
		float target_angle;
		float target_out;
	}Motors3508;
	struct 
	{
		Motor motor[2];						//< 姿态电机结构体
		BasePID_Object turnPID[6];		//< 转向角度控制结构体
		BasePID_Object pitchPID[2];
		float FeedbackAngle[2];
		float HolderCanAngle[2];		
		float angle_last;
		float angle_last2;
		float angle;
		float angle2;
		float target_angle;
		float target_angle2;
		float target_out;
		float speed_target;
		float deg_last;
		float deg_last2;
	}Motors6020;
	
	struct 
	{
		Motor motor[1];						//< 底盘电机结构体
		BasePID_Object RunPID[4];			//< 速度控制PID参数
		float roll_angel;
		float angle_last;
		float angle;
		float turns_count;
		float target_angle;
		float target_out;
		uint8_t manual_rc_flag;
	}Motors2006;
	
	struct 
	{
	  float yall_angle;
		float yall_angle_last;
		float yall_angle_sum;
		float pitch_angle;
		float roll_angle;
		float ver;
		float sum;
		float Q0;
		float Q1;
		float Q2;
		float Q3;
		float yall_angle_Q;
		float pitch_angel_Q;
		float roll_angle_Q;
		float X_SPEED;
		float Y_SPEED;
		float Z_SPEED;
		float pitch_angle_speed;
		float yall_angle_speed;
		float yall_speed;
	}HWTdata;
	struct
	{
	 int16_t circle_num_id1;
	 int16_t last_locate_id1;
	 int16_t circle_num_id2;
	 int16_t last_locate_id2;
	 int16_t circle_num_id3;
	 int16_t last_locate_id3;
	}servolast;
  
}Attitude_t;

void Attitude_6020Init(Attitude_t *attitude, BasePID_Object yaw_angle_pid ,BasePID_Object yaw_speed_pid, BasePID_Object yaw2_angle_pid ,BasePID_Object yaw2_speed_pid, CanNumber canx);
void Holder_Control(Attitude_t *attitude,RC_Ctrl *rc_ctrl);
void Attitude_3508Reset(Attitude_t *attitude);
void Attitude_Control(Attitude_t *attitude,RC_Ctrl *rc_ctrl);
void Attitude_6020Control(Attitude_t *attitude,RC_Ctrl *rc_ctrl);
void Attitude_Init(Attitude_t *attitude, BasePID_Object roll_speed_pid, BasePID_Object roll_angle_pid, BasePID_Object pitch_speed3508, BasePID_Object pitch_angle3508, CanNumber canx);
void AttitudeReset(Attitude_t *attitude);
void AttitudeGetRemoteData(Attitude_t *attitude,RC_Ctrl *rc_ctrl);
void Bypass_motor_Control(RC_Ctrl *rc_ctrl);
void angle_deal(uint8_t *angle_receive,Attitude_t *attitude);
uint8_t Gyro_callback(uint8_t * recBuffer, uint16_t len);
uint8_t HWT606_callback(uint8_t * recBuffer, uint16_t len);
void AT24C02_READ(Attitude_t *attitude);
void AT24C02_WRITE();
extern UART_RxBuffer uart7_buffer;
//extern UART_RxBuffer uart2_buffer;
//extern UART_RxBuffer uart3_buffer;
extern UART_RxBuffer uart4_buffer;
extern UART_RxBuffer uart5_buffer;
extern UART_RxBuffer uart6_buffer;
extern UART_RxBuffer uart8_buffer;

extern float roll_target_speed;	
extern float pitchangel;

#endif