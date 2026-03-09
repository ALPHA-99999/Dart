#include <driver_timer.h>
#include "control_logic.h"
#include "hardware_config.h"



#define servo_red_min 13
#define servo_red_max 20

#define servo_yellow_max 495

uint8_t flag_completely;
uint8_t change_mode = 1;
uint8_t cnt_2006_down, cnt_2006_up, cnt_6020_move, cnt_6020_back;
uint8_t servo_check_number(uint8_t buf[]);					// 校验位
void servo_move(uint16_t id, uint16_t time, int16_t angle); // 位置控制模式
// 需要修改的代码：plunging reset
int b1 = 0;
int cnt__l = 0, flag_none;
#define abs(x) ((x) > 0 ? (x) : (-(x)))

float InversePitchCalculation(float);


Dart_Info infor[8] = {
	{-215, 29.595, 8}, {-146, 29.202, 1}, {-232, 29.722, 4}, {-198, 29.585, 2}, {-80, 30.23159, 9},
	{-50, 30.91769, 3}};

float Yaw_offset = 0;
uint8_t Last_dart_launch_opening_status;
int Speed_3508;
float kll;
uint8_t flag_yaw_stop = 0, mode, shot_complete;
uint16_t cnt_complete;
uint8_t flag_pitch_stop = 0;
uint8_t flag_shoot = servo_red_min;
// uint8_t flag_shoot=10;
uint8_t flag_stop = 0, flag_Camera = 1;
uint8_t flag_reset = 0;
int cnt_2006_yaw, cnt_2006_pitch;
int cnt, cnt_2, flag_2006, cnt_2006;
float Target_Angle_3508 = 0;
float Target_Angle_2006 = 0;
float Target_Angle_2006_yaw = 0;
float Target_Angle_2006_pitch = 0;
int cnt_servo_;
/**
 * @brief  主任务函数
 */
uint8_t flag11, flag_wait;
uint16_t cnt1111;
float a22, a23;

int cntmm;
float Yaw, Pitch;
int servo_yellow = 120;
int Target_Angle_2006_load;
uint8_t Reload_mode;//换弹模式 1-换弹1 2-换弹2 3-换弹 一共三发
float Yaw_add;
uint16_t Camera_cnt, Camera_Fps;
RubberState_t Rubber_state = Rubber_IDLE;
ReloadState_t Reload_state = RELOAD_IDLE;
uint16_t cnt6020_down, cnt6020_up, cnt2006_down, cnt2006_up, cnt_servo, cnt_error;
float Target_6020 = -270;
int cnt_up_stop;
int Timer = 0;
int flll;
float Rubber_Reset_Angel;

void TIM14_Task(void)
{
	int i;
	static int cnt_yaw_stop = 0, cnt_pitch_stop = 0, cnt_down_stop, cnt_reset = 0;
	tim14.ClockTime++;

	Target_Angle_2006 = motor2006.motor[0].Data.TotalAngle;
	Yaw_offset = infor[0].Yaw_offset;

	if (tim14.ClockTime % 1000 == 0)
	{
		Camera_Fps = Camera_cnt;
		Camera_cnt = 0;
	}

	RobotOnlineState(&check_robot_state, &referee2022, &rc_Ctrl_et);
	// 上堵转
	up_stop();

	if (rc_Ctrl_et.isOnline == 1 && flag_2006 == 0 && rc_Ctrl_et.rc.s2 != 2)
		Target_Angle_2006 += (rc_Ctrl_et.rc.ch3 - 1024) * 0.05;

	if (rc_Ctrl_et.isOnline == 1 && rc_Ctrl_et.rc.s2 == 2)
	{
		Target_Angle_2006_yaw -= (rc_Ctrl_et.rc.ch2 - 1024) * 0.05;
		Target_Angle_2006_pitch -= (rc_Ctrl_et.rc.ch3 - 1024) * 0.05;
	}

	if (rc_Ctrl_et.rc.s1 == 3)
		Speed_3508 = 0; // 左拨杆控制3508上下
	else if (rc_Ctrl_et.rc.s1 == 2)
		Speed_3508 = -5000;
	else if (rc_Ctrl_et.rc.s1 == 1)
		Speed_3508 = 8000;

	// 消堵转
	if (rc_Ctrl_et.rc.s2 == 1 && rc_Ctrl_et.rc.s2_last == 3 && flag_stop == 1 && referee2022.game_status.game_progress != 4)
		flag_stop = 0;

	if (rc_Ctrl_et.rc.s2 == 1 && rc_Ctrl_et.rc.s2_last == 3 && referee2022.game_status.game_progress == 4)
	{
		mode++;
		if (mode == 1)
		{
			Target_Angle_2006_pitch = InversePitchCalculation(infor[0].Pitch_offset) + a23;
			if (Rubber_state == Rubber_IDLE)
			{
				Rubber_state = Rubber_STEP1;
				flag_stop = 0;
			}
		}
		else if (mode == 2)
		{
			if (Rubber_state == Rubber_IDLE || Rubber_state == Rubber_COMPLETE)
			{
				Rubber_state = Rubber_STEP1;
				flag_stop = 0;
			}
			flag_completely = 1;
			servo_yellow = servo_yellow_max;
		}
	}

	if (rc_Ctrl_et.rc.s2 == 2 && rc_Ctrl_et.rc.s2_last == 3 && referee2022.game_status.game_progress == 4)
	{
		flag_wait = 1;
	}

	rc_Ctrl_et.rc.s2_last = rc_Ctrl_et.rc.s2;
	rc_Ctrl_et.rc.s1_last = rc_Ctrl_et.rc.s1;

	Change_YawPitch_use_remotecontrol();

	// 开关舵机
	servo_control();

	down_stop();

	Yaw_Pitch_Reset();

	Reload_change();

	Rubber_change();

	if (flag_stop == 0)
		motor3508.motor[0].Data.Target = Speed_3508;
	else
		motor3508.motor[0].Data.Target = 0;

	if (flag_stop == 1) // 堵转后只用位置环控制
		motor3508.motor[0].Data.Output =
			BasePID_PitchSpeedControl((BasePID_Object *)(&Motors3508_SpeedPID),
									  BasePID_PitchAngleControl((BasePID_Object *)(&Motors3508_AngelPID), Target_Angle_3508, motor3508.motor[0].Data.TotalAngle), motor3508.motor[0].Data.SpeedRPM);
	else
		motor3508.motor[0].Data.Output = BasePID_SpeedControl(&run_pid, motor3508.motor[0].Data.Target, motor3508.motor[0].Data.SpeedRPM);

	motor2006.motor[3].Data.Output =
		BasePID_PitchSpeedControl((BasePID_Object *)(&Motors2006_load_SpeedPID),
								  BasePID_YawAngleControl((BasePID_Object *)(&Motors2006_load_AngelPID), Target_Angle_2006_load, motor2006.motor[3].Data.TotalAngle), motor2006.motor[3].Data.SpeedRPM);

	motor6020.motor[0].Data.Output =
		BasePID_PitchSpeedControl((BasePID_Object *)(&Motors6020_pitch_SpeedPID),
								  BasePID_YawAngleControl((BasePID_Object *)(&Motors6020_pitch_AngelPID), Target_6020, motor6020.motor[0].Data.TotalAngle), motor6020.motor[0].Data.SpeedRPM);

	motor2006.motor[0].Data.Output =
		BasePID_YawSpeedControl((BasePID_Object *)(&Motors2006_SpeedPID),
								BasePID_YawAngleControl((BasePID_Object *)(&Motors2006_AngelPID), Target_Angle_2006, motor2006.motor[0].Data.TotalAngle), motor2006.motor[0].Data.SpeedRPM);

	Send_toled();

	if (rc_Ctrl_et.rc.s2 == 2) // 复位标志位 遥控器产生
		cnt_reset++;
	else
		cnt_reset = 0;
	if (cnt_reset > 1000)
	{
		cnt_reset = 0;
		flag_reset = 1;
	}

	if (rc_Ctrl_et.isOnline != 1)
	{
		ET08Init(&rc_Ctrl_et);
		flag_stop = 0;
	}

	if (Yaw_add != 0 && flag_Camera == 0)
	{
		if (Yaw_add > (5 + Yaw_offset) && tim14.ClockTime % 100 == 0 && flag_reset == 1 && flag_yaw_stop == 1 && rc_Ctrl_et.rc.s2 == 1)
			Target_Angle_2006_yaw += 2000;
		else if (Yaw_add < (-5 + Yaw_offset) && tim14.ClockTime % 100 == 0 && flag_reset == 1 && flag_yaw_stop == 1 && rc_Ctrl_et.rc.s2 == 1)
			Target_Angle_2006_yaw -= 2000;
		else if (Yaw_add > (-5 + Yaw_offset) && Yaw_add < (-2 + Yaw_offset) && tim14.ClockTime % 100 == 0 && flag_reset == 1 && flag_yaw_stop == 1 && rc_Ctrl_et.rc.s2 == 1)
			Target_Angle_2006_yaw -= 500;
		else if (Yaw_add < (5 + Yaw_offset) && Yaw_add > (2 + Yaw_offset) && tim14.ClockTime % 100 == 0 && flag_reset == 1 && flag_yaw_stop == 1 && rc_Ctrl_et.rc.s2 == 1)
			Target_Angle_2006_yaw += 500;
	}
	else if (Yaw_add != 0 && flag_Camera == 1)
	{
		if (tim14.ClockTime % 100 == 0)
		{
			if (Yaw_add > (5 + Yaw_offset))
				Target_Angle_2006_yaw += 2000;
			else if (Yaw_add < (-5 + Yaw_offset))
				Target_Angle_2006_yaw -= 2000;
			else if (Yaw_add > (-5 + Yaw_offset) && Yaw_add < (-2 + Yaw_offset))
				Target_Angle_2006_yaw -= 500;
			else if (Yaw_add < (5 + Yaw_offset) && Yaw_add > (2 + Yaw_offset))
				Target_Angle_2006_yaw += 500;
		}
	}
	Lcd_control();

	if (tim14.ClockTime % 30 == 0)
		servo_move(0xFE, 100, servo_yellow);

	float Yaw_l = (209 - (fabs(motor2006.motor[1].Data.TotalAngle - a22) / 533234 * 164));

	if (Yaw_l < 88)
		Yaw = -atan((88 - Yaw_l) / 663.1467) * 57.3;
	else
		Yaw = atan((Yaw_l - 88) / 663.1467) * 57.3; // 三角函数计算Yaw值

	float temp = (593.5 - (motor2006.motor[2].Data.TotalAngle - a23) / 413228 * 128.53);
	Pitch = acos((temp * temp + 543.13405 * 543.13405 - 280.71 * 280.71) / (2 * temp * 543.13405)) * 57.29578;

	if (rc_Ctrl_et.isOnline == 0)
	{
		motor3508.motor[0].Data.Output = 0;
		motor2006.motor[0].Data.Output = 0;
		motor2006.motor[1].Data.Output = 0;
		motor2006.motor[2].Data.Output = 0;
	}
	if (Reload_state != RELOAD_ERROR)
	{
		MotorFillData(&motor3508.motor[0], motor3508.motor[0].Data.Output);
		MotorFillData(&motor2006.motor[0], motor2006.motor[0].Data.Output);
		MotorFillData(&motor2006.motor[1], motor2006.motor[1].Data.Output);
		MotorFillData(&motor2006.motor[2], motor2006.motor[2].Data.Output);
		MotorFillData(&motor6020.motor[0], motor6020.motor[0].Data.Output);
		MotorFillData(&motor2006.motor[3], motor2006.motor[3].Data.Output);
	}
	else
	{
		MotorFillData(&motor3508.motor[0], 0);
		MotorFillData(&motor2006.motor[0], 0);
		MotorFillData(&motor2006.motor[1], 0);
		MotorFillData(&motor2006.motor[2], 0);
		MotorFillData(&motor6020.motor[0], 0);
		MotorFillData(&motor2006.motor[3], 0);
	}

	MotorCanOutput(can2, 0x1FF);
	MotorCanOutput(can1, 0x1FF);
	MotorCanOutput(can2, 0x200);
	Referee_Judge();
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

uint8_t servo_check_number(uint8_t buf[]) // 校验位
{
	uint8_t i;
	uint16_t temp = 0;
	for (i = 2; i < buf[3] + 2; i++)
	{
		temp += buf[i];
	}
	temp = ~temp;
	i = (uint8_t)temp;
	return i;
}

void servo_move(uint16_t id, uint16_t time, int16_t angle) // 幻尔舵机 位置控制模式
{
	static uint8_t buf[10];
	buf[0] = buf[1] = 0x55;
	buf[2] = id;
	buf[3] = 7;
	buf[4] = 1;
	buf[5] = ((uint8_t)(angle));
	buf[6] = ((uint8_t)((angle) >> 8));
	buf[7] = ((uint8_t)(time));
	buf[8] = ((uint8_t)(time >> 8));
	buf[9] = servo_check_number(buf);
	HAL_UART_Transmit_DMA(&huart5, (unsigned char *)buf, 10);
}

void TIM13_Task(void) // 精准控制舵机角度另开的定时器
{
	Timer++;
	if (Timer <= flag_shoot)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	if (Timer == 200)
		Timer = 0;
}

uint8_t Carema_callback(uint8_t *recBuffer, uint16_t len) // 上位机输出摄像头数据，包含视觉偏移和目标角度，前者用于修正Yaw轴误差
{
	if (len == 6)
	{
		if (recBuffer[0] == 0xAA && recBuffer[5] == 0xDD)
		{
			Camera_cnt++;
			memcpy(&Yaw_add, &recBuffer[1], 4);
		}
	}
}

float InversePitchCalculation(float Pitch) // 根据目标Pitch角度反解2006电机的目标角度
{
	const float A = 543.13405f;
	const float B = 280.71f;

	// 1. 角度转弧度
	float Pitch_rad = Pitch / 57.29578f;

	// 2. 解二次方程求Pitch_l
	float cos_val = cosf(Pitch_rad);
	float discriminant = A * A * cos_val * cos_val - (A * A - B * B);
	float Pitch_l = A * cos_val + sqrtf(discriminant); // 取正根

	// 3. 反解电机角度
	float motor_angle = (593.5f - Pitch_l) * 413228.0f / 128.53f;

	return motor_angle;
}

void up_stop(void) // 上堵转检测，通过或GPIOE4连接微动开关实现，微动开关常闭，堵转时被压下断开，持续一定时间则认为上堵转，3508电机目标角度设定为当前角度-200
{
	if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == 0)
		cnt_up_stop++;
	else
		cnt_up_stop = 0;
	if (cnt_up_stop > 5)
	{
		cnt_up_stop = 0;
		flag_stop = 1;
		Target_Angle_3508 = motor3508.motor[0].Data.TotalAngle - 200;
		motor3508.motor[0].Data.Target = 0;
	}
}

void down_stop(void) // 下堵转检测
{
	static int cnt_down_stop = 0;
	if (abs(motor3508.motor[0].Data.Target - motor3508.motor[0].Data.SpeedRPM) > 350 && flag_stop == 0) // 堵转判断
		cnt_down_stop++;
	else
		cnt_down_stop = 0;
	if (cnt_down_stop >= 200)
	{
		cnt_down_stop = 0;
		Target_Angle_3508 = motor3508.motor[0].Data.TotalAngle + 100;
		flag_stop = 1;
		motor3508.motor[0].Data.Target = 0;
	}
}

void servo_control(void) // 舵机控制，换弹时黄色舵机置位，发射时红色舵机切换状态
{
	static int cnt_shoot = 0;
	if (abs(rc_Ctrl_et.rc.ch0 - 1024) > 500 && rc_Ctrl_et.rc.s2 != 2 && referee2022.game_status.game_progress != 4)
		cnt_shoot++;
	else
		cnt_shoot = 0;

	if (cnt_shoot >= 1000)
	{
		if (flag_shoot == servo_red_min)
			flag_shoot = servo_red_max;
		else
			flag_shoot = servo_red_min;
		cnt_shoot = 0;
	}
}
