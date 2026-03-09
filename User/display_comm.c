#include "display_comm.h"
#include "control_logic.h"
#include "hardware_config.h"
#include "usart.h"
#include "stdio.h"
#include "referee.h"
#include "driver_timer.h"
void Send_toled(void)
{
	static int k = 0;
	if (tim14.ClockTime % 80 == k * 5)
		UsartDmaPrintf_("改变角度.x1.val=%d", (int)(Yaw * 1000));
	k++;
	if (tim14.ClockTime % 80 == k * 5)
		UsartDmaPrintf_("改变角度.x0.val=%d", (int)(Pitch * 1000));
	k++;
	if (tim14.ClockTime % 80 == k * 5)
		UsartDmaPrintf_("视觉校准.n1.val=%d", (int)(Yaw_offset));
	k++;
	if (tim14.ClockTime % 80 == k * 5)
		UsartDmaPrintf_("视觉校准.n0.val=%d", (int)Yaw_add);
	k++;
	if (tim14.ClockTime % 80 == k * 5)
		UsartDmaPrintf_("修改Pitch.x1.val=%d", (int)(1000 * infor[0].Pitch_offset));
	k++;
	if (tim14.ClockTime % 80 == k * 5)
		UsartDmaPrintf_("修改Pitch.x2.val=%d", (int)(1000 * infor[1].Pitch_offset));
	k++;
	if (tim14.ClockTime % 80 == k * 5)
		UsartDmaPrintf_("修改Pitch.x3.val=%d", (int)(1000 * infor[2].Pitch_offset));
	k++;
	if (tim14.ClockTime % 80 == k * 5)
		UsartDmaPrintf_("修改Pitch.x4.val=%d", (int)(1000 * infor[3].Pitch_offset));
	k++;
	if (tim14.ClockTime % 80 == k * 5)
		UsartDmaPrintf_("修改Pitch.b2.txt=\"选中: %d\"", (int)(change_mode));
	k++;
}

uint8_t LCD_callback(uint8_t *recBuffer, uint16_t len) // 测试用LCD 含多种功能
{
	if (recBuffer[0] == 0x55 && recBuffer[1] == 0x01 && recBuffer[2] == 0x00 && recBuffer[3] == 0x01) // 换弹
	{
		Reload_mode++;
		if (Reload_state == RELOAD_COMPLETE)
			Reload_state = RELOAD_IDLE;
	}
	else if (recBuffer[0] == 0x55 && recBuffer[1] == 0x01 && recBuffer[2] == 0x00 && recBuffer[3] == 0x00) // 蓄能
	{

		if (Rubber_state == Rubber_IDLE || Rubber_state == Rubber_COMPLETE)
		{
			Rubber_state = Rubber_STEP1;
			flag_stop = 0;
		}
	}
	else if (recBuffer[0] == 0x55 && recBuffer[1] == 0x01 && recBuffer[2] == 0x00 && recBuffer[3] == 0x02) // 一次全流程
	{
		if (Rubber_state == Rubber_IDLE || Rubber_state == Rubber_COMPLETE)
		{
			Rubber_state = Rubber_STEP1;
			flag_stop = 0;
		}
		flag_completely = 1;
		servo_yellow = servo_yellow_max;
	}
	else if (recBuffer[0] == 0x55 && recBuffer[1] == 0x01 && recBuffer[2] == 0x00 && recBuffer[3] == 0x03) // 换弹复位
	{
		Rubber_Reset_Angel = motor2006.motor[0].Data.TotalAngle;
	}
	else if (recBuffer[0] == 0x55 && recBuffer[1] == 0x01 && recBuffer[2] == 0x00 && recBuffer[3] == 0x04) // Yaw Pitch 复位
	{
		flag_reset = 1;
	}

	else if (recBuffer[0] == 0x55 && recBuffer[1] == 0x01 && recBuffer[2] == 0x01 && recBuffer[3] == 0x00) // 闸门正在开启
	{
		mode++;
		flag_none = 1;
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
	else if (recBuffer[0] == 0x55 && recBuffer[1] == 0x01 && recBuffer[2] == 0x01 && recBuffer[3] == 0x01) // 闸门开启
	{
		if (mode == 1 && Rubber_state == Rubber_COMPLETE)
		{
			if (flag_shoot == servo_red_min)
				flag_shoot = servo_red_max;
			else
				flag_shoot = servo_red_min;
			shot_complete = 1;
		}
		if (mode == 2 && Rubber_state == Rubber_COMPLETE)
		{
			if (flag_shoot == servo_red_min)
				flag_shoot = servo_red_max;
			else
				flag_shoot = servo_red_min;
			shot_complete = 1;
		}
	}
	else if (recBuffer[0] == 0x55 && recBuffer[1] == 0x01 && recBuffer[2] == 0x02 && recBuffer[3] == 0x00) // 视觉标定
	{
		if (flag_Camera)
			flag_Camera = 0;
		else
			flag_Camera = 1;
		if (Yaw_add != 0)
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
	else if (recBuffer[0] == 0x55 && recBuffer[1] == 0x01 && recBuffer[2] == 0x02 && recBuffer[3] == 0x01) // 视觉偏移++
	{

		Yaw_offset += 2;
	}
	else if (recBuffer[0] == 0x55 && recBuffer[1] == 0x01 && recBuffer[2] == 0x02 && recBuffer[3] == 0x02) // 视觉偏移--
	{

		Yaw_offset -= 2;
	}
	else if (recBuffer[0] == 0x55 && recBuffer[1] == 0x01 && recBuffer[2] == 0x03 && recBuffer[3] == 0x01) // Pitch++
	{

		infor[change_mode - 1].Pitch_offset += 0.05;
	}
	else if (recBuffer[0] == 0x55 && recBuffer[1] == 0x01 && recBuffer[2] == 0x03 && recBuffer[3] == 0x02) // Pitch--
	{

		infor[change_mode - 1].Pitch_offset -= 0.05;
	}
	else if (recBuffer[0] == 0x55 && recBuffer[1] == 0x01 && recBuffer[2] == 0x03 && recBuffer[3] == 0x03) // 选择修改镖号
	{
		change_mode++;
		if (change_mode == 5)
			change_mode = 1;
	}
	return 0;
}

void Lcd_control(void) // 通过LCD模拟比赛两次发射信号
{

	if (flag_none)
		cnt__l++;
	if (cnt__l >= 18000)
	{

		cnt__l = 0;
		flag_none = 0;
		if (mode == 1 && Rubber_state == Rubber_COMPLETE)
		{
			if (flag_shoot == servo_red_min)
				flag_shoot = servo_red_max;
			else
				flag_shoot = servo_red_min;
			shot_complete = 1;
		}
		if (mode == 2 && Rubber_state == Rubber_COMPLETE)
		{
			if (flag_shoot == servo_red_min)
				flag_shoot = servo_red_max;
			else
				flag_shoot = servo_red_min;
			shot_complete = 1;
		}
	}
	if (shot_complete == 1)
		cnt_complete++;
	if (cnt_complete > 1500)
	{
		shot_complete = 2;
		cnt_complete = 0;
	}
	if (shot_complete == 2)
	{
		if (Rubber_state == Rubber_IDLE || Rubber_state == Rubber_COMPLETE)
		{
			Rubber_state = Rubber_STEP1;
			flag_stop = 0;
		}
		flag_completely = 1;
		servo_yellow = servo_yellow_max;
		shot_complete = 3;
	}

	if (shot_complete == 3 && Rubber_state == Rubber_COMPLETE && Reload_state == RELOAD_COMPLETE) //
	{
		if (flag_shoot == servo_red_min)
			flag_shoot = servo_red_max;
		else
			flag_shoot = servo_red_min;
		shot_complete = 0;
	}
}