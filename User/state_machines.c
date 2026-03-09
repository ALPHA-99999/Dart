#include "state_machines.h"
#include "control_logic.h"
#include "hardware_config.h"

void Reload_change(void)
{
	switch (Reload_state)
	{
	case RELOAD_IDLE:
		if (Reload_mode) // 舵机置位
		{

			servo_yellow = servo_yellow_max;
			cnt_servo++;
			if (cnt_servo > 1000)
			{
				cnt_servo = 0;
				Reload_state = RELOAD_STEP1;
			}
		}
		break;

	case RELOAD_STEP1: // 根据每一个镖不同参数调整6020及yaw和pitch偏差角
		if (Reload_mode == 1)
		{
			Target_6020 += 91;
			Yaw_offset = infor[1].Yaw_offset;
			Target_Angle_2006_pitch = InversePitchCalculation(infor[1].Pitch_offset) + a23;
		}
		else if (Reload_mode == 2)
		{
			Target_6020 += 181;
			Yaw_offset = infor[2].Yaw_offset;
			Target_Angle_2006_pitch = InversePitchCalculation(infor[2].Pitch_offset) + a23;
		}
		else if (Reload_mode == 3)
		{
			Target_6020 += 270;
			Yaw_offset = infor[3].Yaw_offset;
			Target_Angle_2006_pitch = InversePitchCalculation(infor[3].Pitch_offset) + a23;
		}
		Reload_state = RELOAD_STEP1_5;
		break;

	case RELOAD_STEP1_5: // 6020旋转到位检测
		if (abs(Target_6020 - motor6020.motor[0].Data.TotalAngle) < 1.5)
			cnt_6020_move++;
		else
			cnt_6020_move = 0;

		if (cnt_6020_move > 50)
		{
			cnt_6020_move = 0;
			Reload_state = RELOAD_STEP2;
			Target_Angle_2006_load -= 60436; // 初始化第二步目标
		}

		break;

	case RELOAD_STEP2: // 2006下降检测及异常检测(飞镖下落不顺畅导致卡住)
		if (abs(motor2006.motor[3].Data.SpeedRPM - Motors2006_load_AngelPID.Out) > 200)
			cnt_error++;
		else
			cnt_error = 0;
		if (cnt_error > 100)
			Reload_state = RELOAD_ERROR;

		if (abs(Target_Angle_2006_load - motor2006.motor[3].Data.TotalAngle) < 200)
			cnt_2006_down++;
		else
			cnt_2006_down = 0;

		if (cnt_2006_down > 100)
		{
			cnt_2006_down = 0;

			Reload_state = RELOAD_STEP3;
			Target_Angle_2006_load += 60436; // 初始化第三步目标
		}

		break;

	case RELOAD_STEP3: // 2006上升检测
		if (abs(Target_Angle_2006_load - motor2006.motor[3].Data.TotalAngle) < 9000)
			cnt_2006_up++;
		else
			cnt_2006_up = 0;
		if (abs(Target_Angle_2006_load - motor2006.motor[3].Data.TotalAngle) < 45000)
			cnt_servo++;
		else
			cnt_servo = 0;

		if (cnt_servo > 50)
		{
			cnt_servo = 0;
			servo_yellow = 120;
		}

		if (cnt_2006_up > 100)
		{
			cnt_2006_up = 0;
			Reload_state = RELOAD_STEP4;
			if (Reload_mode == 1)
				Target_6020 -= 91; // 初始化第一步目标
			else if (Reload_mode == 2)
				Target_6020 -= 181;
			else if (Reload_mode == 3)
				Target_6020 -= 270;
		}

		break;
	case RELOAD_STEP4:
		if (abs(Target_6020 - motor6020.motor[0].Data.TotalAngle) < 1)
			cnt_6020_back++;
		else
			cnt_6020_back = 0;

		if (cnt_6020_back > 100)
		{
			cnt_6020_back = 0;
			Reload_state = RELOAD_COMPLETE;
		}

		break;
	case RELOAD_COMPLETE:
		break;
	case RELOAD_ERROR:
		motor2006.motor[3].Data.Output = 0;
		motor6020.motor[0].Data.Output = 0;
		servo_yellow = servo_yellow_max;
		break;
	}
}

void Rubber_change(void)
{
	switch (Rubber_state)
	{
	case Rubber_IDLE:
		break;

	case Rubber_STEP1: // 下降 3508转速-5000 堵转检测跳到Rubber_STEP2
	{
		Speed_3508 = -5000;
		if (flag_stop == 1)
			Rubber_state = Rubber_STEP2;
	}

	break;

	case Rubber_STEP2: // 舵机角度最大，开关闭合
		flag_shoot = servo_red_max;
		cnt_servo_++;
		if (flag_completely == 1)
		{
			Reload_mode++;
			if (Reload_state == RELOAD_COMPLETE)
				Reload_state = RELOAD_STEP1;
			flag_completely = 0;
		}
		if (cnt_servo_ > 1500) // 等待舵机动作完成 跳到Rubber_STEP3
		{
			cnt_servo_ = 0;
			Rubber_state = Rubber_STEP3;
			flag_stop = 0;
		}

		break;

	case Rubber_STEP3:
		if (Reload_mode == RELOAD_IDLE || Reload_state == RELOAD_COMPLETE) // 上升3508转速6000，等待换弹完成后跳到Rubber_COMPLETE
		{
			Speed_3508 = 6000;
			if (flag_stop == 1)
				Rubber_state = Rubber_COMPLETE;
		}
		break;
	case Rubber_COMPLETE:
		break;
	case Rubber_ERROR:
		break;
	}
}