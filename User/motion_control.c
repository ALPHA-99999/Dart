#include "motion_control.h"
#include "control_logic.h"
#include "hardware_config.h"

void Yaw_Pitch_Reset(void)
{
	static int cnt_yaw_stop=0,cnt_pitch_stop=0;
	if (flag_reset == 1)
	{
		motor2006.motor[1].Data.Target = -4000;
		motor2006.motor[2].Data.Target = -6000;

		if (flag_yaw_stop == 0)
			motor2006.motor[1].Data.Output = BasePID_SpeedControl(&run_pid, motor2006.motor[1].Data.Target, motor2006.motor[1].Data.SpeedRPM);
		else
		{
			motor2006.motor[1].Data.Output =
				BasePID_PitchSpeedControl((BasePID_Object *)(&Motors2006_yaw_SpeedPID),
										  BasePID_YawAngleControl((BasePID_Object *)(&Motors2006_yaw_AngelPID), Target_Angle_2006_yaw, motor2006.motor[1].Data.TotalAngle), motor2006.motor[1].Data.SpeedRPM);
			if (motor2006.motor[1].Data.Output > 5000)
				motor2006.motor[1].Data.Output = 5000;
			else if (motor2006.motor[1].Data.Output < -5000)
				motor2006.motor[1].Data.Output = -5000;
		}

		if (flag_pitch_stop == 0)
			motor2006.motor[2].Data.Output = BasePID_SpeedControl(&run_pid, motor2006.motor[2].Data.Target, motor2006.motor[2].Data.SpeedRPM);
		else
		{
			motor2006.motor[2].Data.Output =
				BasePID_PitchSpeedControl((BasePID_Object *)(&Motors2006_pitch_SpeedPID),
										  BasePID_YawAngleControl((BasePID_Object *)(&Motors2006_pitch_AngelPID), Target_Angle_2006_pitch, motor2006.motor[2].Data.TotalAngle), motor2006.motor[2].Data.SpeedRPM);
			if (motor2006.motor[2].Data.Output > 5000)
				motor2006.motor[2].Data.Output = 5000;
			else if (motor2006.motor[2].Data.Output < -5000)
				motor2006.motor[2].Data.Output = -5000;
		}

		if (abs(motor2006.motor[1].Data.Target - motor2006.motor[1].Data.SpeedRPM) > 250 && flag_yaw_stop == 0) // 堵转判断
			cnt_yaw_stop++;
		else
			cnt_yaw_stop = 0;
		if (cnt_yaw_stop >= 50)
		{
			cnt_yaw_stop = 0;

			flag_yaw_stop = 1;
			a22 = motor2006.motor[1].Data.TotalAngle;
			// Target_Angle_2006_yaw=motor2006.motor[1].Data.TotalAngle+391263;
			//			Target_Angle_2006_yaw=motor2006.motor[1].Data.TotalAngle+191263;
			Target_Angle_2006_yaw = motor2006.motor[1].Data.TotalAngle + 111263;
		}

		if (abs(motor2006.motor[2].Data.Target - motor2006.motor[2].Data.SpeedRPM) > 250 && flag_pitch_stop == 0) // 堵转判断
			cnt_pitch_stop++;
		else
			cnt_pitch_stop = 0;
		if (cnt_pitch_stop >= 50)
		{
			cnt_pitch_stop = 0;
			a23 = motor2006.motor[2].Data.TotalAngle;
			flag_pitch_stop = 1;
			Target_Angle_2006_pitch = motor2006.motor[2].Data.TotalAngle + 170000;
		}
	}
}

void Change_YawPitch_use_remotecontrol(void)
{
	if (abs(rc_Ctrl_et.rc.ch1 - 1024) > 500 && rc_Ctrl_et.rc.s2 == 2)
		cnt_2006_pitch++;
	else
		cnt_2006_pitch = 0;

	if (abs(rc_Ctrl_et.rc.ch0 - 1024) > 500 && rc_Ctrl_et.rc.s2 == 2)
		cnt_2006_yaw++;
	else
		cnt_2006_yaw = 0;

	if (cnt_2006_pitch >= 800 && rc_Ctrl_et.rc.ch1 - 1024 >= 0)
	{
		Target_Angle_2006_pitch -= 12900;
		cnt_2006_pitch = 0;
	}
	if (cnt_2006_pitch >= 800 && rc_Ctrl_et.rc.ch1 - 1024 <= 0)
	{
		Target_Angle_2006_pitch += 12900;
		cnt_2006_pitch = 0;
	}
	if (cnt_2006_yaw >= 800 && rc_Ctrl_et.rc.ch0 - 1024 >= 0)
	{
		Target_Angle_2006_yaw -= 3800;
		cnt_2006_yaw = 0;
	}
	if (cnt_2006_yaw >= 800 && rc_Ctrl_et.rc.ch0 - 1024 <= 0)
	{
		Target_Angle_2006_yaw += 3800;
		cnt_2006_yaw = 0;
	}

	if (abs(rc_Ctrl_et.rc.ch2 - 1024) > 500 && rc_Ctrl_et.rc.s2 != 2)
		cnt_2006++;
	else
		cnt_2006 = 0;

	if (cnt_2006 >= 800 && rc_Ctrl_et.rc.ch2 - 1024 >= 0)
	{
		Target_Angle_2006 -= 1900;
		cnt_2006 = 0;
	}
	else if (cnt_2006 >= 800 && rc_Ctrl_et.rc.ch2 - 1024 < 0)
	{
		Target_Angle_2006 += 1900;
		cnt_2006 = 0;
	}
}