#include "referee_control.h"
#include "control_logic.h"
#include "hardware_config.h"
#include "referee.h"

void Referee_Judge(void)
{
	if (Last_dart_launch_opening_status == 1 && referee2022.dart_client_cmd.dart_launch_opening_status == 2 && referee2022.game_status.stage_remain_time <= 400 && referee2022.game_status.game_progress == 4)
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
	if (Last_dart_launch_opening_status == 2 && referee2022.dart_client_cmd.dart_launch_opening_status == 0 && referee2022.game_status.stage_remain_time <= 400 && referee2022.game_status.game_progress == 4)
	{
		flag_wait = 1;
	}
	if (flag_wait == 1 && Rubber_state == Rubber_COMPLETE)
	{
		if (mode == 1)
		{
			if (flag_shoot == servo_red_min)
				flag_shoot = servo_red_max;
			else
				flag_shoot = servo_red_min;
			shot_complete = 1;
		}
		if (mode == 2)
		{
			if (flag_shoot == servo_red_min)
				flag_shoot = servo_red_max;
			else
				flag_shoot = servo_red_min;
			shot_complete = 1;
		}
		flag_wait = 0;
	}
	Last_dart_launch_opening_status = referee2022.dart_client_cmd.dart_launch_opening_status;
}