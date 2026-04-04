#ifndef __PID_H_
#define __PID_H_
#include <stm32h7xx_hal.h>

typedef struct
{
	float Kp, Ki, Kd;
	float Error;
	float KpPart, KiPart, KdPart;
	float Out;
	float KiPartDetachment;
	float LastError;
	float Speed_detach;
	float Speed_detach_Negative;
} BasePID_Object;

typedef struct
{
	BasePID_Object *ShellPID;
	BasePID_Object *CorePID;
} DualPID_Object;

void BasePID_Init(BasePID_Object *base_pid, float kp, float ki, float kd, float detach);
float BasePID_SpeedControl(BasePID_Object *base_pid, float target_speed, float feedback_speed);
int32_t BasePID_AngleControl(BasePID_Object *base_pid, float target_angle, float feedback_angle);
int32_t BasePID_LiftingSpeedControl(BasePID_Object *base_pid, float target_speed, float feedback_speed);
int32_t BasePID_LiftingAngleControl(BasePID_Object *base_pid, float target_angle, float feedback_angle);
int32_t BasePID_PitchAngleControl(BasePID_Object *base_pid, float target_angle, float feedback_angle);
int32_t BasePID_PitchSpeedControl(BasePID_Object *base_pid, float target_speed, float feedback_speed);
int32_t BasePID_YawAngleControl(BasePID_Object *base_pid, float target_angle, float feedback_angle);
int32_t BasePID_YawSpeedControl(BasePID_Object *base_pid, float target_speed, float feedback_speed);
int32_t BasePID_SidewaySpeedControl(BasePID_Object *base_pid, float target_speed, float feedback_speed);
int32_t BasePID_SidewayAngleControl(BasePID_Object *base_pid, float target_angle, float feedback_angle);
int32_t BasePID_mecaSpeedControl(BasePID_Object *base_pid, float target_speed, float feedback_speed);
int32_t BasePID_mecaAngleControl(BasePID_Object *base_pid, float target_angle, float feedback_angle);
#endif
