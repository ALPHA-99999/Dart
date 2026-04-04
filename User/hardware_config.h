#ifndef HARDWARE_CONFIG_H_
#define HARDWARE_CONFIG_H_

#include <motor.h>
#include "stm32h7xx_hal.h"
#include "usart.h"
#include <swerve_chassis.h>
#include "mecanum_chassis.h"
#include <control_logic.h>
#include <attitude.h>
#include "lifting.h"
#include "protract.h"
#include "sideway.h"
#include "servo.h"
#include <ore.h>
#include "Custom_controller.h"`r`n#include "gimbal_axis.h"
void HardwareConfig(void);
uint8_t vofa_callback(uint8_t *recBuffer, uint16_t len);
extern BasePID_Object Motors3508_SpeedPID;
extern BasePID_Object Motors3508_AngelPID;
extern BasePID_Object pid_yaw_angle; // ��̬��е��PID
extern BasePID_Object pid_yaw_speed;
extern BasePID_Object pid_pitch_angle;
extern BasePID_Object pid_pitch_speed;
extern BasePID_Object pid_yawreset;
extern BasePID_Object pid_pitchreset;
extern BasePID_Object run_pid;
extern MecanumChassis mecanumchassis;

extern BasePID_Object Motors2006_load_SpeedPID;
extern BasePID_Object Motors2006_load_AngelPID;
extern BasePID_Object Motors6020_pitch_SpeedPID;
extern BasePID_Object Motors6020_pitch_AngelPID;

extern BasePID_Object Motors2006_SpeedPID;
extern BasePID_Object Motors2006_AngelPID;
extern BasePID_Object Motors2006_yaw_SpeedPID;
extern BasePID_Object Motors2006_yaw_AngelPID;
extern BasePID_Object Motors2006_pitch_SpeedPID;
extern BasePID_Object Motors2006_pitch_AngelPID;
// extern MecanumChassis mecanumChassis;
// extern SwerveChassis swerveChassis;
extern Attitude_t attiTude;

#endif

