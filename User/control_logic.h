#ifndef CONTROLLOGIC_H_
#define CONTROLLOGIC_H_

#include <stm32h7xx_hal.h>
#include "hardware_config.h"
#include <swerve_chassis.h>
#include <check.h>

// 包含新文件的头文件
#include "display_comm.h"
#include "referee_control.h"
#include "state_machines.h"
#include "motion_control.h"

// 常量定义
#define servo_red_min 13
#define servo_red_max 20
#define servo_yellow_max 495

// 数据结构定义
typedef struct
{
	int Yaw_offset;
	float Pitch_offset;
	uint8_t Index;
} Dart_Info;

// 主任务函数
void TIM14_Task(void);

// 短函数声明（20行以内）
void TIM13_Task(void);
uint8_t CAN1_rxCallBack(CAN_RxBuffer* rxBuffer);
uint8_t CAN2_rxCallBack(CAN_RxBuffer* rxBuffer);
uint8_t servo_check_number(uint8_t buf[]);
void servo_move(uint16_t id, uint16_t time, int16_t angle);
uint8_t Carema_callback(uint8_t *recBuffer, uint16_t len);
float InversePitchCalculation(float Pitch);
void up_stop(void);
void down_stop(void);
void servo_control(void);

// 状态枚举
typedef enum {
    RELOAD_IDLE = 0,      // 空闲状态 舵机横置
    RELOAD_STEP1,         // 第一步 
    RELOAD_STEP1_5,       // 第一点五步：6020电机旋转
    RELOAD_STEP2,         // 第二步：2006电机下移
    RELOAD_STEP3,         // 第三步：2006电机上移
    RELOAD_STEP4,         // 第四步：6020电机复位
    RELOAD_COMPLETE,      // 完成
    RELOAD_ERROR
} ReloadState_t; // 换弹状态枚举

typedef enum {
    Rubber_IDLE = 0,      
    Rubber_STEP1,         // 3508下拉    
    Rubber_STEP2,         // 舵机合拢
    Rubber_STEP3,         // 3508上拉
    Rubber_COMPLETE,     
    Rubber_ERROR
} RubberState_t; // 皮筋蓄能状态枚举

// 全局变量声明（在control_logic.c中定义）
extern uint8_t flag_completely;
extern uint8_t change_mode;
extern uint8_t cnt_2006_down, cnt_2006_up, cnt_6020_move, cnt_6020_back;
extern int b1;
extern int cnt__l, flag_none;
extern Dart_Info infor[8];
extern float Yaw_offset;
extern uint8_t Last_dart_launch_opening_status;
extern int Speed_3508;
extern float kll;
extern uint8_t flag_yaw_stop, mode, shot_complete;
extern uint16_t cnt_complete;
extern uint8_t flag_pitch_stop;
extern uint8_t flag_shoot;
extern uint8_t flag_stop, flag_Camera;
extern uint8_t flag_reset;
extern int cnt_2006_yaw, cnt_2006_pitch;
extern int cnt, cnt_2, flag_2006, cnt_2006;
extern float Target_Angle_3508;
extern float Target_Angle_2006;
extern float Target_Angle_2006_yaw;
extern float Target_Angle_2006_pitch;
extern int cnt_servo_;
extern uint8_t flag11, flag_wait;
extern uint16_t cnt1111;
extern float a22, a23;
extern int cntmm;
extern float Yaw, Pitch;
extern int servo_yellow;
extern int Target_Angle_2006_load;
extern uint8_t Reload_mode;
extern float Yaw_add;
extern uint16_t Camera_cnt, Camera_Fps;
extern RubberState_t Rubber_state;
extern ReloadState_t Reload_state;
extern uint16_t cnt6020_down, cnt6020_up, cnt2006_down, cnt2006_up, cnt_servo, cnt_error;
extern float Target_6020;
extern int cnt_up_stop;
extern int Timer;
extern int flll;
extern float Rubber_Reset_Angel;

extern uint8_t Auto_state;

#endif /* CONTROLLOGIC_H_ */



