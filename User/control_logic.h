#ifndef CONTROLLOGIC_H_
#define CONTROLLOGIC_H_

#include <stm32h7xx_hal.h>
#include "hardware_config.h"
#include <swerve_chassis.h>
#include <check.h>

// 包含新文件的头文件


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



#endif /* CONTROLLOGIC_H_ */



