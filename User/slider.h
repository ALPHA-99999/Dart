/**
 ******************************************************************************
 * @file           : slider.h
 * @brief          : Slider滑动机构控制模块 - 头文件
 ******************************************************************************
 * @attention
 *
 * 包含3508电机控制和限位开关检测
 * 支持速度环和角度环两种控制模式
 *
 ******************************************************************************
 */

#ifndef SLIDER_H_
#define SLIDER_H_

#include <stdint.h>
#include <stm32h7xx_hal.h>

#include "hardware_config.h"
#include "motor.h"
#include "pid.h"
#include "driver_can.h"

// ==================== 模式定义 ====================
typedef enum {
    SLIDER_MODE_SPEED = 0,
    SLIDER_MODE_ANGLE
} SliderMode_t;

// ==================== 状态枚举 ====================
typedef enum {
    SLIDER_STATE_IDLE = 0,
    SLIDER_STATE_MOVING_UP,
    SLIDER_STATE_MOVING_DOWN,
    SLIDER_STATE_STOP_UP,
    SLIDER_STATE_STOP_DOWN,
    SLIDER_STATE_ERROR
} SliderState_e;

typedef SliderState_e SliderState_t;

// ==================== GPIO限位开关配置 ====================
typedef struct {
    GPIO_TypeDef*  GPIOx;
    uint16_t       GPIO_Pin;
    uint8_t        active_level;
} SliderLimitSwitch_t;

// ==================== 扳机舵机状态 ====================
typedef enum {
    SLIDER_TRIGGER_CLOSE = 0,
    SLIDER_TRIGGER_OPEN
} SliderTriggerState_t;

// ==================== Slider结构体 ====================
typedef struct {
    Motor               motor;

    BasePID_Object      speed_pid;
    BasePID_Object      angle_pid;

    SliderLimitSwitch_t limit_up;
    SliderLimitSwitch_t limit_down;

    SliderMode_t        control_mode;

    SliderState_t       state;
    int16_t             target_speed;
    float               target_angle;

    uint8_t             enabled;

    uint16_t            jam_count;
    float               jam_angle_before;
    float               jam_speed_diff;
    uint16_t            jam_threshold;

    uint16_t            limit_up_count;
    uint16_t            limit_up_threshold;

    SliderTriggerState_t trigger_state;
    uint16_t            trigger_pos_close;
    uint16_t            trigger_pos_open;

} Slider_t;

extern Slider_t g_slider;

void Slider_Init(void);
void Slider_SetMode(SliderMode_t mode);
void Slider_Enable(uint8_t enable);
void Slider_Control(void);

SliderState_t Slider_GetState(void);
uint8_t Slider_GetLimitUp(void);
uint8_t Slider_IsEnabled(void);

void Slider_MoveUp(int16_t speed);
void Slider_MoveDown(int16_t speed);
void Slider_Stop(void);
void Slider_Reset(void);
void Slider_JamDetection(void);

// 扳机舵机控制（独立调用，不进Slider_Control）
void Slider_TriggerOpen(void);
void Slider_TriggerClose(void);
void Slider_TriggerToggle(void);
void Slider_TriggerConfig(uint16_t close_pos,uint16_t open_pos);
void Slider_TriggerControl();
#endif /* SLIDER_H_ */
