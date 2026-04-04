/**
 ******************************************************************************
 * @file           : slider.c
 * @brief          : Slider滑动机构控制模块 - 实现文件
 ******************************************************************************
 * @attention
 * 
 * 包含3508电机控制和限位开关检测
 * 支持速度环和角度环两种控制模式
 * 
 ******************************************************************************
 */

#include "slider.h"
#include <math.h>

// ==================== 全局变量 ====================
Slider_t g_slider;

// ==================== 辅助宏 ====================
#ifndef LIMIT
#define LIMIT(x, max, min) (((x) > (max)) ? (max) : (((x) < (min)) ? (min) : (x)))
#endif

#ifndef abs
#define abs(x) ((x) > 0 ? (x) : (-(x)))
#endif

static void Slider_AngleControl(void);
static void Slider_SpeedControl(void);
static uint8_t Slider_TriggerChecksum(const uint8_t *buf)
{
    uint8_t i;
    uint16_t sum = 0;

    for (i = 2; i < (uint8_t)(buf[3] + 2); i++)
    {
        sum += buf[i];
    }

    return (uint8_t)(~sum);
}



// ==================== 外部API接口 ====================

/**
 * @brief  向上运动（速度环控制）
 * @param  speed: 目标速度（正值向上）
 */
void Slider_MoveUp(int16_t speed)
{
    g_slider.enabled = 1;
    g_slider.target_speed = speed;
    g_slider.control_mode = SLIDER_MODE_SPEED;
    g_slider.state = SLIDER_STATE_MOVING_UP;
}

/**
 * @brief  向下运动（速度环控制）
 * @param  speed: 目标速度（负值向下）
 */
void Slider_MoveDown(int16_t speed)
{
    g_slider.enabled = 1;
    g_slider.target_speed = speed;
    g_slider.control_mode = SLIDER_MODE_SPEED;
    g_slider.state = SLIDER_STATE_MOVING_DOWN;
}

/**
 * @brief  停止运动，切换到角度环控制
 * @param  angle: 目标角度（保持当前位置）
 */
void Slider_Stop(void)
{
    // 记录当前位置作为角度环目标
    g_slider.target_angle = g_slider.motor.Data.TotalAngle;
    g_slider.control_mode = SLIDER_MODE_ANGLE;
}

/**
 * @brief  复位到空闲状态
 */
void Slider_Reset(void)
{
    g_slider.state = SLIDER_STATE_IDLE;
    g_slider.target_speed = 0;
    g_slider.motor.Data.Output = 0;
    g_slider.jam_count = 0;
    g_slider.limit_up_count = 0;
    g_slider.enabled = 0;
}

// ==================== 初始化与配置 ====================

/**
 * @brief  Slider初始化 - 默认配置
 */
void Slider_Init(void)
{
    // 初始化电机参数
    MotorInit(&g_slider.motor, 0, Motor3508, CAN2, 0x203);
    
    // 初始化PID参数 - 用户需要自行配置
    BasePID_Init(&g_slider.speed_pid, 15, 0, 0, 0);
    BasePID_Init(&g_slider.angle_pid, 1, 0, 0, 0);
    
    // 初始化限位开关 - 默认配置，用户需自行修改
    // TODO: 请根据实际硬件配置设置以下参数
    // 示例配置：GPIOE, GPIO_PIN_4（用户可根据实际硬件修改）
    g_slider.limit_up.GPIOx = GPIOE;
    g_slider.limit_up.GPIO_Pin = GPIO_PIN_4;
    g_slider.limit_up.active_level = 0;
    
    // 初始化控制模式
    g_slider.control_mode = SLIDER_MODE_SPEED;
    
    // 初始化状态
    g_slider.state = SLIDER_STATE_IDLE;
    g_slider.target_speed = 0;
    g_slider.target_angle = 0;
    g_slider.enabled = 0;
    
    // 初始化堵转参数
    g_slider.jam_count = 0;
    g_slider.jam_angle_before = 0;
    g_slider.jam_speed_diff = 350.0f;  // 默认阈值
    g_slider.jam_threshold = 100;       // 默认判定次数
    
    // 初始化微动开关检测参数
    g_slider.limit_up_count = 0;
    g_slider.limit_up_threshold = 50;   // 默认连续5次触发判定

    // 初始化扳机舵机参数（扳机）
    g_slider.trigger_state = SLIDER_TRIGGER_CLOSE;
    g_slider.trigger_pos_close = 650;
    g_slider.trigger_pos_open = 900;

}




// ==================== 控制模式切换 ====================

/**
 * @brief  设置控制模式
 * @param  mode: SLIDER_MODE_SPEED 或 SLIDER_MODE_ANGLE
 */
void Slider_SetMode(SliderMode_t mode)
{
    g_slider.control_mode = mode;
}

/**
 * @brief  使能/禁用Slider
 * @param  enable: 1=使能, 0=禁用
 */
void Slider_Enable(uint8_t enable)
{
    g_slider.enabled = enable;
    if(!enable) {
        g_slider.motor.Data.Output = 0;
        g_slider.state = SLIDER_STATE_IDLE;
    }
}


// ==================== 主控制函数 ====================

/**
 * @brief  Slider主控制函数 - 需要在定时器中周期调用
 * @note  该函数处理状态转换：
 *       - 向上运动中：检测微动开关，触发后切换到STOP_UP状态（角度环）
 *       - 向下运动中：检测堵转，触发后切换到STOP_DOWN状态（角度环）
 */
void Slider_Control(void)
{
    if(!g_slider.enabled) {
        g_slider.motor.Data.Output = 0;
        return;
    }
    
    // 根据当前状态处理
    switch(g_slider.state) {
        case SLIDER_STATE_MOVING_UP:
            // 向上运动中 - 检测微动开关（连续触发判定）
            if(Slider_GetLimitUp()) {
                g_slider.limit_up_count++;
                if(g_slider.limit_up_count >= g_slider.limit_up_threshold) {
                    // 连续触发达到阈值，切换到角度环控制
                    g_slider.limit_up_count = 0;
                    g_slider.target_angle = g_slider.motor.Data.TotalAngle;
                    g_slider.control_mode = SLIDER_MODE_ANGLE;
                    g_slider.state = SLIDER_STATE_STOP_UP;
                }
            } else {
                // 未触发时计数清零
                g_slider.limit_up_count = 0;
                // 速度环控制
                Slider_SpeedControl();
            }
            break;
            
        case SLIDER_STATE_MOVING_DOWN:
            // 向下运动中 - 检测堵转
            {
                float speed_diff = abs(g_slider.motor.Data.Target - g_slider.motor.Data.SpeedRPM);
                if(speed_diff > g_slider.jam_speed_diff) {
                    g_slider.jam_count++;
                } else {
                    g_slider.jam_count = 0;
                }
                
                if(g_slider.jam_count > g_slider.jam_threshold) {
                    // 堵转触发，切换到角度环控制
                    g_slider.jam_count = 0;
                    g_slider.jam_angle_before = g_slider.motor.Data.TotalAngle;
                    g_slider.target_angle = g_slider.motor.Data.TotalAngle + 100; // 微调位置
                    g_slider.control_mode = SLIDER_MODE_ANGLE;
                    g_slider.state = SLIDER_STATE_STOP_DOWN;
                } else {
                    // 速度环控制
                    Slider_SpeedControl();
                }
            }
            break;
            
        case SLIDER_STATE_STOP_UP:
        case SLIDER_STATE_STOP_DOWN:
            // 停止状态 - 角度环控制
            Slider_AngleControl();
            break;
            
        case SLIDER_STATE_IDLE: g_slider.motor.Data.Output=0;break;
        case SLIDER_STATE_ERROR:
        default:
            g_slider.motor.Data.Output = 0;
            break;
    }
}

/**
 * @brief  速度环控制
 */
static void Slider_SpeedControl(void)
{
    g_slider.motor.Data.Target = g_slider.target_speed;
    g_slider.motor.Data.Output = 
        BasePID_SpeedControl(&g_slider.speed_pid, 
                           g_slider.motor.Data.Target, 
                           g_slider.motor.Data.SpeedRPM);
}

/**
 * @brief  角度环控制
 */
static void Slider_AngleControl(void)
{
    // 角度环 -> 速度环 -> 电机输出
    int32_t speed_output = 
        BasePID_AngleControl(&g_slider.angle_pid,
                           g_slider.target_angle,
                           g_slider.motor.Data.TotalAngle);
    
    g_slider.motor.Data.Target = speed_output;
    g_slider.motor.Data.Output = 
        BasePID_SpeedControl(&g_slider.speed_pid,
                           g_slider.motor.Data.Target,
                           g_slider.motor.Data.SpeedRPM);
}

// ==================== 状态读取 ====================

/**
 * @brief  获取Slider状态
 */
SliderState_t Slider_GetState(void)
{
    return g_slider.state;
}

/**
 * @brief  获取上限位开关状态
 * @return 1=触发, 0=未触发
 */
uint8_t Slider_GetLimitUp(void)
{
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(g_slider.limit_up.GPIOx, 
                                                g_slider.limit_up.GPIO_Pin);
    if(g_slider.limit_up.active_level == 0) {
        return (pin_state == GPIO_PIN_RESET) ? 1 : 0;
    } else {
        return (pin_state == GPIO_PIN_SET) ? 1 : 0;
    }
}


/**
 * @brief  获取使能状态
 */
uint8_t Slider_IsEnabled(void)
{
    return g_slider.enabled;
}

// ==================== 堵转检测 ====================

/**
 * @brief  堵转检测（独立检测函数）
 */
void Slider_JamDetection(void)
{
    // 只在运动状态下检测
    if(g_slider.state != SLIDER_STATE_MOVING_UP && 
       g_slider.state != SLIDER_STATE_MOVING_DOWN) {
        return;
    }
    
    // 检测速度差是否超过阈值
    float speed_diff = abs(g_slider.motor.Data.Target - g_slider.motor.Data.SpeedRPM);
    
    if(speed_diff > g_slider.jam_speed_diff) {
        g_slider.jam_count++;
    } else {
        g_slider.jam_count = 0;
    }
    
    // 连续多次检测到则认为堵转
    if(g_slider.jam_count > g_slider.jam_threshold) {
        g_slider.jam_count = 0;
        g_slider.state = SLIDER_STATE_ERROR;
        
        // 记录堵转位置
        g_slider.jam_angle_before = g_slider.motor.Data.TotalAngle;
        
        // 输出置零
        g_slider.motor.Data.Output = 0;
    }
}


/**
 * @brief 扳机打开
 */
void Slider_TriggerOpen(void)
{
    g_slider.trigger_state = SLIDER_TRIGGER_OPEN;
}

/**
 * @brief 扳机闭合
 */
void Slider_TriggerClose(void)
{
    g_slider.trigger_state = SLIDER_TRIGGER_CLOSE;
}

/**
 * @brief 扳机状态切换
 */
void Slider_TriggerToggle(void)
{
    if (g_slider.trigger_state == SLIDER_TRIGGER_CLOSE)
    {
        Slider_TriggerOpen();
    }
    else
    {
        Slider_TriggerClose();
    }
}

/**
 * @brief 配置扳机舵机参数
 */
void Slider_TriggerConfig( uint16_t close_pos, uint16_t open_pos)
{
    g_slider.trigger_pos_close = close_pos;
    g_slider.trigger_pos_open = open_pos;
}

void Slider_TriggerControl()
{
	static int Timer;
	uint8_t pos;
	Timer++;
	
	if (g_slider.trigger_state==SLIDER_TRIGGER_CLOSE) pos=g_slider.trigger_pos_close;
	else pos=g_slider.trigger_pos_open;
	
	if (Timer <= pos)HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		else HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	
		
	if (Timer == 200)Timer = 0;
		
}
