/**
 ******************************************************************************
 * @file           : robot_control.h
 * @brief          : 飞镖机器人统一控制系统 - 状态管理和任务调度
 ******************************************************************************
 * @attention
 * 
 * 优化版本：整合所有状态和任务调度，提高代码可维护性
 * 
 ******************************************************************************
 */

#ifndef ROBOT_CONTROL_H_
#define ROBOT_CONTROL_H_

#include <stm32h7xx_hal.h>
#include "hardware_config.h"
#include "control_logic.h"

// ==================== 常量定义 ====================
// 角度调整步长
#define YAW_ADJUST_STEP               3800    // 遥控器Yaw调整步长
#define PITCH_ADJUST_STEP             12900   // 遥控器Pitch调整步长
#define PLUNGER_ADJUST_STEP           1900    // 推弹机构调整步长

// 机构参数
#define DART_LOAD_ANGLE               60436   // 镖加载角度 (2006电机)
#define YAW_RESET_OFFSET              111263  // Yaw复位偏移角度
#define PITCH_RESET_OFFSET            170000  // Pitch复位偏移角度
#define DART_6020_ANGLE_DART1        91      // 第一发6020旋转角度
#define DART_6020_ANGLE_DART2        181     // 第二发6020旋转角度
#define DART_6020_ANGLE_DART3        270     // 第三发6020旋转角度
#define DART_MAX_COUNT                3       // 最大镖数量

// 检测阈值
#define JAM_SPEED_THRESHOLD            350     // 堵转速度差阈值
#define JAM_DETECT_COUNT              200     // 堵转判定计数
#define STOP_SWITCH_COUNT             5       // 微动开关检测次数
#define JAM_ERROR_THRESHOLD           200     // 异常检测阈值
#define JAM_ERROR_COUNT               100     // 异常判定计数

// 定时器阈值
#define RELOAD_TIMEOUT_MS             5000    // 换弹超时时间
#define RUBBER_TIMEOUT_MS             3000    // 蓄能超时时间
#define SERVO_ACTION_TIMEOUT          1500    // 舵机动作超时
#define RESET_HOLD_TIME_MS            1000    // 复位保持时间

// 遥控器阈值
#define RC_STICK_DEADZONE             500     // 摇杆死区
#define RC_HOLD_TRIGGER_COUNT         800     // 遥控器按住触发计数
#define RC_SHOOT_TRIGGER_COUNT        1000    // 发射触发计数

// 舵机角度
#define SERVO_YELLOW_RETRACT           120     // 黄色舵机收回角度
#define SERVO_YELLOW_EXTEND           495     // 黄色舵机伸出角度
#define SERVO_RED_MIN                 13      // 红色舵机最小角度
#define SERVO_RED_MAX                 20      // 红色舵机最大角度

// 复位速度
#define YAW_RESET_SPEED               -4000   // Yaw复位速度
#define PITCH_RESET_SPEED             -6000   // Pitch复位速度
#define RESET_JAM_SPEED_DIFF          250     // 复位堵转速度差

// PID输出限幅
#define MOTOR_OUTPUT_MAX              5000    // 电机输出最大值
#define MOTOR_OUTPUT_MIN              -5000   // 电机输出最小值

// ==================== 类型定义 ====================
typedef struct {
    int         yaw_offset;      // Yaw偏移
    float       pitch_offset;    // Pitch偏移
    uint8_t     index;          // 镖编号
} DartParams_t;

// ==================== 状态枚举 ====================
typedef enum {
    SYSTEM_STATE_UNINIT = 0,
    SYSTEM_STATE_INIT,
    SYSTEM_STATE_STANDBY,
    SYSTEM_STATE_AUTO_AIM,
    SYSTEM_STATE_MANUAL,
    SYSTEM_STATE_ERROR
} SystemState_t;

typedef enum {
    DART_STATE_READY = 0,
    DART_STATE_LOADING,
    DART_STATE_POWERING,
    DART_STATE_LAUNCHING,
    DART_STATE_COMPLETE,
    DART_STATE_ERROR
} DartState_t;

// ==================== 统一数据结构 ====================
typedef struct {
    // ------------------- 系统状态 -------------------
    SystemState_t        system_state;       // 系统状态
    DartState_t          dart_state;         // 飞镖状态
    uint8_t              online_status;      // 在线状态
    uint8_t              error_code;         // 错误码
    
    // ------------------- 云台控制 -------------------
    struct {
        float             target_yaw;        // Yaw目标角度
        float             target_pitch;      // Pitch目标角度
        float             current_yaw;       // Yaw当前角度
        float             current_pitch;     // Pitch当前角度
        uint8_t           yaw_reset_done;    // Yaw复位完成标志
        uint8_t           pitch_reset_done;  // Pitch复位完成标志
        float             yaw_offset;        // Yaw偏移
        float             pitch_offset;      // Pitch偏移
        uint8_t           yaw_stop_flag;     // Yaw停止标志 (对应flag_yaw_stop)
        uint8_t           pitch_stop_flag;   // Pitch停止标志 (对应flag_pitch_stop)
    } gimbal;
    
    // ------------------- 飞镖参数 -------------------
    struct {
        uint8_t           current_index;      // 当前镖号 (0-2)
        uint8_t           loaded_count;       // 已装载数量
        uint8_t           launch_count;       // 已发射数量
        uint8_t           max_count;         // 最大镖数
        float             target_6020;        // 6020目标角度
        float             target_2006_load;   // 2006装载目标角度
    } dart;
    
    // ------------------- 皮筋蓄能 -------------------
    struct {
        uint8_t           state;              // 0=空闲, 1=下拉, 2=蓄能, 3=完成
        uint8_t           complete_flag;     // 完成标志
        int               target_speed;       // 目标速度
    } rubber;
    
    // ------------------- 堵转状态 -------------------
    struct {
        uint8_t           jam_flag;           // 堵转标志 (对应flag_stop)
        uint8_t           upper_jammed;      // 上堵转
        uint8_t           lower_jammed;      // 下堵转
        uint16_t          jam_count;         // 堵转计数
    } jam;
    
    // ------------------- 视觉 --------------------
    struct {
        float             yaw_add;            // Yaw增量 (对应Yaw_add)
        uint8_t           enabled;            // 使能标志 (对应flag_Camera)
        uint16_t          fps;                // 帧率 (对应Camera_Fps)
        uint16_t          frame_count;       // 帧计数 (对应Camera_cnt)
    } vision;
    
    // ------------------- 遥控器 --------------------
    struct {
        uint8_t           last_s1;            // 上次S1状态
        uint8_t           last_s2;            // 上次S2状态
        uint16_t          ch0_hold_cnt;       // 通道0保持计数
        uint16_t          ch1_hold_cnt;       // 通道1保持计数
        uint16_t          ch2_hold_cnt;       // 通道2保持计数
        uint16_t          ch3_hold_cnt;       // 通道3保持计数
        uint8_t           mode;               // 模式
    } remote;
    
    // ------------------- 舵机 --------------------
    struct {
        uint8_t           yellow_angle;       // 黄色舵机角度
        uint8_t           red_state;          // 红色舵机状态
    } servo;
    
    // ------------------- 请求标志 -------------------
    struct {
        uint8_t           reload_request;     // 换弹请求
        uint8_t           rubber_request;     // 蓄能请求
        uint8_t           reset_request;      // 复位请求
        uint8_t           shoot_request;      // 发射请求
    } request;
    
} RobotInfo_t;

// ==================== 外部变量声明 ====================
extern RobotInfo_t            g_robot_info;
extern uint8_t               flag_completely;
extern uint8_t               Reload_mode;
extern uint8_t               Reload_state;
extern RubberState_t         Rubber_state;
extern uint8_t               flag_stop;
extern uint8_t               flag_reset;
extern uint8_t               flag_yaw_stop;
extern uint8_t               flag_pitch_stop;
extern float                 Yaw_add;
extern float                 Yaw_offset;
extern int                   servo_yellow;
extern uint8_t               flag_shoot;
extern uint16_t              Camera_cnt;
extern uint16_t              Camera_Fps;
extern int                   Speed_3508;
extern int                   cnt_up_stop;
extern uint16_t              cnt_complete;
extern uint8_t               shot_complete;
extern float                 Target_Angle_3508;
extern float                 Target_Angle_2006;
extern float                 Target_Angle_2006_yaw;
extern float                 Target_Angle_2006_pitch;
extern float                 Target_Angle_2006_load;
extern float                 Target_6020;
extern float                 a22;
extern float                 a23;

// ==================== 函数声明 ====================

// 系统初始化
void RobotControl_Init(void);

// 主任务调度
void RobotControl_Task(void);

// 子任务函数
void RemoteControl_Task(void);
void Vision_Task(void);
void Gimbal_Task(void);
void DartMechanism_Task(void);
void MotorControl_Task(void);
void Display_Task(void);

// 状态机
void Reload_StateMachine(void);
void Rubber_StateMachine(void);

// 堵转检测
void JamDetection_Task(void);

// 辅助函数
float CalculatePitchAngle(float pitch_offset);
void SetServoYellow(uint8_t angle);
void TriggerReload(uint8_t dart_index);
void TriggerRubber(void);
void TriggerReset(void);

#endif /* ROBOT_CONTROL_H_ */
