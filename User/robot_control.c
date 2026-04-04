/**
 ******************************************************************************
 * @file           : robot_control.c
 * @brief          : 飞镖机器人统一控制系统 - 实现文件
 ******************************************************************************
 * @attention
 * 
 * 优化版本：整合所有状态和任务调度，提高代码可维护性
 * 变量命名统一规范：
 * - 全局变量：g_ 前缀
 * - 静态变量：s_ 前缀
 * - 常量：全大写下划线分隔
 * - 函数：驼峰命名
 * 
 ******************************************************************************
 */

#include "robot_control.h"
#include "control_logic.h"
#include "hardware_config.h"
#include "state_machines.h"
#include "referee.h"
#include "referee_control.h"
#include "display_comm.h"
#include "driver_timer.h"
#include "et08.h"
#include "main.h"

// ==================== 全局变量初始化 ====================
RobotInfo_t g_robot_info;

// ==================== 飞镖参数表 ====================
static const DartParams_t s_dart_params[DART_MAX_COUNT] = {
    {DART_6020_ANGLE_DART1, -215, 29.595f},  // 镖1
    {DART_6020_ANGLE_DART2, -232, 29.722f},  // 镖2
    {DART_6020_ANGLE_DART3, -198, 29.585f},  // 镖3
};

// ==================== 状态机变量定义 ====================
ReloadStateMachine_t s_reload_sm = {
    .state = RELOAD_IDLE,
    .prev_state = RELOAD_IDLE,
    .state_enter_time = 0,
    .error_count = 0
};

RubberStateMachine_t s_rubber_sm = {
    .state = Rubber_IDLE,
    .prev_state = Rubber_IDLE,
    .state_enter_time = 0
};

// ==================== 辅助变量 ====================
static uint16_t s_cnt_reset = 0;
static uint16_t s_cnt_yaw_stop = 0;
static uint16_t s_cnt_pitch_stop = 0;
static uint16_t s_cnt_down_stop = 0;
static uint16_t s_cnt_shoot = 0;
static float s_target_angle_3508 = 0;
static float s_target_angle_2006 = 0;
static float s_target_angle_2006_yaw = 0;
static float s_target_angle_2006_pitch = 0;
static float s_target_angle_2006_load = 0;
static float s_target_6020 = -270;

// ==================== 内部函数声明 ====================
static void Reload_OnEnter(ReloadState_t state);
static void Reload_OnExit(ReloadState_t state);
static void Rubber_OnEnter(RubberState_t state);
static void Rubber_OnExit(RubberState_t state);
static void UpdateVisionFPS(void);
static float CalculateYawFromMotor(float motor_angle, float base_angle);
static float CalculatePitchFromMotor(float motor_angle, float base_angle);
static void MotorSpeedControl3508(void);
static void MotorAngleControl2006Load(void);
static void MotorAngleControl6020(void);
static void MotorAngleControl2006(void);

// ==================== 系统初始化 ====================
void RobotControl_Init(void)
{
    // 初始化状态
    g_robot_info.system_state = SYSTEM_STATE_INIT;
    g_robot_info.online_status = 0;
    g_robot_info.error_code = 0;
    
    // 初始化云台
    g_robot_info.gimbal.yaw_reset_done = 0;
    g_robot_info.gimbal.pitch_reset_done = 0;
    g_robot_info.gimbal.target_yaw = 0;
    g_robot_info.gimbal.target_pitch = 0;
    g_robot_info.gimbal.yaw_offset = 0;
    g_robot_info.gimbal.pitch_offset = 0;
    g_robot_info.gimbal.yaw_stop_flag = 0;
    g_robot_info.gimbal.pitch_stop_flag = 0;
    
    // 初始化飞镖参数
    g_robot_info.dart.current_index = 0;
    g_robot_info.dart.loaded_count = 0;
    g_robot_info.dart.launch_count = 0;
    g_robot_info.dart.max_count = DART_MAX_COUNT;
    s_target_6020 = -270;
    g_robot_info.dart.target_6020 = s_target_6020;
    s_target_angle_2006_load = 0;
    g_robot_info.dart.target_2006_load = s_target_angle_2006_load;
    
    // 初始化蓄能
    g_robot_info.rubber.state = 0;
    g_robot_info.rubber.complete_flag = 0;
    g_robot_info.rubber.target_speed = 0;
    
    // 初始化堵转
    g_robot_info.jam.jam_flag = 0;
    g_robot_info.jam.upper_jammed = 0;
    g_robot_info.jam.lower_jammed = 0;
    g_robot_info.jam.jam_count = 0;
    
    // 初始化舵机
    g_robot_info.servo.yellow_angle = SERVO_YELLOW_RETRACT;
    g_robot_info.servo.red_state = SERVO_RED_MIN;
    
    // 初始化视觉
    g_robot_info.vision.enabled = 1;
    g_robot_info.vision.fps = 0;
    g_robot_info.vision.frame_count = 0;
    g_robot_info.vision.yaw_add = 0;
    
    // 初始化请求
    g_robot_info.request.reload_request = 0;
    g_robot_info.request.rubber_request = 0;
    g_robot_info.request.reset_request = 0;
    g_robot_info.request.shoot_request = 0;
}

// ==================== 主任务调度 ====================
void RobotControl_Task(void)
{
    // 更新系统时间
    tim14.ClockTime++;
    
    // 1. 在线状态检测
    RobotOnlineState(&check_robot_state, &referee2022, &rc_Ctrl_et);
    g_robot_info.online_status = rc_Ctrl_et.isOnline;
    
    // 2. 更新视觉帧率
    UpdateVisionFPS();
    
    // 3. 堵转检测
    JamDetection_Task();
    
    // 4. 遥控器处理
    RemoteControl_Task();
    
    // 5. 视觉处理
    Vision_Task();
    
    // 6. 复位逻辑
    if(g_robot_info.request.reset_request) {
        Yaw_Pitch_Reset();
    }
    
    // 7. 舵机控制
    servo_control();
    
    // 8. 状态机更新
    Reload_StateMachine();
    Rubber_StateMachine();
    
    // 9. 电机控制
    MotorControl_Task();
    
    // 10. 屏幕显示
    Display_Task();
    
    // 11. CAN输出
    MotorCanOutput(can2, 0x1FF);
    MotorCanOutput(can1, 0x1FF);
    MotorCanOutput(can2, 0x200);
    
    // 12. 裁判系统
    Referee_Judge();
}

// ==================== 遥控器任务 ====================
void RemoteControl_Task(void)
{
    RC_Ctrl* rc = &rc_Ctrl_et;
    
    // 电机目标角度初始化
    s_target_angle_2006 = motor2006.motor[0].Data.TotalAngle;
    g_robot_info.gimbal.yaw_offset = infor[0].Yaw_offset;
    
    // 通道3控制
    if(rc->isOnline == 1 && g_robot_info.remote.mode == 0 && rc->rc.s2 != 2) {
        s_target_angle_2006 += (rc->rc.ch3 - 1024) * 0.05f;
    }
    
    // S1拨杆 - 3508速度控制
    if(rc->rc.s1 == 3) {
        g_robot_info.rubber.target_speed = 0;
    } else if(rc->rc.s1 == 2) {
        g_robot_info.rubber.target_speed = -5000;
    } else if(rc->rc.s1 == 1) {
        g_robot_info.rubber.target_speed = 8000;
    }
    
    // S2拨杆模式切换 - 自动模式
    if(rc->rc.s2 == 1 && rc->rc.s2_last == 3 && referee2022.game_status.game_progress == 4) {
        g_robot_info.remote.mode++;
        if(g_robot_info.remote.mode == 1) {
            s_target_angle_2006_pitch = CalculatePitchAngle(infor[0].Pitch_offset) + a23;
            if(g_robot_info.rubber.state == 0) {
                TriggerRubber();
            }
        } else if(g_robot_info.remote.mode == 2) {
            if(g_robot_info.rubber.state == 0 || g_robot_info.rubber.state == 3) {
                TriggerRubber();
            }
            g_robot_info.request.reload_request = 1;
            SetServoYellow(SERVO_YELLOW_EXTEND);
        }
    }
    
    // S2拨杆 - 手动模式角度控制
    if(rc->isOnline == 1 && rc->rc.s2 == 2) {
        s_target_angle_2006_yaw -= (rc->rc.ch2 - 1024) * 0.05f;
        s_target_angle_2006_pitch -= (rc->rc.ch3 - 1024) * 0.05f;
    }
    
    // 消堵转
    if(rc->rc.s2 == 1 && rc->rc.s2_last == 3 && 
       g_robot_info.jam.jam_flag == 1 && referee2022.game_status.game_progress != 4) {
        g_robot_info.jam.jam_flag = 0;
    }
    
    // 检测通道保持
    if(abs(rc->rc.ch0 - 1024) > RC_STICK_DEADZONE && rc->rc.s2 != 2) {
        g_robot_info.remote.ch0_hold_cnt++;
    } else {
        g_robot_info.remote.ch0_hold_cnt = 0;
    }
    
    if(abs(rc->rc.ch1 - 1024) > RC_STICK_DEADZONE && rc->rc.s2 == 2) {
        g_robot_info.remote.ch1_hold_cnt++;
    } else {
        g_robot_info.remote.ch1_hold_cnt = 0;
    }
    
    // Yaw调整
    if(g_robot_info.remote.ch0_hold_cnt >= RC_HOLD_TRIGGER_COUNT) {
        g_robot_info.remote.ch0_hold_cnt = 0;
        if(rc->rc.ch0 > 1024) {
            s_target_angle_2006_yaw += YAW_ADJUST_STEP;
        } else {
            s_target_angle_2006_yaw -= YAW_ADJUST_STEP;
        }
    }
    
    // Pitch调整
    if(g_robot_info.remote.ch1_hold_cnt >= RC_HOLD_TRIGGER_COUNT && rc->rc.s2 == 2) {
        g_robot_info.remote.ch1_hold_cnt = 0;
        if(rc->rc.ch1 > 1024) {
            s_target_angle_2006_pitch -= PITCH_ADJUST_STEP;
        } else {
            s_target_angle_2006_pitch += PITCH_ADJUST_STEP;
        }
    }
    
    // 复位检测
    if(rc->rc.s2 == 2) {
        s_cnt_reset++;
    } else {
        s_cnt_reset = 0;
    }
    if(s_cnt_reset > RESET_HOLD_TIME_MS) {
        s_cnt_reset = 0;
        g_robot_info.request.reset_request = 1;
    }
    
    // 发射触发
    if(abs(rc->rc.ch0 - 1024) > RC_STICK_DEADZONE && 
       rc->rc.s2 != 2 && referee2022.game_status.game_progress != 4) {
        s_cnt_shoot++;
    } else {
        s_cnt_shoot = 0;
    }
    if(s_cnt_shoot >= RC_SHOOT_TRIGGER_COUNT) {
        s_cnt_shoot = 0;
        if(g_robot_info.servo.red_state == SERVO_RED_MIN) {
            g_robot_info.servo.red_state = SERVO_RED_MAX;
        } else {
            g_robot_info.servo.red_state = SERVO_RED_MIN;
        }
    }
    
    // 保存上次的拨杆状态
    rc->rc.s2_last = rc->rc.s2;
    rc->rc.s1_last = rc->rc.s1;
    
    // 在线检测
    if(rc->isOnline != 1) {
        ET08Init(&rc_Ctrl_et);
        g_robot_info.jam.jam_flag = 0;
    }
}

// ==================== 视觉任务 ====================
void Vision_Task(void)
{
    // 视觉Yaw调整
    if(g_robot_info.vision.yaw_add != 0 && g_robot_info.vision.enabled == 0) {
        if(tim14.ClockTime % 100 == 0 && g_robot_info.request.reset_request == 1 && 
           g_robot_info.gimbal.yaw_stop_flag == 1 && rc_Ctrl_et.rc.s2 == 1) {
            AdjustYawWithVision();
        }
    } else if(g_robot_info.vision.yaw_add != 0 && g_robot_info.vision.enabled == 1) {
        if(tim14.ClockTime % 100 == 0) {
            AdjustYawWithVision();
        }
    }
}

static void AdjustYawWithVision(void)
{
    float yaw_add = g_robot_info.vision.yaw_add;
    float yaw_offset = g_robot_info.gimbal.yaw_offset;
    
    if(yaw_add > (5 + yaw_offset)) {
        s_target_angle_2006_yaw += 2000;
    } else if(yaw_add < (-5 + yaw_offset)) {
        s_target_angle_2006_yaw -= 2000;
    } else if(yaw_add > (-5 + yaw_offset) && yaw_add < (-2 + yaw_offset)) {
        s_target_angle_2006_yaw -= 500;
    } else if(yaw_add < (5 + yaw_offset) && yaw_add > (2 + yaw_offset)) {
        s_target_angle_2006_yaw += 500;
    }
}

// ==================== 云台任务 ====================
void Gimbal_Task(void)
{
    if(g_robot_info.request.reset_request) {
        // 复位模式 - 使用速度控制
        motor2006.motor[1].Data.Target = YAW_RESET_SPEED;
        motor2006.motor[2].Data.Target = PITCH_RESET_SPEED;
        
        // Yaw控制
        if(g_robot_info.gimbal.yaw_stop_flag == 0) {
            motor2006.motor[1].Data.Output = 
                BasePID_SpeedControl(&run_pid, motor2006.motor[1].Data.Target, 
                                    motor2006.motor[1].Data.SpeedRPM);
        } else {
            motor2006.motor[1].Data.Output = 
                BasePID_PitchSpeedControl((BasePID_Object*)&Motors2006_yaw_SpeedPID,
                    BasePID_YawAngleControl((BasePID_Object*)&Motors2006_yaw_AngelPID,
                        s_target_angle_2006_yaw, motor2006.motor[1].Data.TotalAngle),
                    motor2006.motor[1].Data.SpeedRPM);
            motor2006.motor[1].Data.Output = LIMIT(motor2006.motor[1].Data.Output, 
                                                    MOTOR_OUTPUT_MAX, MOTOR_OUTPUT_MIN);
        }
        
        // Pitch控制
        if(g_robot_info.gimbal.pitch_stop_flag == 0) {
            motor2006.motor[2].Data.Output = 
                BasePID_SpeedControl(&run_pid, motor2006.motor[2].Data.Target, 
                                    motor2006.motor[2].Data.SpeedRPM);
        } else {
            motor2006.motor[2].Data.Output = 
                BasePID_PitchSpeedControl((BasePID_Object*)&Motors2006_pitch_SpeedPID,
                    BasePID_YawAngleControl((BasePID_Object*)&Motors2006_pitch_AngelPID,
                        s_target_angle_2006_pitch, motor2006.motor[2].Data.TotalAngle),
                    motor2006.motor[2].Data.SpeedRPM);
            motor2006.motor[2].Data.Output = LIMIT(motor2006.motor[2].Data.Output, 
                                                    MOTOR_OUTPUT_MAX, MOTOR_OUTPUT_MIN);
        }
    } else {
        // 正常模式 - 使用角度环
        motor2006.motor[1].Data.Output = 
            BasePID_PitchSpeedControl((BasePID_Object*)&Motors2006_yaw_SpeedPID,
                BasePID_YawAngleControl((BasePID_Object*)&Motors2006_yaw_AngelPID,
                    s_target_angle_2006_yaw, motor2006.motor[1].Data.TotalAngle),
                motor2006.motor[1].Data.SpeedRPM);
        
        motor2006.motor[2].Data.Output = 
            BasePID_PitchSpeedControl((BasePID_Object*)&Motors2006_pitch_SpeedPID,
                BasePID_YawAngleControl((BasePID_Object*)&Motors2006_pitch_AngelPID,
                    s_target_angle_2006_pitch, motor2006.motor[2].Data.TotalAngle),
                motor2006.motor[2].Data.SpeedRPM);
    }
}

// ==================== 堵转检测 ====================
void JamDetection_Task(void)
{
    // 上堵转检测 (微动开关)
    if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == 0) {
        g_robot_info.jam.upper_jammed++;
    } else {
        g_robot_info.jam.upper_jammed = 0;
    }
    if(g_robot_info.jam.upper_jammed > STOP_SWITCH_COUNT) {
        g_robot_info.jam.upper_jammed = 0;
        g_robot_info.jam.jam_flag = 1;
        s_target_angle_3508 = motor3508.motor[0].Data.TotalAngle - 200;
        motor3508.motor[0].Data.Target = 0;
    }
    
    // 下堵转检测 (速度差)
    if(abs(motor3508.motor[0].Data.Target - motor3508.motor[0].Data.SpeedRPM) > JAM_SPEED_THRESHOLD && 
       g_robot_info.jam.jam_flag == 0) {
        g_robot_info.jam.jam_count++;
    } else {
        g_robot_info.jam.jam_count = 0;
    }
    if(g_robot_info.jam.jam_count >= JAM_DETECT_COUNT) {
        g_robot_info.jam.jam_count = 0;
        s_target_angle_3508 = motor3508.motor[0].Data.TotalAngle + 100;
        g_robot_info.jam.jam_flag = 1;
        motor3508.motor[0].Data.Target = 0;
    }
}

// ==================== 电机控制 ====================
void MotorControl_Task(void)
{
    // 3508电机控制
    if(g_robot_info.jam.jam_flag == 0) {
        motor3508.motor[0].Data.Target = g_robot_info.rubber.target_speed;
        motor3508.motor[0].Data.Output = 
            BasePID_SpeedControl(&run_pid, motor3508.motor[0].Data.Target, 
                               motor3508.motor[0].Data.SpeedRPM);
    } else {
        motor3508.motor[0].Data.Target = 0;
        motor3508.motor[0].Data.Output = 
            BasePID_PitchSpeedControl((BasePID_Object*)&Motors3508_SpeedPID,
                BasePID_PitchAngleControl((BasePID_Object*)&Motors3508_AngelPID,
                    s_target_angle_3508, motor3508.motor[0].Data.TotalAngle),
                motor3508.motor[0].Data.SpeedRPM);
    }
    
    // 2006装载电机
    MotorAngleControl2006Load();
    
    // 6020电机
    MotorAngleControl6020();
    
    // 2006主电机
    MotorAngleControl2006();
    
    // CAN输出
    MotorCanOutput_Task();
    
    // 在线检测 - 离线时清零输出
    if(rc_Ctrl_et.isOnline == 0) {
        motor3508.motor[0].Data.Output = 0;
        motor2006.motor[0].Data.Output = 0;
        motor2006.motor[1].Data.Output = 0;
        motor2006.motor[2].Data.Output = 0;
    }
}

static void MotorSpeedControl3508(void)
{
    if(g_robot_info.jam.jam_flag == 0) {
        motor3508.motor[0].Data.Target = g_robot_info.rubber.target_speed;
        motor3508.motor[0].Data.Output = 
            BasePID_SpeedControl(&run_pid, motor3508.motor[0].Data.Target, 
                               motor3508.motor[0].Data.SpeedRPM);
    }
}

static void MotorAngleControl2006Load(void)
{
    motor2006.motor[3].Data.Output = 
        BasePID_PitchSpeedControl((BasePID_Object*)&Motors2006_load_SpeedPID,
            BasePID_YawAngleControl((BasePID_Object*)&Motors2006_load_AngelPID,
                g_robot_info.dart.target_2006_load, motor2006.motor[3].Data.TotalAngle),
            motor2006.motor[3].Data.SpeedRPM);
}

static void MotorAngleControl6020(void)
{
    motor6020.motor[0].Data.Output = 
        BasePID_PitchSpeedControl((BasePID_Object*)&Motors6020_pitch_SpeedPID,
            BasePID_YawAngleControl((BasePID_Object*)&Motors6020_pitch_AngelPID,
                g_robot_info.dart.target_6020, motor6020.motor[0].Data.TotalAngle),
            motor6020.motor[0].Data.SpeedRPM);
}

static void MotorAngleControl2006(void)
{
    motor2006.motor[0].Data.Output = 
        BasePID_YawSpeedControl((BasePID_Object*)&Motors2006_SpeedPID,
            BasePID_YawAngleControl((BasePID_Object*)&Motors2006_AngelPID,
                s_target_angle_2006, motor2006.motor[0].Data.TotalAngle),
            motor2006.motor[0].Data.SpeedRPM);
}

static void MotorCanOutput_Task(void)
{
    if(Reload_state != RELOAD_ERROR) {
        MotorFillData(&motor3508.motor[0], motor3508.motor[0].Data.Output);
        MotorFillData(&motor2006.motor[0], motor2006.motor[0].Data.Output);
        MotorFillData(&motor2006.motor[1], motor2006.motor[1].Data.Output);
        MotorFillData(&motor2006.motor[2], motor2006.motor[2].Data.Output);
        MotorFillData(&motor6020.motor[0], motor6020.motor[0].Data.Output);
        MotorFillData(&motor2006.motor[3], motor2006.motor[3].Data.Output);
    } else {
        MotorFillData(&motor3508.motor[0], 0);
        MotorFillData(&motor2006.motor[0], 0);
        MotorFillData(&motor2006.motor[1], 0);
        MotorFillData(&motor2006.motor[2], 0);
        MotorFillData(&motor6020.motor[0], 0);
        MotorFillData(&motor2006.motor[3], 0);
    }
}

// ==================== 显示任务 ====================
void Display_Task(void)
{
    Send_toled();
    
    if(tim14.ClockTime % 30 == 0) {
        servo_move(0xFE, 100, g_robot_info.servo.yellow_angle);
    }
    
    // 计算并更新当前角度
    g_robot_info.gimbal.current_yaw = CalculateYawFromMotor(
        motor2006.motor[1].Data.TotalAngle, a22);
    g_robot_info.gimbal.current_pitch = CalculatePitchFromMotor(
        motor2006.motor[2].Data.TotalAngle, a23);
}

// ==================== 换弹状态机 ====================
void Reload_StateMachine(void)
{
    uint32_t duration = tim14.ClockTime - s_reload_sm.state_enter_time;
    
    switch(s_reload_sm.state) {
        case RELOAD_IDLE:
            if(g_robot_info.request.reload_request) {
                g_robot_info.request.reload_request = 0;
                Reload_mode = g_robot_info.dart.current_index + 1;
                STATE_TRANSITION(RELOAD_STEP1);
            }
            break;
            
        case RELOAD_STEP1:
            SetDartParameters(Reload_mode);
            STATE_TRANSITION(RELOAD_STEP1_5);
            break;
            
        case RELOAD_STEP1_5:
            if(Check6020Position()) {
                STATE_TRANSITION(RELOAD_STEP2);
            }
            if(duration > RELOAD_TIMEOUT_MS) {
                s_reload_sm.error_count++;
                if(s_reload_sm.error_count > 3) {
                    STATE_TRANSITION(RELOAD_ERROR);
                }
            }
            break;
            
        case RELOAD_STEP2:
            if(Check2006Jam()) {
                STATE_TRANSITION(RELOAD_ERROR);
            }
            if(Check2006Down()) {
                STATE_TRANSITION(RELOAD_STEP3);
            }
            break;
            
        case RELOAD_STEP3:
            Check2006Up();
            if(CheckServoPosition()) {
                SetServoYellow(SERVO_YELLOW_RETRACT);
            }
            if(Check2006UpComplete()) {
                STATE_TRANSITION(RELOAD_STEP4);
            }
            break;
            
        case RELOAD_STEP4:
            if(Check6020Reset()) {
                STATE_TRANSITION(RELOAD_COMPLETE);
            }
            break;
            
        case RELOAD_COMPLETE:
            g_robot_info.dart.loaded_count++;
            g_robot_info.dart.current_index = 
                (g_robot_info.dart.current_index + 1) % g_robot_info.dart.max_count;
            STATE_TRANSITION(RELOAD_IDLE);
            break;
            
        case RELOAD_ERROR:
            motor2006.motor[3].Data.Output = 0;
            motor6020.motor[0].Data.Output = 0;
            SetServoYellow(SERVO_YELLOW_EXTEND);
            break;
    }
    
    Reload_state = s_reload_sm.state;
}

static void SetDartParameters(uint8_t dart_num)
{
    if(dart_num == 1) {
        g_robot_info.dart.target_6020 += DART_6020_ANGLE_DART1;
        g_robot_info.gimbal.yaw_offset = infor[1].Yaw_offset;
        s_target_angle_2006_pitch = CalculatePitchAngle(infor[1].Pitch_offset) + a23;
    } else if(dart_num == 2) {
        g_robot_info.dart.target_6020 += DART_6020_ANGLE_DART2;
        g_robot_info.gimbal.yaw_offset = infor[2].Yaw_offset;
        s_target_angle_2006_pitch = CalculatePitchAngle(infor[2].Pitch_offset) + a23;
    } else if(dart_num == 3) {
        g_robot_info.dart.target_6020 += DART_6020_ANGLE_DART3;
        g_robot_info.gimbal.yaw_offset = infor[3].Yaw_offset;
        s_target_angle_2006_pitch = CalculatePitchAngle(infor[3].Pitch_offset) + a23;
    }
}

static uint8_t Check6020Position(void)
{
    if(abs(g_robot_info.dart.target_6020 - motor6020.motor[0].Data.TotalAngle) < 1.5f) {
        s_reload_sm.cnt_6020_move++;
    } else {
        s_reload_sm.cnt_6020_move = 0;
    }
    return (s_reload_sm.cnt_6020_move > 50);
}

static uint8_t Check2006Jam(void)
{
    if(abs(motor2006.motor[3].Data.SpeedRPM - Motors2006_load_AngelPID.Out) > JAM_ERROR_THRESHOLD) {
        s_reload_sm.cnt_error++;
    } else {
        s_reload_sm.cnt_error = 0;
    }
    return (s_reload_sm.cnt_error > JAM_ERROR_COUNT);
}

static uint8_t Check2006Down(void)
{
    if(abs(g_robot_info.dart.target_2006_load - motor2006.motor[3].Data.TotalAngle) < 200) {
        s_reload_sm.cnt_2006_down++;
    } else {
        s_reload_sm.cnt_2006_down = 0;
    }
    if(s_reload_sm.cnt_2006_down > 100) {
        s_reload_sm.cnt_2006_down = 0;
        g_robot_info.dart.target_2006_load += DART_LOAD_ANGLE;
        return 1;
    }
    return 0;
}

static void Check2006Up(void)
{
    if(abs(g_robot_info.dart.target_2006_load - motor2006.motor[3].Data.TotalAngle) < 9000) {
        s_reload_sm.cnt_2006_up++;
    } else {
        s_reload_sm.cnt_2006_up = 0;
    }
}

static uint8_t CheckServoPosition(void)
{
    if(abs(g_robot_info.dart.target_2006_load - motor2006.motor[3].Data.TotalAngle) < 45000) {
        s_reload_sm.cnt_servo++;
    } else {
        s_reload_sm.cnt_servo = 0;
    }
    return (s_reload_sm.cnt_servo > 50);
}

static uint8_t Check2006UpComplete(void)
{
    return (s_reload_sm.cnt_2006_up > 100);
}

static uint8_t Check6020Reset(void)
{
    if(abs(g_robot_info.dart.target_6020 - motor6020.motor[0].Data.TotalAngle) < 1.0f) {
        s_reload_sm.cnt_6020_back++;
    } else {
        s_reload_sm.cnt_6020_back = 0;
    }
    if(s_reload_sm.cnt_6020_back > 100) {
        s_reload_sm.cnt_6020_back = 0;
        return 1;
    }
    return 0;
}

void Reload_OnEnter(ReloadState_t state)
{
    (void)state;
}

void Reload_OnExit(ReloadState_t state)
{
    if(state == RELOAD_STEP1_5 || state == RELOAD_STEP2 || 
       state == RELOAD_STEP3 || state == RELOAD_STEP4) {
        s_reload_sm.error_count = 0;
    }
}

// ==================== 蓄能状态机 ====================
void Rubber_StateMachine(void)
{
    uint32_t duration = tim14.ClockTime - s_rubber_sm.state_enter_time;
    
    switch(s_rubber_sm.state) {
        case Rubber_IDLE:
            if(g_robot_info.request.rubber_request) {
                g_robot_info.request.rubber_request = 0;
                STATE_TRANSITION(Rubber_STEP1);
            }
            break;
            
        case Rubber_STEP1:
            g_robot_info.rubber.target_speed = -5000;
            if(g_robot_info.jam.jam_flag == 1) {
                STATE_TRANSITION(Rubber_STEP2);
            }
            if(duration > RUBBER_TIMEOUT_MS) {
                STATE_TRANSITION(Rubber_ERROR);
            }
            break;
            
        case Rubber_STEP2:
            g_robot_info.servo.red_state = SERVO_RED_MAX;
            if(g_robot_info.request.reload_request) {
                Reload_mode++;
                if(Reload_state == RELOAD_COMPLETE) {
                    Reload_state = RELOAD_STEP1;
                    g_robot_info.request.reload_request = 1;
                }
                g_robot_info.request.reload_request = 0;
            }
            s_rubber_sm.cnt_servo++;
            if(s_rubber_sm.cnt_servo > SERVO_ACTION_TIMEOUT) {
                s_rubber_sm.cnt_servo = 0;
                g_robot_info.jam.jam_flag = 0;
                STATE_TRANSITION(Rubber_STEP3);
            }
            break;
            
        case Rubber_STEP3:
            if(Reload_mode == 0 || Reload_state == RELOAD_COMPLETE) {
                g_robot_info.rubber.target_speed = 6000;
                if(g_robot_info.jam.jam_flag == 1) {
                    STATE_TRANSITION(Rubber_COMPLETE);
                }
            }
            if(duration > RUBBER_TIMEOUT_MS * 2) {
                STATE_TRANSITION(Rubber_ERROR);
            }
            break;
            
        case Rubber_COMPLETE:
            g_robot_info.rubber.complete_flag = 1;
            g_robot_info.rubber.state = 3;
            break;
            
        case Rubber_ERROR:
            g_robot_info.rubber.state = 4;
            break;
    }
    
    Rubber_state = s_rubber_sm.state;
}

void Rubber_OnEnter(RubberState_t state)
{
    s_rubber_sm.cnt_servo = 0;
    (void)state;
}

void Rubber_OnExit(RubberState_t state)
{
    (void)state;
}

// ==================== 辅助函数 ====================
void UpdateVisionFPS(void)
{
    static uint16_t s_last_camera_cnt = 0;
    
    if(tim14.ClockTime % 1000 == 0) {
        g_robot_info.vision.fps = g_robot_info.vision.frame_count - s_last_camera_cnt;
        s_last_camera_cnt = g_robot_info.vision.frame_count;
    }
}

float CalculateYawFromMotor(float motor_angle, float base_angle)
{
    float yaw_l = 209 - (fabs(motor_angle - base_angle) / 533234.0f * 164.0f);
    
    if(yaw_l < 88) {
        return -atan((88 - yaw_l) / 663.1467f) * 57.3f;
    } else {
        return atan((yaw_l - 88) / 663.1467f) * 57.3f;
    }
}

float CalculatePitchFromMotor(float motor_angle, float base_angle)
{
    float temp = 593.5f - (motor_angle - base_angle) / 413228.0f * 128.53f;
    float a = 543.13405f;
    float b = 280.71f;
    
    return acos((temp * temp + a * a - b * b) / (2 * temp * a)) * 57.29578f;
}

float CalculatePitchAngle(float pitch_offset)
{
    const float A = 543.13405f;
    const float B = 280.71f;
    
    float pitch_rad = pitch_offset / 57.29578f;
    float cos_val = cosf(pitch_rad);
    float discriminant = A * A * cos_val * cos_val - (A * A - B * B);
    float pitch_l = A * cos_val + sqrtf(discriminant);
    float motor_angle = (593.5f - pitch_l) * 413228.0f / 128.53f;
    
    return motor_angle;
}

void SetServoYellow(uint8_t angle)
{
    g_robot_info.servo.yellow_angle = angle;
}

void TriggerReload(uint8_t dart_index)
{
    if(dart_index < g_robot_info.dart.max_count) {
        g_robot_info.dart.current_index = dart_index;
        g_robot_info.request.reload_request = 1;
    }
}

void TriggerRubber(void)
{
    if(g_robot_info.rubber.state == 0 || g_robot_info.rubber.state == 3) {
        g_robot_info.request.rubber_request = 1;
        g_robot_info.jam.jam_flag = 0;
    }
}

void TriggerReset(void)
{
    g_robot_info.request.reset_request = 1;
}

// ==================== 辅助宏 ====================
#ifndef LIMIT
#define LIMIT(x, max, min) (((x) > (max)) ? (max) : (((x) < (min)) ? (min) : (x)))
#endif

#ifndef abs
#define abs(x) ((x) > 0 ? (x) : (-(x)))
#endif
