/**
 ******************************************************************************
 * @file           : state_machines.h
 * @brief          : 状态机管理头文件
 ******************************************************************************
 * @attention
 * 
 * 包含换弹状态机和蓄能状态机的定义
 * 
 ******************************************************************************
 */

#ifndef STATE_MACHINES_H_
#define STATE_MACHINES_H_

#include <stm32h7xx_hal.h>
#include "control_logic.h"

// ==================== 状态机类型定义 ====================
typedef struct {
    ReloadState_t     state;
    ReloadState_t     prev_state;
    uint32_t          state_enter_time;
    uint32_t          error_count;
    uint16_t          cnt_6020_move;
    uint16_t          cnt_6020_back;
    uint16_t          cnt_2006_down;
    uint16_t          cnt_2006_up;
    uint16_t          cnt_error;
    uint16_t          cnt_servo;
} ReloadStateMachine_t;

typedef struct {
    RubberState_t     state;
    RubberState_t     prev_state;
    uint32_t          state_enter_time;
    uint16_t          cnt_servo;
} RubberStateMachine_t;

// ==================== 状态机变量声明 ====================
extern ReloadStateMachine_t s_reload_sm;
extern RubberStateMachine_t s_rubber_sm;

// ==================== 状态机宏定义 ====================
#define STATE_TRANSITION(new_state) do { \
    s_reload_sm.prev_state = s_reload_sm.state; \
    s_reload_sm.state = new_state; \
    s_reload_sm.state_enter_time = tim14.ClockTime; \
    Reload_OnExit(s_reload_sm.prev_state); \
    Reload_OnEnter(new_state); \
} while(0)

#define RUBBER_STATE_TRANSITION(new_state) do { \
    s_rubber_sm.prev_state = s_rubber_sm.state; \
    s_rubber_sm.state = new_state; \
    s_rubber_sm.state_enter_time = tim14.ClockTime; \
    Rubber_OnExit(s_rubber_sm.prev_state); \
    Rubber_OnEnter(new_state); \
} while(0)

#define STATE_DURATION() (tim14.ClockTime - s_reload_sm.state_enter_time)
#define RUBBER_STATE_DURATION() (tim14.ClockTime - s_rubber_sm.state_enter_time)

// ==================== 函数声明 ====================
void Reload_change(void);
void Rubber_change(void);
void Reload_StateMachine(void);
void Rubber_StateMachine(void);
void Reload_OnEnter(ReloadState_t state);
void Reload_OnExit(ReloadState_t state);
void Rubber_OnEnter(RubberState_t state);
void Rubber_OnExit(RubberState_t state);

#endif /* STATE_MACHINES_H_ */
