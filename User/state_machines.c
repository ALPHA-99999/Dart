/**
 ******************************************************************************
 * @file           : state_machines.c
 * @brief          : 状态机管理实现文件
 ******************************************************************************
 * @attention
 * 
 * 包含换弹状态机和蓄能状态机的实现
 * 兼容原有用法，同时提供新的状态机接口
 * 
 ******************************************************************************
 */

#include "state_machines.h"
#include "control_logic.h"
#include "hardware_config.h"

// ==================== 保留原有函数（向后兼容） ====================
// 这些函数保留以确保与原有代码的兼容性
// 新代码应使用 robot_control.c 中的 Reload_StateMachine() 和 Rubber_StateMachine()

void Reload_change(void)
{
    // 兼容原有调用 - 转发到新状态机
    Reload_StateMachine();
}

void Rubber_change(void)
{
    // 兼容原有调用 - 转发到新状态机
    Rubber_StateMachine();
}
