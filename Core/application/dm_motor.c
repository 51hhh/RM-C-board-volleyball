/**
  ******************************************************************************
  * @file    dm_motor.c
  * @brief   DM4340 执行机构层实现(从 robot_control.c 下沉)
  *
  *   使能 + 角度/速度环下发逻辑与重构前逐字不变；新增"发球请求"机制：
  *   EXTI 中断只置 volatile 标志，设角度与定时回收均在主循环上下文执行，
  *   消除原先从 ISR 直接写 DM 目标的跨上下文访问。
  ******************************************************************************
  */
#include "dm_motor.h"

#include "CAN_receive.h"   /* CAN_cmd_motor_control / DM4340_Control_Init / _Loop / _Set_Target_Angle */
#include "scheduler.h"     /* schedule_after */

#define DM_MOTOR_COUNT         3U      // DM4340 执行电机数(CAN2)
#define DM_ANGLE_STRIKE        1.1f    // 发球(击出)目标角(rad)
#define DM_ANGLE_RECOVER       0.1f    // 回收目标角(rad)
#define DM_STRIKE_RECOVER_MS   500U    // 击出后回收延时(ms)

static volatile uint8_t s_strike_request = 0; // 发球请求标志(ISR 置位，主循环消费)

// 设置三个 DM4340 的目标角(主循环上下文)
static void dm_motor_set_angle_all(float angle)
{
    for (uint8_t i = 0; i < DM_MOTOR_COUNT; i++) {
        DM4340_Set_Target_Angle(i, angle);
    }
}

// 延时回调：击出 500ms 后回收到 DM_ANGLE_RECOVER(由 sched_tick 在主循环触发)
static void cb_dm_recover(void)
{
    dm_motor_set_angle_all(DM_ANGLE_RECOVER);
}

// 3 路 DM4340 控制初始化
void dm_motor_init(void)
{
    for (uint8_t i = 0; i < DM_MOTOR_COUNT; i++) {
        DM4340_Control_Init(i);
    }
}

// 请求一次发球：ISR 安全，仅置 volatile 标志，真正动作在 dm_motor_update 执行
void dm_motor_request_strike(void)
{
    s_strike_request = 1;
}

// 每控制拍：先处理发球请求(主循环上下文)，再使能 + 角度/速度环下发
void dm_motor_update(void)
{
    if (s_strike_request) {
        s_strike_request = 0;
        dm_motor_set_angle_all(DM_ANGLE_STRIKE);
        schedule_after(DM_STRIKE_RECOVER_MS, cb_dm_recover); // 击出后定时回收
    }

    for (uint8_t i = 0; i < DM_MOTOR_COUNT; i++) {
        CAN_cmd_motor_control((uint16_t)(i + 1U), 0, true);  // 使能
    }

    for (uint8_t i = 0; i < DM_MOTOR_COUNT; i++) {
        DM4340_Control_Loop(i);                              // 角度/速度环 + 电流下发
    }
}
