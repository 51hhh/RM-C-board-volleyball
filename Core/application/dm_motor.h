/**
  ******************************************************************************
  * @file    dm_motor.h
  * @brief   DM4340 执行机构层：3 路 MIT 模式电机(CAN2)的初始化、每拍角度/速度环
  *          下发，以及由外部触发(PB12 下降沿)发起的发球(击出 -> 回收)序列。
  *
  *   发球请求经 dm_motor_request_strike() 投递(ISR 安全)，真正的设角度 + 定时
  *   回收都在 dm_motor_update() 的主循环上下文执行——避免在中断里直接驱动执行机构。
  ******************************************************************************
  */
#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* 3 路 DM4340 控制初始化 */
void dm_motor_init(void);

/* 每控制拍调用：处理待执行的发球请求 + 使能 + 角度/速度环下发 */
void dm_motor_update(void);

/* 请求一次发球(击出后定时回收)。
 * ISR 安全：仅置 volatile 标志，动作延后到 dm_motor_update 在主循环执行。 */
void dm_motor_request_strike(void);

#ifdef __cplusplus
}
#endif

#endif /* DM_MOTOR_H */
