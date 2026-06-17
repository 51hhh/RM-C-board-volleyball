/**
  ******************************************************************************
  * @file    scheduler.h
  * @brief   裸机时序架构：1kHz 时基(TIM6) + DWT 微秒计时 + 非阻塞延时事件调度
  *
  * 设计目标：
  *   - 控制环固定频率（不受后台任务抖动影响）
  *   - 多频率任务用"基准节拍的不同分频"调用（见 main.c app_scheduler）
  *   - 稳定的延时一次性触发（如电磁铁释放后延时击球），全程非阻塞
  *
  * 铁律：主循环及所有回调禁止使用 HAL_Delay 等阻塞调用。
  ******************************************************************************
  */
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 1kHz 系统节拍（由 TIM6 更新中断累加），单位 ms */
extern volatile uint32_t g_tick;
/* 每个节拍置 1，主循环消费后清 0 —— 控制环的触发信号 */
extern volatile uint8_t  g_tick_flag;

/* 时基初始化：配置并启动 TIM6@1kHz 更新中断 + 启用 DWT 周期计数器 */
void timebase_init(void);

/* 基于 DWT 的微秒时间戳，用于高精度计时 / 任务耗时测量 */
uint32_t micros(void);

/* ===== 非阻塞一次性延时事件 ===== */
typedef void (*evt_cb_t)(void);

/* delay_ms 毫秒后触发 cb（一次性）。返回 0 成功，-1 事件表已满。
 * 注意：cb 在主循环上下文中执行，必须保持非阻塞。*/
int  schedule_after(uint32_t delay_ms, evt_cb_t cb);

/* 每个节拍调用一次，扫描并触发到期事件 */
void sched_tick(uint32_t now);

#ifdef __cplusplus
}
#endif

#endif /* SCHEDULER_H */
