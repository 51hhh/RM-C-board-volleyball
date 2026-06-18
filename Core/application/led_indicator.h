/**
  ******************************************************************************
  * @file    led_indicator.h
  * @brief   板载 RGB LED 状态指示 (PH12=R, PH11=G, PH10=B, 低电平点亮)
  *
  * 状态:
  *   LED_INIT     蓝色常亮      —— 开机/初始化中
  *   LED_RUNNING  绿色闪烁      —— 正常运行
  *   LED_ERROR    红绿 4 位码   —— 错误, 红=0 绿=1, 高位在前, 每位 100ms,
  *                                 4 位后停顿 500ms 再循环
  *
  * 非阻塞: 由 app_scheduler 每个 1kHz 节拍调用 led_update(g_tick)。
  ******************************************************************************
  */
#ifndef LED_INDICATOR_H
#define LED_INDICATOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LED_INIT = 0,   // 蓝色常亮
    LED_RUNNING,    // 绿色闪烁
    LED_ERROR,      // 红绿 4 位错误码
} led_state_t;

/* 初始化(熄灭所有 LED, 进入 LED_INIT 态) */
void led_init(void);

/* 切换到正常运行(绿色闪烁) */
void led_set_running(void);

/* 切换到初始化(蓝色常亮) */
void led_set_init(void);

/* 进入错误指示, code 取低 4 位(0~15), 红绿交替闪出该码 */
void led_set_error(uint8_t code);

/* 节拍驱动, 在 app_scheduler 中每拍调用, now = g_tick(ms) */
void led_update(uint32_t now);

#ifdef __cplusplus
}
#endif

#endif /* LED_INDICATOR_H */
