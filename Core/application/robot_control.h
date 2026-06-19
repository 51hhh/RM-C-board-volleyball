/**
  ******************************************************************************
  * @file    robot_control.h
  * @brief   排球机器人应用控制调度入口
  ******************************************************************************
  */
#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void robot_control_init(void);
void robot_control_start(void);
void robot_control_tick(uint32_t tick_ms);
void robot_control_handle_exti(uint16_t gpio_pin);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_CONTROL_H */
