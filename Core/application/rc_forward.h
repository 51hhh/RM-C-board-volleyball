/**
  ******************************************************************************
  * @file    rc_forward.h
  * @brief   DBUS 遥控器数据转发接口
  ******************************************************************************
  */
#ifndef RC_FORWARD_H
#define RC_FORWARD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化转发链路（例如 USB CDC 或 USART1 兜底通道） */
void rc_forward_init(void);

/* 每检测到一帧新 DBUS 数据时调用一次，执行一次转发。 */
void rc_forward_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* RC_FORWARD_H */
