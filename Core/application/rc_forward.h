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

/* 固定长度二进制帧长度，便于上位机解析（字节） */
#define RC_FORWARD_FRAME_LEN 24u

/* 帧头和版本 */
#define RC_FORWARD_MAGIC     0xA55Au
#define RC_FORWARD_VERSION   0x01u

/* 固定字段定义：左右摇杆 + 左右拨杆 + 辅助字段 + CRC16 尾校验 */
typedef struct __attribute__((packed))
{
    uint16_t magic;       /* 0xA55A */
    uint8_t  version;     /* 协议版本 */
    uint8_t  payload_len; /* 固定长度(当前值=RC_FORWARD_FRAME_LEN) */
    uint16_t seq;         /* 递增序号，防止重放/丢帧检测 */
    uint32_t tick_ms;     /* HAL tick 时间戳 */

    int16_t  lx;          /* 左摇杆 X（-660..660） */
    int16_t  ly;          /* 左摇杆 Y（-660..660） */
    int16_t  rx;          /* 右摇杆 X（-660..660） */
    int16_t  ry;          /* 右摇杆 Y（-660..660） */

    uint8_t  sw_left;     /* 左拨杆：RC_SW_UP/MID/DOWN（1/2/3） */
    uint8_t  sw_right;    /* 右拨杆：RC_SW_UP/MID/DOWN（1/2/3） */
    uint16_t reserved;     /* 预留字 */

    uint16_t crc16;       /* CRC16/Modbus over [0..RC_FORWARD_FRAME_LEN-3] */
} rc_forward_frame_t;

/* 初始化转发链路（例如 USB CDC 或 USART1 兜底通道） */
void rc_forward_init(void);

/* 每次主循环调用。内部会检测 DBUS 新帧并尝试发送，失败时保留最新一帧覆盖重发。 */
void rc_forward_poll(void);

/* USB CDC 回调唤起：在 USB 收发完成时由 usbd_cdc_if.c 触发。 */
void rc_forward_notify_usb_tx_complete(void);

/* USB 断开/复位/挂起时清理本地发送忙状态，避免等待已丢失的完成回调。 */
void rc_forward_notify_usb_unavailable(void);

#ifdef __cplusplus
}
#endif

#endif /* RC_FORWARD_H */
