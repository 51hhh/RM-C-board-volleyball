/**
  ******************************************************************************
  * @file    rc_forward.c
  * @brief   遥控器 DBUS 数据转发
  *
  * 默认实现为 USART1 文本输出，便于在未完成 USB CDC 生成前联调。
  * 后续你用 CubeMX 生成 CDC 后可将 rc_forward_send_impl 改为 CDC_Transmit_FS。
  ******************************************************************************
  */
#include "rc_forward.h"
#include "remote_control.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define USE_USB_FORWARD 1

static uint32_t last_forwarded = 0U;

static void rc_forward_send_impl(const char *text, uint16_t len)
{
#if USE_USB_FORWARD
    extern uint8_t CDC_Transmit_FS(uint8_t *buf, uint16_t Len);
    (void)CDC_Transmit_FS((uint8_t *)text, len);
#else
    extern void uart_queue_send(const char *data, uint16_t length);
    uart_queue_send(text, len);
#endif
}

void rc_forward_init(void)
{
    last_forwarded = remote_control_get_frame_count();
}

void rc_forward_poll(void)
{
    uint32_t current = remote_control_get_frame_count();
    if (current == last_forwarded) {
        return;
    }

    const RC_ctrl_t *rc = get_remote_control_point();
    if (rc == NULL) {
        return;
    }

    char payload[128];
    int n = snprintf(payload, sizeof(payload),
                     "RC ch0=%d ch1=%d ch2=%d ch3=%d sw=%u%u m=%u/%u k=0x%04x\r\n",
                     (int)rc->rc.ch[0],
                     (int)rc->rc.ch[1],
                     (int)rc->rc.ch[2],
                     (int)rc->rc.ch[3],
                     (unsigned)rc->rc.s[0], (unsigned)rc->rc.s[1],
                     (unsigned)rc->mouse.press_l, (unsigned)rc->mouse.press_r,
                     rc->key.v);
    if (n < 0) {
        return;
    }
    if ((uint16_t)n > sizeof(payload) - 1) {
        n = sizeof(payload) - 1;
    }

    rc_forward_send_impl(payload, (uint16_t)n);
    last_forwarded = current;
}
