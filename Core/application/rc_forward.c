/**
  *****************************************************************************/
#include "rc_forward.h"
#include "remote_control.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include <string.h>

#ifndef USE_USB_FORWARD
#define USE_USB_FORWARD 1
#endif

/* USB IN 传输超过该时长仍未完成时，允许清 BUSY 重试，避免总线异常导致长时间卡顿。 */
#ifndef RC_FORWARD_USB_TX_BUSY_TIMEOUT_MS
#define RC_FORWARD_USB_TX_BUSY_TIMEOUT_MS 200U
#endif

static uint32_t s_last_remote_frame = 0U;
static uint16_t s_seq = 0U;
static volatile uint8_t s_cdc_busy = 0U;
static volatile uint8_t s_pending = 0U;
static volatile uint32_t s_cdc_busy_start_tick = 0U;
static rc_forward_frame_t s_pending_frame;

_Static_assert(sizeof(rc_forward_frame_t) == RC_FORWARD_FRAME_LEN, "rc_forward frame must be 24 bytes");

#if USE_USB_FORWARD
static void rc_forward_set_cdc_busy(uint8_t busy)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_cdc_busy = busy;
    s_cdc_busy_start_tick = busy ? HAL_GetTick() : 0U;
    __set_PRIMASK(primask);
}

static uint8_t rc_forward_is_cdc_busy(uint32_t now_tick)
{
    uint8_t busy = 0U;
    uint32_t busy_start = 0U;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    busy = s_cdc_busy;
    busy_start = s_cdc_busy_start_tick;
    if ((busy != 0U) && ((now_tick - busy_start) > RC_FORWARD_USB_TX_BUSY_TIMEOUT_MS)) {
        /* 传输栈异常时释放 BUSY，下一拍可重发，防止永远停滞。 */
        s_cdc_busy = 0U;
        s_cdc_busy_start_tick = 0U;
        busy = 0U;
    }
    __set_PRIMASK(primask);

    return busy;
}
#else
static uint8_t rc_forward_is_cdc_busy(uint32_t now_tick)
{
    (void)now_tick;
    return 0U;
}
#endif

#if !USE_USB_FORWARD
static uint8_t rc_forward_send_impl(const rc_forward_frame_t *frame)
{
    if (frame == NULL) {
        return USBD_OK;
    }
    extern void uart_queue_send(const char *data, uint16_t length);
    /* 串口兜底通道：一次性发送整帧 */
    uart_queue_send((const char *)frame, RC_FORWARD_FRAME_LEN);
    return USBD_OK;
}
#else
static uint8_t rc_forward_send_impl(const rc_forward_frame_t *frame)
{
    uint8_t result = USBD_FAIL;

    if (frame == NULL) {
        return USBD_FAIL;
    }
    if (CDC_IsTxReady_FS() == 0U) {
        return USBD_BUSY;
    }
    result = CDC_Transmit_FS((uint8_t *)frame, RC_FORWARD_FRAME_LEN);
    if (result == USBD_OK) {
        /* USB 正在发送，等待完成回调清 BUSY */
        rc_forward_set_cdc_busy(1U);
    }
    return result;
}
#endif

static uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0u; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0u; b < 8u; b++) {
            if (crc & 1u) {
                crc = (uint16_t)((crc >> 1u) ^ 0xA001u);
            } else {
                crc >>= 1u;
            }
        }
    }
    return crc;
}

void rc_forward_notify_usb_tx_complete(void)
{
    /* USB 发送完成回调，允许下一帧补发 */
#if USE_USB_FORWARD
    rc_forward_set_cdc_busy(0U);
#else
    s_cdc_busy = 0U;
#endif
}

void rc_forward_notify_usb_unavailable(void)
{
#if USE_USB_FORWARD
    rc_forward_set_cdc_busy(0U);
#else
    s_cdc_busy = 0U;
#endif
}

void rc_forward_init(void)
{
    /* 以当前 DBUS 帧计数初始化，避免上电重复转发上一拍 */
    s_last_remote_frame = remote_control_get_frame_count();
    s_seq = 0U;
#if USE_USB_FORWARD
    rc_forward_set_cdc_busy(0U);
#else
    s_cdc_busy = 0U;
#endif
    s_pending = 0U;
    memset((void *)&s_pending_frame, 0, sizeof(s_pending_frame));
}

void rc_forward_poll(void)
{
    RC_ctrl_t rc_snapshot;
    uint32_t current_frame = 0U;
    uint8_t send_result = USBD_BUSY;
    const uint32_t now_tick = HAL_GetTick();
    uint8_t cdc_busy = rc_forward_is_cdc_busy(now_tick);

    current_frame = remote_control_copy(&rc_snapshot);
    if (current_frame != s_last_remote_frame) {
        rc_forward_frame_t frame = {0};
        /* 构建定长二进制帧：避开文本解析歧义，便于上位机 CRC 校验 */
        frame.magic = RC_FORWARD_MAGIC;
        frame.version = RC_FORWARD_VERSION;
        frame.payload_len = RC_FORWARD_FRAME_LEN;
        frame.seq = s_seq++;
        frame.tick_ms = now_tick;

        frame.lx = rc_snapshot.rc.ch[2]; /* 左摇杆 X 对应 ch2 */
        frame.ly = rc_snapshot.rc.ch[3]; /* 左摇杆 Y 对应 ch3 */
        frame.rx = rc_snapshot.rc.ch[0]; /* 右摇杆 X 对应 ch0 */
        frame.ry = rc_snapshot.rc.ch[1]; /* 右摇杆 Y 对应 ch1 */
        frame.sw_left = (uint8_t)rc_snapshot.rc.s[1];
        frame.sw_right = (uint8_t)rc_snapshot.rc.s[0];
        frame.reserved = 0u;

        frame.crc16 = crc16_modbus((const uint8_t *)&frame, (uint16_t)(RC_FORWARD_FRAME_LEN - sizeof(frame.crc16)));

        /* 有新遥控帧到达即更新待发缓存：USB 忙时保留最新一帧 */
        s_pending_frame = frame;
        s_pending = 1U;
        s_last_remote_frame = current_frame;
    }

    if (CDC_IsHostOpen_FS() == 0U) {
        rc_forward_set_cdc_busy(0U);
        return;
    }
    cdc_busy = rc_forward_is_cdc_busy(now_tick);

    /* 空闲且有待发数据才发送一次；否则不重复发送历史帧。 */
    if (s_pending && !cdc_busy) {
        send_result = rc_forward_send_impl(&s_pending_frame);
        if (send_result == USBD_OK) {
            s_pending = 0U;
        }
    }
    (void)send_result;
}
