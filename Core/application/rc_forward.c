/**
  *****************************************************************************/
#include "rc_forward.h"
#include "remote_control.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
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
static volatile uint8_t s_tx_active = 0U;
static volatile uint32_t s_tx_start_tick = 0U;
static uint8_t s_pending = 0U;
static uint8_t s_recovery_requested = 0U;
static uint32_t s_cdc_busy_since = 0U;
static rc_forward_frame_t s_pending_frame;
static rc_forward_frame_t s_tx_frame;

_Static_assert(sizeof(rc_forward_frame_t) == RC_FORWARD_FRAME_LEN, "rc_forward frame must be 24 bytes");

static uint8_t rc_forward_tx_timed_out(uint32_t now_tick)
{
    uint8_t timed_out = 0U;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if ((s_tx_active != 0U) &&
        ((now_tick - s_tx_start_tick) > RC_FORWARD_USB_TX_BUSY_TIMEOUT_MS)) {
        s_pending_frame = s_tx_frame;
        s_pending = 1U;
        s_tx_active = 0U;
        s_tx_start_tick = 0U;
        timed_out = 1U;
    }
    __set_PRIMASK(primask);

    return timed_out;
}

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
    uint32_t primask = 0U;

    if (frame == NULL) {
        return USBD_FAIL;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if ((s_tx_active != 0U) || (CDC_IsTxReady_FS() == 0U)) {
        __set_PRIMASK(primask);
        return USBD_BUSY;
    }

    s_tx_frame = *frame;
    s_tx_active = 1U;
    s_tx_start_tick = HAL_GetTick();
    result = CDC_Transmit_FS((uint8_t *)&s_tx_frame, RC_FORWARD_FRAME_LEN);
    if (result != USBD_OK) {
        s_tx_active = 0U;
        s_tx_start_tick = 0U;
    }
    __set_PRIMASK(primask);
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
    s_tx_active = 0U;
    s_tx_start_tick = 0U;
    s_cdc_busy_since = 0U;
}

void rc_forward_notify_usb_unavailable(void)
{
    s_tx_active = 0U;
    s_tx_start_tick = 0U;
    s_recovery_requested = 0U;
    s_cdc_busy_since = 0U;
}

void rc_forward_notify_usb_resume(void)
{
    if (s_tx_active != 0U) {
        s_tx_start_tick = HAL_GetTick();
    }
}

void rc_forward_init(void)
{
    /* 以当前 DBUS 帧计数初始化，避免上电重复转发上一拍 */
    s_last_remote_frame = remote_control_get_frame_count();
    s_seq = 0U;
    s_tx_active = 0U;
    s_tx_start_tick = 0U;
    s_pending = 0U;
    s_recovery_requested = 0U;
    s_cdc_busy_since = 0U;
    memset((void *)&s_pending_frame, 0, sizeof(s_pending_frame));
    memset((void *)&s_tx_frame, 0, sizeof(s_tx_frame));
}

void rc_forward_poll(void)
{
    RC_ctrl_t rc_snapshot;
    uint32_t current_frame = 0U;
    uint8_t send_result = USBD_BUSY;
    const uint32_t now_tick = HAL_GetTick();

    if ((CDC_IsHostOpen_FS() != 0U) &&
        (s_recovery_requested == 0U) &&
        rc_forward_tx_timed_out(now_tick)) {
        s_recovery_requested = 1U;
        USB_DEVICE_RequestRecovery(USB_RECOVERY_REASON_TX_TIMEOUT);
        return;
    }

    current_frame = remote_control_get_frame_count();
    if (current_frame != s_last_remote_frame) {
        current_frame = remote_control_copy(&rc_snapshot);
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
        s_cdc_busy_since = 0U;
        return;
    }

    /* 空闲且有待发数据才发送一次；否则不重复发送历史帧。 */
    if (s_pending && (s_tx_active == 0U)) {
        if (CDC_IsTxReady_FS() == 0U) {
            if (s_cdc_busy_since == 0U) {
                s_cdc_busy_since = now_tick;
            } else if ((s_recovery_requested == 0U) &&
                       ((now_tick - s_cdc_busy_since) > RC_FORWARD_USB_TX_BUSY_TIMEOUT_MS)) {
                /* 上层空闲但 CDC TxState 长期非零，说明内部状态已失步。 */
                s_recovery_requested = 1U;
                s_cdc_busy_since = 0U;
                USB_DEVICE_RequestRecovery(USB_RECOVERY_REASON_TX_TIMEOUT);
            }
            return;
        }
        s_cdc_busy_since = 0U;
        send_result = rc_forward_send_impl(&s_pending_frame);
        if (send_result == USBD_OK) {
            s_pending = 0U;
        }
    }
    (void)send_result;
}
