#include "usb_retry_guard.h"

#include "stm32f4xx.h"
#include "stm32f4xx_ll_usb.h"

#include <stdint.h>

#define USB_RETRY_GUARD_MAGIC       0x55534252UL /* "USBR" */
#define USB_RETRY_GUARD_MAX_RESETS  3U

typedef struct {
    uint32_t magic;
    uint32_t consecutive_resets;
} usb_retry_guard_record_t;

static volatile usb_retry_guard_record_t s_usb_retry_guard
    __attribute__((section(".noinit.usb_retry_guard"), aligned(8)));

static void usb_emergency_disconnect(void)
{
    if ((RCC->AHB2ENR & RCC_AHB2ENR_OTGFSEN) != 0U) {
        USB_OTG_DeviceTypeDef *usb_device =
            (USB_OTG_DeviceTypeDef *)((uintptr_t)USB_OTG_FS + USB_OTG_DEVICE_BASE);
        usb_device->DCTL |= USB_OTG_DCTL_SDIS;
        __DSB();
    }
}

void usb_retry_guard_reset_or_halt(void)
{
    uint32_t reset_count = 0U;

    __disable_irq();
    usb_emergency_disconnect();

    if (s_usb_retry_guard.magic == USB_RETRY_GUARD_MAGIC) {
        reset_count = s_usb_retry_guard.consecutive_resets;
    }

    reset_count++;
    s_usb_retry_guard.magic = USB_RETRY_GUARD_MAGIC;
    s_usb_retry_guard.consecutive_resets = reset_count;
    __DSB();

    if (reset_count <= USB_RETRY_GUARD_MAX_RESETS) {
        NVIC_SystemReset();
    }

    for (;;) {
        __DSB();
        __WFI();
    }
}

void usb_retry_guard_mark_stable(void)
{
    if (s_usb_retry_guard.magic == USB_RETRY_GUARD_MAGIC) {
        s_usb_retry_guard.consecutive_resets = 0U;
    }
}
