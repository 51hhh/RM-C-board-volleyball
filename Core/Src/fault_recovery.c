#include "fault_recovery.h"

#include "stm32f4xx.h"
#include "stm32f4xx_ll_usb.h"

#include <stddef.h>

#define FAULT_RECORD_MAGIC          0x4641554CU
#define FAULT_RECORD_VERSION        1U
#define FAULT_RESET_LIMIT           3U
#define FAULT_INVALID_REGISTER      0xFFFFFFFFU
#define FAULT_STACKING_ERROR_MASK   0x00003838U

#define SRAM_START_ADDR             0x20000000UL
#define SRAM_END_ADDR               0x20020000UL
#define CCMRAM_START_ADDR           0x10000000UL
#define CCMRAM_END_ADDR             0x10010000UL

volatile fault_record_t g_fault_record __attribute__((section(".noinit.fault"), aligned(8)));
uint32_t g_fault_emergency_stack[128] __attribute__((section(".ccmram"), aligned(8)));

static uint8_t range_contains(uintptr_t start, uintptr_t end, uintptr_t address, size_t length)
{
    if ((address < start) || (address >= end)) {
        return 0U;
    }
    return (length <= (size_t)(end - address)) ? 1U : 0U;
}

static uint8_t exception_frame_is_valid(const uint32_t *frame, uint32_t cfsr,
                                        size_t frame_words)
{
    uintptr_t address = (uintptr_t)frame;

    if ((address & 0x3U) != 0U) {
        return 0U;
    }
    if ((cfsr & FAULT_STACKING_ERROR_MASK) != 0U) {
        return 0U;
    }
    if (range_contains(SRAM_START_ADDR, SRAM_END_ADDR, address,
                       frame_words * sizeof(uint32_t))) {
        return 1U;
    }
    return range_contains(CCMRAM_START_ADDR, CCMRAM_END_ADDR, address,
                          frame_words * sizeof(uint32_t));
}

static void fault_record_prepare(uint32_t reason)
{
    uint32_t count = 0U;

    if ((g_fault_record.magic == FAULT_RECORD_MAGIC) &&
        (g_fault_record.version == FAULT_RECORD_VERSION)) {
        count = g_fault_record.consecutive_faults;
    }

    /* 最后写 magic，避免复位中断写入时把半条记录误判为有效。 */
    g_fault_record.magic = 0U;
    g_fault_record.version = FAULT_RECORD_VERSION;
    g_fault_record.consecutive_faults = count + 1U;
    g_fault_record.reason = reason;
    g_fault_record.cfsr = SCB->CFSR;
    g_fault_record.hfsr = SCB->HFSR;
    g_fault_record.mmfar = SCB->MMFAR;
    g_fault_record.bfar = SCB->BFAR;
    g_fault_record.r0 = FAULT_INVALID_REGISTER;
    g_fault_record.r1 = FAULT_INVALID_REGISTER;
    g_fault_record.r2 = FAULT_INVALID_REGISTER;
    g_fault_record.r3 = FAULT_INVALID_REGISTER;
    g_fault_record.r12 = FAULT_INVALID_REGISTER;
    g_fault_record.lr = FAULT_INVALID_REGISTER;
    g_fault_record.pc = FAULT_INVALID_REGISTER;
    g_fault_record.psr = FAULT_INVALID_REGISTER;
    g_fault_record.exc_return = FAULT_INVALID_REGISTER;
    g_fault_record.frame_valid = 0U;
    __DSB();
    g_fault_record.magic = FAULT_RECORD_MAGIC;
    __DSB();
}

static void usb_force_disconnect(void)
{
    if ((RCC->AHB2ENR & RCC_AHB2ENR_OTGFSEN) != 0U) {
        USB_OTG_DeviceTypeDef *usb_device =
            (USB_OTG_DeviceTypeDef *)((uintptr_t)USB_OTG_FS + USB_OTG_DEVICE_BASE);
        usb_device->DCTL |= USB_OTG_DCTL_SDIS;
        __DSB();
    }
}

static void fault_halt(void) __attribute__((noreturn));
static void fault_halt(void)
{
    __disable_irq();
    for (;;) {
        __DSB();
        __WFI();
    }
}

static void fault_finish(void) __attribute__((noreturn));
static void fault_finish(void)
{
    usb_force_disconnect();

    if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U) {
        __BKPT(0);
        fault_halt();
    }
    if (g_fault_record.consecutive_faults >= FAULT_RESET_LIMIT) {
        fault_halt();
    }

    __DSB();
    NVIC_SystemReset();
    fault_halt();
}

void Fault_Capture(uint32_t *frame, uint32_t exc_return, uint32_t reason)
{
    const size_t core_frame_offset = ((exc_return & (1UL << 4)) == 0U) ? 18U : 0U;
    const size_t stacked_words = core_frame_offset + 8U;
    uint32_t *core_frame = frame;

    __disable_irq();
    fault_record_prepare(reason);
    g_fault_record.exc_return = exc_return;

    if (exception_frame_is_valid(frame, g_fault_record.cfsr, stacked_words)) {
        core_frame += core_frame_offset;
        g_fault_record.r0 = core_frame[0];
        g_fault_record.r1 = core_frame[1];
        g_fault_record.r2 = core_frame[2];
        g_fault_record.r3 = core_frame[3];
        g_fault_record.r12 = core_frame[4];
        g_fault_record.lr = core_frame[5];
        g_fault_record.pc = core_frame[6];
        g_fault_record.psr = core_frame[7];
        g_fault_record.frame_valid = 1U;
    }

    fault_finish();
}

void fault_recovery_fatal(uint32_t reason)
{
    __disable_irq();
    fault_record_prepare(reason);
    fault_finish();
}

void fault_recovery_mark_stable(void)
{
    if ((g_fault_record.magic == FAULT_RECORD_MAGIC) &&
        (g_fault_record.version == FAULT_RECORD_VERSION)) {
        g_fault_record.consecutive_faults = 0U;
    }
}
