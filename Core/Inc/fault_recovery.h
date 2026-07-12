#ifndef FAULT_RECOVERY_H
#define FAULT_RECOVERY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    FAULT_REASON_HARDFAULT = 1U,
    FAULT_REASON_MEMMANAGE = 2U,
    FAULT_REASON_BUSFAULT = 3U,
    FAULT_REASON_USAGEFAULT = 4U,
    FAULT_REASON_NMI = 5U,
    FAULT_REASON_ERROR_HANDLER = 0xE001U
};

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t consecutive_faults;
    uint32_t reason;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t psr;
    uint32_t exc_return;
    uint32_t frame_valid;
} fault_record_t;

extern volatile fault_record_t g_fault_record;
extern uint32_t g_fault_emergency_stack[128];

void Fault_Capture(uint32_t *frame, uint32_t exc_return, uint32_t reason)
    __attribute__((noreturn));
void fault_recovery_fatal(uint32_t reason) __attribute__((noreturn));
void fault_recovery_mark_stable(void);

#ifdef __cplusplus
}
#endif

#endif /* FAULT_RECOVERY_H */
