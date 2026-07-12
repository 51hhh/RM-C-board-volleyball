#ifndef USB_RETRY_GUARD_H
#define USB_RETRY_GUARD_H

#ifdef __cplusplus
extern "C" {
#endif

void usb_retry_guard_reset_or_halt(void) __attribute__((noreturn));
void usb_retry_guard_mark_stable(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_RETRY_GUARD_H */
