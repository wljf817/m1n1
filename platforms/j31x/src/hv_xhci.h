/* SPDX-License-Identifier: MIT */

#ifndef HV_XHCI_H
#define HV_XHCI_H

#include "types.h"

/*
 * Emulated xHCI controller with one high-speed USB HID device (boot keyboard + boot mouse)
 * for the standalone hypervisor guest. Guest-visible MMIO window and interrupt:
 */
/*
 * Run 37: Windows refuses to start an ACPI-enumerated device whose _CRS memory lies above 4 GB
 * (QWordMemory at 0x5300000000 -> START fails with STATUS_INVALID_PARAMETER before the driver
 * runs), while a Memory32Fixed window works and usbxhci goes on to enumerate our device. So the
 * window lives below 4 GB, where this guest has neither RAM (0x10000000000+) nor MMIO
 * (0x200000000+). The GSIV is one no device in the J314 device tree uses (the vGIC unmasks the
 * same AIC IRQ).
 */
#define HV_XHCI_BASE 0xF0000000ULL
#define HV_XHCI_SIZE 0x10000
#define HV_XHCI_IRQ  201

#define HV_XHCI_NWIN 1
static const u64 hv_xhci_win_base[HV_XHCI_NWIN] = {HV_XHCI_BASE};
static const u32 hv_xhci_win_irq[HV_XHCI_NWIN] = {HV_XHCI_IRQ};
static const char *const hv_xhci_win_name[HV_XHCI_NWIN] = {"XHC0"};

/* Interfaces of the emulated device */
#define HV_USBHID_KBD   0
#define HV_USBHID_MOUSE 1

int hv_xhci_init(u64 guest_base, u64 guest_end);
/* Called from the hypervisor tick (boot CPU): poll inputs, complete pending transfers, interrupt */
void hv_xhci_tick(void);
/* Print controller/device state on the console (debug overlay) */
void hv_xhci_dump(void);
/* Queue an input report for one of the interfaces (boot keyboard: 8 bytes, boot mouse: 4 bytes) */
void hv_usbhid_push(int iface, const u8 *report, u32 len);
/* Keyboard LED state last set by the guest (SET_REPORT), bit 1 = caps lock */
u8 hv_usbhid_leds(void);

#endif
