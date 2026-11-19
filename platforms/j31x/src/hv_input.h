/* SPDX-License-Identifier: MIT */

#ifndef HV_INPUT_H
#define HV_INPUT_H

#include "types.h"

/* Internal keyboard/trackpad -> emulated USB HID device for the standalone hypervisor guest */
int hv_input_init(u64 guest_base, u64 guest_end);
void hv_input_tick(void);
void hv_input_stats(void);
/* hv_boot.c: screen to the host console (on) / back to the guest (off) */
void hv_debug_overlay(bool on);

#endif
