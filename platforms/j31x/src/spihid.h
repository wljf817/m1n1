/* SPDX-License-Identifier: MIT */

#ifndef SPIHID_H
#define SPIHID_H

#include "types.h"

/* Devices behind the SPI HID transport (Apple's "ipd" node: internal keyboard + trackpad) */
#define SPIHID_DEV_MNGT 0x00
#define SPIHID_DEV_KBD  0x01
#define SPIHID_DEV_TP   0x02
#define SPIHID_DEV_INFO 0xd0

#define SPIHID_MAX_DEVICES 3
#define SPIHID_DESC_MAX    512

struct spihid_iface {
    char name[32];
    u8 country;
    u16 max_input_len;
    u16 max_output_len;
    u16 max_control_len;
    u8 desc[SPIHID_DESC_MAX];
    u32 desc_len;
    bool ready;
};

/* Called with every input report (payload after the message header, report ID first) */
typedef void (*spihid_report_cb_t)(u8 device, const u8 *report, u32 len);

/* Power the device on, read the device/interface info and the HID report descriptors. */
int spihid_init(void);
bool spihid_is_ready(void);

/* Poll the interrupt line; if asserted, read and process one packet. Returns true if one was read. */
bool spihid_poll(void);

void spihid_set_report_cb(spihid_report_cb_t cb);
const struct spihid_iface *spihid_get_iface(u8 device);

/* Output / feature reports towards the device (caps lock LED, trackpad multitouch mode...) */
int spihid_set_report(u8 device, u8 report_id, const u8 *buf, u32 len);
int spihid_output_report(u8 device, const u8 *buf, u32 len);

/* "kbdtest=1": bring the device up and print every report, forever */
void spihid_selftest(void) __attribute__((noreturn));

#endif
