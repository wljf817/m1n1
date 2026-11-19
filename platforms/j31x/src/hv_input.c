/* SPDX-License-Identifier: MIT */

/*
 * Glue between the internal keyboard/trackpad (spihid.c, real hardware driven at EL2) and the
 * emulated USB HID device (hv_xhci.c) the guest sees.
 */

#include "hv_input.h"
#include "hv_xhci.h"
#include "spihid.h"
#include "string.h"
#include "utils.h"

static bool input_ready;
static u32 kbd_reports, tp_reports, dropped;
static u8 last_leds;

/*
 * Keyboard: the SPI device's interface is "Keyboard / Boot", its first input report ID 1 is the
 * boot keyboard report (modifiers, reserved, 6 key codes) followed by Apple extras (Fn key).
 * Report ID 0x11/0x12 etc. carry media keys; those are left alone for now.
 */
static bool overlay_on, hotkey_down;

static void kbd_report(const u8 *r, u32 len)
{
    /* Fn+Esc: toggle the hypervisor debug overlay (byte 9 = Fn, 0x29 = Esc) */
    if (len >= 10 && r[0] == 0x01) {
        bool hot = r[9] && (r[3] == 0x29 || r[4] == 0x29);
        if (hot && !hotkey_down) {
            overlay_on = !overlay_on;
            hv_debug_overlay(overlay_on);
        }
        hotkey_down = hot;
        if (hot)
            return;
    }
    if (len >= 9 && r[0] == 0x01) {
        hv_usbhid_push(HV_USBHID_KBD, r + 1, 8);
        kbd_reports++;
        return;
    }
    dropped++;
}

/*
 * Trackpad. Its report ID 2 starts with a plain HID mouse report {id, buttons, rel_x, rel_y,
 * pad[4]} but the device never fills in rel_x/rel_y (run 29: always 0, only the button and a
 * "finger down" flag move). After the {0x02, 0x01} feature report (hid-magicmouse.c,
 * magicmouse_enable_multitouch) the same report continues with the vendor multitouch payload:
 * struct tp_header (38 bytes: unknown[22], num_fingers, buttons, unknown[14]) and one 30 byte
 * struct tp_finger per finger (abs_x/abs_y at +4/+6 as signed 16 bit, touch_major at +18).
 * Relative motion for the boot mouse is derived from the first finger's absolute position.
 */
#define TP_MOUSE_HDR 8
#define TP_HDR_SIZE  38
#define TP_FINGER    30
#define TP_SCALE_DIV 8 /* trackpad units (~95 per mm) per mouse count */

static bool tp_mt_enabled;
static bool tp_tracking;
static int tp_prev_x, tp_prev_y, tp_acc_x, tp_acc_y;
static u8 tp_prev_buttons;

static void tp_enable_mt(void)
{
    u8 mt[2] = {0x02, 0x01};
    if (spihid_set_report(SPIHID_DEV_TP, 0x02, mt, sizeof(mt)) == 0) {
        tp_mt_enabled = true;
        printf("HV: trackpad multitouch mode on\n");
    } else {
        printf("HV: trackpad multitouch mode request failed\n");
    }
}

static inline s16 le16s(const u8 *p)
{
    return (s16)(p[0] | (p[1] << 8));
}

static void tp_report(const u8 *r, u32 len)
{
    tp_reports++;
    if (len >= 1 && r[0] == 0x60) {
        /* touch controller reset: it forgot the mode */
        tp_mt_enabled = false;
        tp_enable_mt();
        return;
    }
    if (len < 4 || r[0] != 0x02) {
        dropped++;
        return;
    }

    u8 buttons = r[1] & 0x01;
    int dx = 0, dy = 0;
    u32 fingers = 0;

    if (len >= TP_MOUSE_HDR + TP_HDR_SIZE) {
        const u8 *hdr = r + TP_MOUSE_HDR;
        u32 n = hdr[22];
        buttons |= hdr[23] & 0x01;
        u32 avail = (len - TP_MOUSE_HDR - TP_HDR_SIZE) / TP_FINGER;
        if (n > avail)
            n = avail;
        const u8 *first = NULL;
        for (u32 i = 0; i < n; i++) {
            const u8 *f = hdr + TP_HDR_SIZE + i * TP_FINGER;
            if (le16s(f + 18) == 0) /* touch_major: finger lifted */
                continue;
            fingers++;
            if (!first)
                first = f;
        }
        if (first) {
            int x = le16s(first + 4);
            int y = -le16s(first + 6); /* y axis is inverted */
            if (tp_tracking) {
                tp_acc_x += x - tp_prev_x;
                tp_acc_y += y - tp_prev_y;
                dx = tp_acc_x / TP_SCALE_DIV;
                dy = tp_acc_y / TP_SCALE_DIV;
                tp_acc_x -= dx * TP_SCALE_DIV;
                tp_acc_y -= dy * TP_SCALE_DIV;
            } else {
                tp_acc_x = tp_acc_y = 0;
            }
            tp_prev_x = x;
            tp_prev_y = y;
            tp_tracking = true;
        } else {
            tp_tracking = false;
        }
    }

    /* Physical click: one finger = left, two or more = right */
    u8 btn = 0;
    if (buttons)
        btn = fingers >= 2 ? 0x02 : 0x01;

    if (dx > 127)
        dx = 127;
    if (dx < -127)
        dx = -127;
    if (dy > 127)
        dy = 127;
    if (dy < -127)
        dy = -127;

    if (dx || dy || btn != tp_prev_buttons) {
        u8 m[4] = {btn, (u8)(s8)dx, (u8)(s8)dy, 0};
        hv_usbhid_push(HV_USBHID_MOUSE, m, sizeof(m));
        tp_prev_buttons = btn;
    }
}

static void on_report(u8 device, const u8 *report, u32 len)
{
    if (device == SPIHID_DEV_KBD)
        kbd_report(report, len);
    else if (device == SPIHID_DEV_TP)
        tp_report(report, len);
}

int hv_input_init(u64 guest_base, u64 guest_end)
{
    int ret = spihid_init();
    if (ret < 0) {
        printf("HV: internal keyboard not available, no USB input device\n");
        return ret;
    }
    spihid_set_report_cb(on_report);
    tp_enable_mt();
    if (hv_xhci_init(guest_base, guest_end) < 0)
        return -1;
    input_ready = true;
    return 0;
}

void hv_input_tick(void)
{
    if (!input_ready)
        return;
    /* Drain whatever the device has queued (one packet per poll) */
    for (int i = 0; i < 16 && spihid_poll(); i++)
        ;
    hv_xhci_tick();

    /* Keyboard LEDs set by the guest -> real caps lock LED */
    u8 leds = hv_usbhid_leds();
    if (leds != last_leds) {
        last_leds = leds;
        u8 out[2] = {0x01, leds};
        spihid_output_report(SPIHID_DEV_KBD, out, sizeof(out));
    }
}

void hv_input_stats(void)
{
    printf("HV: input: ready %d, kbd %u tp %u dropped %u, trackpad mt %d, leds %02x\n", input_ready,
           kbd_reports, tp_reports, dropped, tp_mt_enabled, last_leds);
}
