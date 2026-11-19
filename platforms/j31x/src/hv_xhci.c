/* SPDX-License-Identifier: MIT */

/*
 * A small emulated xHCI host controller for the standalone hypervisor guest, with a single
 * high-speed USB device behind its one root port: a composite HID device with a boot-protocol
 * keyboard interface and a boot-protocol mouse interface. The real input comes from the MacBook's
 * internal keyboard/trackpad (spihid.c), which Windows cannot drive itself; Windows' inbox
 * usbxhci/usbhub3/hidusb/kbdhid/mouhid stack drives this instead.
 *
 * Scope: what the Windows and EDK2 drivers actually use. One interrupter, no streams, no
 * scratchpad buffers, no isochronous endpoints, 32 byte contexts, 64 bit addressing, event data
 * TRBs, short packets, EP0 stalls with reset endpoint / set TR dequeue recovery. Guest memory is
 * identity mapped at stage 2, so guest DMA addresses are used directly.
 *
 * The MMIO window is HV_XHCI_BASE (hv_map_hook), the interrupt a vGIC SPI; interrupt-IN transfers
 * that find no report pending stay on their ring and are completed from hv_xhci_tick() once a
 * report arrives.
 */

#include "hv_xhci.h"
#include "hv.h"
#include "memory.h"
#include "string.h"
#include "utils.h"

#define xhci_dbg(...)                                                                              \
    do {                                                                                           \
        if (xhci_debug)                                                                            \
            printf("xhci: " __VA_ARGS__);                                                          \
    } while (0)

static bool xhci_debug = false;

/* ------------------------------------------------------------------------------------------ */
/* Register map                                                                                 */

#define CAP_LEN     0x40
#define OP_BASE     CAP_LEN
#define PORT_BASE   (OP_BASE + 0x400)
#define RT_BASE     0x1000
#define IR0_BASE    (RT_BASE + 0x20)
#define DB_BASE     0x2000
#define XECP_BASE   0x80
#define MAX_SLOTS   4
#define MAX_PORTS   1
#define ERST_MAX    4 /* 2^4 segments */

/* USBCMD */
#define CMD_RS     BIT(0)
#define CMD_HCRST  BIT(1)
#define CMD_INTE   BIT(2)
#define CMD_HSEE   BIT(3)
#define CMD_LHCRST BIT(7)
#define CMD_CSS    BIT(8)
#define CMD_CRS    BIT(9)
#define CMD_EWE    BIT(10)
#define CMD_EU3S   BIT(11)
/* USBSTS */
#define STS_HCH  BIT(0)
#define STS_HSE  BIT(2)
#define STS_EINT BIT(3)
#define STS_PCD  BIT(4)
#define STS_SSS  BIT(8)
#define STS_RSS  BIT(9)
#define STS_SRE  BIT(10)
#define STS_CNR  BIT(11)
#define STS_HCE  BIT(12)
/* CRCR */
#define CRCR_RCS BIT(0)
#define CRCR_CS  BIT(1)
#define CRCR_CA  BIT(2)
#define CRCR_CRR BIT(3)
/* PORTSC */
#define PORTSC_CCS   BIT(0)
#define PORTSC_PED   BIT(1)
#define PORTSC_OCA   BIT(3)
#define PORTSC_PR    BIT(4)
#define PORTSC_PLS   GENMASK(8, 5)
#define PORTSC_PP    BIT(9)
#define PORTSC_SPEED GENMASK(13, 10)
#define PORTSC_PIC   GENMASK(15, 14)
#define PORTSC_LWS   BIT(16)
#define PORTSC_CSC   BIT(17)
#define PORTSC_PEC   BIT(18)
#define PORTSC_WRC   BIT(19)
#define PORTSC_OCC   BIT(20)
#define PORTSC_PRC   BIT(21)
#define PORTSC_PLC   BIT(22)
#define PORTSC_CEC   BIT(23)
#define PORTSC_CAS   BIT(24)
#define PORTSC_WCE   BIT(25)
#define PORTSC_WDE   BIT(26)
#define PORTSC_WOE   BIT(27)
#define PORTSC_DR    BIT(30)
#define PORTSC_WPR   BIT(31)
#define PORTSC_CHANGE_BITS                                                                         \
    (PORTSC_CSC | PORTSC_PEC | PORTSC_WRC | PORTSC_OCC | PORTSC_PRC | PORTSC_PLC | PORTSC_CEC)
#define PLS_U0      0
#define PLS_U3      3
#define PLS_DISABLED 4
#define PLS_RXDETECT 5
#define PLS_POLLING 7
#define SPEED_HS    3
/* IMAN */
#define IMAN_IP BIT(0)
#define IMAN_IE BIT(1)

/* TRB types */
#define TRB_NORMAL        1
#define TRB_SETUP         2
#define TRB_DATA          3
#define TRB_STATUS        4
#define TRB_ISOCH         5
#define TRB_LINK          6
#define TRB_EVENT_DATA    7
#define TRB_NOOP          8
#define TRB_ENABLE_SLOT   9
#define TRB_DISABLE_SLOT  10
#define TRB_ADDRESS_DEV   11
#define TRB_CONFIG_EP     12
#define TRB_EVAL_CTX      13
#define TRB_RESET_EP      14
#define TRB_STOP_EP       15
#define TRB_SET_TR_DEQ    16
#define TRB_RESET_DEV     17
#define TRB_FORCE_EVENT   18
#define TRB_NEG_BW        19
#define TRB_SET_LAT       20
#define TRB_GET_PORT_BW   21
#define TRB_FORCE_HDR     22
#define TRB_CMD_NOOP      23
#define TRB_TRANSFER_EVT  32
#define TRB_CMD_COMPLETE  33
#define TRB_PORT_STATUS   34
#define TRB_HOST_CTRL_EVT 37
#define TRB_MFINDEX_WRAP  39

/* TRB control bits */
#define TRB_C    BIT(0)
#define TRB_TC   BIT(1)
#define TRB_ENT  BIT(1)
#define TRB_ISP  BIT(2)
#define TRB_NS   BIT(3)
#define TRB_CH   BIT(4)
#define TRB_IOC  BIT(5)
#define TRB_IDT  BIT(6)
#define TRB_BEI  BIT(9)
#define TRB_TYPE GENMASK(15, 10)
#define TRB_DIR  BIT(16)
#define TRB_TRT  GENMASK(17, 16)
#define TRB_BSR  BIT(9)
#define TRB_DC   BIT(9)
#define TRB_ED   BIT(2)
#define TRB_EP_ID   GENMASK(20, 16)
#define TRB_SLOT_ID GENMASK(31, 24)
#define TRB_LEN     GENMASK(16, 0)
#define TRB_CC      GENMASK(31, 24)

/* Completion codes */
#define CC_SUCCESS          1
#define CC_DATA_BUF_ERR     2
#define CC_BABBLE           3
#define CC_USB_TRANS_ERR    4
#define CC_TRB_ERR          5
#define CC_STALL            6
#define CC_RESOURCE_ERR     7
#define CC_NO_SLOTS         9
#define CC_SLOT_NOT_ENABLED 11
#define CC_EP_NOT_ENABLED   12
#define CC_SHORT_PACKET     13
#define CC_PARAMETER_ERR    17
#define CC_CONTEXT_STATE    19
#define CC_CMD_RING_STOPPED 24
#define CC_STOPPED          26

/* Slot / endpoint states */
#define SLOT_DISABLED   0
#define SLOT_DEFAULT    1
#define SLOT_ADDRESSED  2
#define SLOT_CONFIGURED 3
#define EP_DISABLED 0
#define EP_RUNNING  1
#define EP_HALTED   2
#define EP_STOPPED  3
#define EP_ERROR    4

struct trb {
    u64 ptr;
    u32 status;
    u32 control;
};

#define TRB_TYPE_OF(t) FIELD_GET(TRB_TYPE, (t)->control)

/* ------------------------------------------------------------------------------------------ */
/* USB device: composite HID (boot keyboard + boot mouse), high speed                           */

#define USB_VID 0x1209 /* pid.codes */
#define USB_PID 0x0001 /* test PID */

static const u8 dev_desc[18] = {
    18,   0x01,               /* bLength, DEVICE */
    0x00, 0x02,               /* bcdUSB 2.00 */
    0x00, 0x00, 0x00,         /* class/subclass/protocol: per interface */
    64,                       /* bMaxPacketSize0 */
    USB_VID & 0xff, USB_VID >> 8, USB_PID & 0xff, USB_PID >> 8,
    0x00, 0x01,               /* bcdDevice 1.00 */
    1,    2,    3,            /* iManufacturer, iProduct, iSerialNumber */
    1,                        /* bNumConfigurations */
};

static const u8 kbd_report_desc[] = {
    0x05, 0x01, 0x09, 0x06, 0xa1, 0x01, 0x05, 0x07, 0x19, 0xe0, 0x29, 0xe7, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x95, 0x01, 0x75, 0x08, 0x81, 0x01, 0x95, 0x05, 0x75, 0x01,
    0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x91, 0x02, 0x95, 0x01, 0x75, 0x03, 0x91, 0x01, 0x95, 0x06,
    0x75, 0x08, 0x15, 0x00, 0x25, 0x65, 0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00, 0xc0,
};

static const u8 mouse_report_desc[] = {
    0x05, 0x01, 0x09, 0x02, 0xa1, 0x01, 0x09, 0x01, 0xa1, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x03,
    0x15, 0x00, 0x25, 0x01, 0x95, 0x03, 0x75, 0x01, 0x81, 0x02, 0x95, 0x01, 0x75, 0x05, 0x81, 0x01,
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38, 0x15, 0x81, 0x25, 0x7f, 0x75, 0x08, 0x95, 0x03,
    0x81, 0x06, 0xc0, 0xc0,
};

#define KBD_REPORT_LEN   8
#define MOUSE_REPORT_LEN 4
#define KBD_EP   1 /* endpoint number -> DCI 3 */
#define MOUSE_EP 2 /* endpoint number -> DCI 5 */
#define DCI_IN(ep) ((ep) * 2 + 1)

#define CFG_DESC_LEN (9 + 2 * (9 + 9 + 7))
static u8 cfg_desc[CFG_DESC_LEN];

static void build_cfg_desc(void)
{
    u8 *p = cfg_desc;
    const struct {
        u8 ep, protocol, report_len;
        u16 desc_len;
    } ifs[2] = {
        {KBD_EP, 1, KBD_REPORT_LEN, sizeof(kbd_report_desc)},
        {MOUSE_EP, 2, MOUSE_REPORT_LEN, sizeof(mouse_report_desc)},
    };

    u8 cfg[9] = {9, 0x02, CFG_DESC_LEN & 0xff, CFG_DESC_LEN >> 8, 2, 1, 0, 0x80, 50};
    memcpy(p, cfg, 9);
    p += 9;
    for (int i = 0; i < 2; i++) {
        u8 iface[9] = {9, 0x04, i, 0, 1, 0x03, 0x01, ifs[i].protocol, 0};
        u8 hid[9] = {9, 0x21, 0x11, 0x01, 0, 1, 0x22, ifs[i].desc_len & 0xff, ifs[i].desc_len >> 8};
        u8 ep[7] = {7, 0x05, 0x80 | ifs[i].ep, 0x03, 8, 0, 4 /* 2^(4-1) uframes = 1 ms */};
        memcpy(p, iface, 9);
        p += 9;
        memcpy(p, hid, 9);
        p += 9;
        memcpy(p, ep, 7);
        p += 7;
    }
}

static const char *const strings[] = {
    NULL,
    "bridge",
    "Apple Internal Keyboard / Trackpad (bridge)",
    "0001",
};

static const u8 qualifier_desc[10] = {10, 0x06, 0x00, 0x02, 0, 0, 0, 64, 1, 0};

struct usb_setup {
    u8 bmRequestType;
    u8 bRequest;
    u16 wValue;
    u16 wIndex;
    u16 wLength;
} PACKED;

/* Reports queued towards the guest */
#define RQ_SIZE 32
struct report_queue {
    u8 buf[RQ_SIZE][8];
    u8 len[RQ_SIZE];
    u32 head, tail;
};

static struct {
    u8 config;
    u8 protocol[2]; /* 0 = boot, 1 = report */
    u8 idle[2];
    u8 leds;
    struct report_queue rq[2];
    u8 last_report[2][8];
} usbdev;

static bool rq_empty(struct report_queue *q)
{
    return q->head == q->tail;
}

static void rq_push(struct report_queue *q, const u8 *d, u32 len)
{
    u32 next = (q->tail + 1) % RQ_SIZE;
    if (next == q->head) {
        /* full: drop the oldest */
        q->head = (q->head + 1) % RQ_SIZE;
    }
    len = min(len, 8u);
    memcpy(q->buf[q->tail], d, len);
    q->len[q->tail] = len;
    q->tail = next;
}

static u32 rq_pop(struct report_queue *q, u8 *out)
{
    if (rq_empty(q))
        return 0;
    u32 len = q->len[q->head];
    memcpy(out, q->buf[q->head], len);
    q->head = (q->head + 1) % RQ_SIZE;
    return len;
}

/* ------------------------------------------------------------------------------------------ */
/* Controller state                                                                             */

struct ring {
    u64 deq;
    bool ccs;
};

struct endpoint {
    u8 state;
    u8 type; /* EP context type field */
    u16 mps;
    struct ring ring;
    u32 interval;
};

struct slot {
    u8 state;
    u8 addr;
    u32 port;
    u64 out_ctx; /* output device context (guest) */
    struct endpoint ep[32];
    /* control transfer in progress on EP0 */
    struct usb_setup setup;
    bool have_setup;
    bool ctrl_in;
    bool ctrl_stall;
    u8 ctrl_buf[1024];
    u32 ctrl_len; /* response length (IN) or expected length (OUT) */
    u32 ctrl_pos;
};

static struct {
    bool inited;
    spinlock_t lock;

    u32 usbcmd, usbsts, dnctrl, config;
    u64 crcr;
    u64 dcbaap;
    struct ring cmd_ring;
    bool cmd_running;

    u32 portsc;

    /* interrupter 0 */
    u32 iman, imod, erstsz;
    u64 erstba, erdp;
    struct {
        u64 seg_base;
        u32 seg_size;
        u32 seg_idx;
        u32 enq;
        bool ccs;
        bool valid;
    } evt;
    bool irq_level;
    bool irq_injected;

    struct slot slots[MAX_SLOTS + 1];

    u64 ram_base, ram_top;
    u32 events, cmds, xfers, mmio_rd, mmio_wr, irqs;
    u32 win_access[HV_XHCI_NWIN];
    struct {
        u32 off, val;
        u8 write;
    } trace[24];
    u32 trace_n;
    struct {
        u64 ptr;
        u32 status, control, trb_len, trb_done;
    } evlog[16];
    u32 evlog_n, evlog_trb_len, evlog_trb_done;
    u8 setuplog[12][8];
    u32 setuplog_n;
} hc;

static void trace_access(u32 off, u32 val, bool write)
{
    /* The driver polls a few registers; keep the trace about state changes */
    if (!write && (off == OP_BASE + 0x00 || off == OP_BASE + 0x04 || off == RT_BASE ||
                   off == IR0_BASE + 0x00 || off == PORT_BASE))
        return;
    u32 i = hc.trace_n % ARRAY_SIZE(hc.trace);
    hc.trace[i].off = off;
    hc.trace[i].val = val;
    hc.trace[i].write = write;
    hc.trace_n++;
}

static inline bool guest_range_ok(u64 addr, u64 len)
{
    return addr >= hc.ram_base && addr + len <= hc.ram_top && addr + len >= addr;
}

static bool guest_read(u64 addr, void *buf, u32 len)
{
    if (!guest_range_ok(addr, len)) {
        printf("xhci: guest read 0x%lx+%u out of RAM\n", addr, len);
        return false;
    }
    memcpy(buf, (void *)addr, len);
    return true;
}

static bool guest_write(u64 addr, const void *buf, u32 len)
{
    if (!guest_range_ok(addr, len)) {
        printf("xhci: guest write 0x%lx+%u out of RAM\n", addr, len);
        return false;
    }
    memcpy((void *)addr, buf, len);
    return true;
}

static inline u32 guest_read32(u64 addr)
{
    u32 v = 0;
    guest_read(addr, &v, 4);
    return v;
}

static inline void guest_write32(u64 addr, u32 v)
{
    guest_write(addr, &v, 4);
}

static inline u64 guest_read64(u64 addr)
{
    u64 v = 0;
    guest_read(addr, &v, 8);
    return v;
}

/* ------------------------------------------------------------------------------------------ */
/* Event ring + interrupt                                                                       */

static void update_irq(void)
{
    bool level = (hc.usbsts & STS_EINT) && (hc.iman & IMAN_IP) && (hc.iman & IMAN_IE) &&
                 (hc.usbcmd & CMD_INTE);
    hc.irq_level = level;
    if (!level) {
        hc.irq_injected = false;
        return;
    }
    if (hc.irq_injected)
        return;
    /* Raise every SPI the guest enabled among the variants' interrupts (normally just one) */
    bool any = false;
    for (int w = 0; w < HV_XHCI_NWIN; w++) {
        u32 irq = hv_xhci_win_irq[w];
        bool dup = false;
        for (int v = 0; v < w; v++)
            if (hv_xhci_win_irq[v] == irq)
                dup = true;
        if (dup || !hv_vgic3_spi_enabled(irq))
            continue;
        hv_vgic_inject_spi(irq);
        any = true;
    }
    if (!any)
        return; /* retried from the tick */
    hc.irq_injected = true;
    hc.irqs++;
}

static bool evt_load_segment(void)
{
    if (!hc.erstsz || !hc.erstba) {
        hc.evt.valid = false;
        return false;
    }
    u64 entry = hc.erstba + hc.evt.seg_idx * 16;
    hc.evt.seg_base = guest_read64(entry) & ~0x3fULL;
    hc.evt.seg_size = guest_read32(entry + 8) & 0xffff;
    if (hc.evt.seg_size < 16 || !guest_range_ok(hc.evt.seg_base, hc.evt.seg_size * 16)) {
        printf("xhci: bad ERST segment %u: base 0x%lx size %u\n", hc.evt.seg_idx, hc.evt.seg_base,
               hc.evt.seg_size);
        hc.evt.valid = false;
        return false;
    }
    hc.evt.valid = true;
    return true;
}

static void evt_reset(void)
{
    hc.evt.seg_idx = 0;
    hc.evt.enq = 0;
    hc.evt.ccs = true;
    evt_load_segment();
}

static void post_event(u64 ptr, u32 status, u32 control)
{
    if (!hc.evt.valid)
        evt_reset(); /* ERST programmed in an order we did not catch */
    if (!hc.evt.valid || !(hc.usbcmd & CMD_RS))
        return;

    u64 addr = hc.evt.seg_base + hc.evt.enq * 16;
    u64 erdp = hc.erdp & ~0xfULL;
    /* Full if the next slot is where the consumer still is (one slot is kept free) */
    u32 next = hc.evt.enq + 1;
    u64 next_addr = next < hc.evt.seg_size ? hc.evt.seg_base + next * 16 : 0;
    if (next_addr && next_addr == erdp) {
        printf("xhci: event ring full, dropping event (type %lu)\n", FIELD_GET(TRB_TYPE, control));
        return;
    }

    struct trb e = {ptr, status, (control & ~TRB_C) | (hc.evt.ccs ? TRB_C : 0)};
    hc.evlog[hc.evlog_n % ARRAY_SIZE(hc.evlog)] =
        (typeof(hc.evlog[0])){ptr, status, control, hc.evlog_trb_len, hc.evlog_trb_done};
    hc.evlog_n++;
    hc.evlog_trb_len = hc.evlog_trb_done = 0;
    /* Write the cycle bit last */
    guest_write(addr, &e, 12);
    sysop("dmb ish");
    guest_write32(addr + 12, e.control);
    sysop("dmb ish");
    hc.events++;

    hc.evt.enq++;
    if (hc.evt.enq >= hc.evt.seg_size) {
        hc.evt.enq = 0;
        hc.evt.seg_idx++;
        if (hc.evt.seg_idx >= hc.erstsz) {
            hc.evt.seg_idx = 0;
            hc.evt.ccs = !hc.evt.ccs;
        }
        evt_load_segment();
    }

    hc.usbsts |= STS_EINT;
    hc.iman |= IMAN_IP;
    update_irq();
}

static void post_cmd_completion(u64 cmd_trb, u32 cc, u32 param, u32 slot_id)
{
    post_event(cmd_trb, FIELD_PREP(TRB_CC, cc) | (param & 0xffffff),
               FIELD_PREP(TRB_TYPE, TRB_CMD_COMPLETE) | FIELD_PREP(TRB_SLOT_ID, slot_id));
}

static void post_transfer_event(u64 ptr, u32 cc, u32 residual, u32 slot_id, u32 dci, bool ed)
{
    post_event(ptr, FIELD_PREP(TRB_CC, cc) | (residual & 0xffffff),
               FIELD_PREP(TRB_TYPE, TRB_TRANSFER_EVT) | FIELD_PREP(TRB_SLOT_ID, slot_id) |
                   FIELD_PREP(TRB_EP_ID, dci) | (ed ? TRB_ED : 0));
}

static void post_port_status(u32 port)
{
    hc.usbsts |= STS_PCD;
    post_event((u64)port << 24, FIELD_PREP(TRB_CC, CC_SUCCESS),
               FIELD_PREP(TRB_TYPE, TRB_PORT_STATUS));
}

/* ------------------------------------------------------------------------------------------ */
/* Rings                                                                                        */

/* Fetch the TRB at the ring's dequeue pointer, following link TRBs. False if the ring is empty. */
static bool ring_peek(struct ring *r, struct trb *t, u64 *trb_addr)
{
    for (int hops = 0; hops < 16; hops++) {
        if (!guest_read(r->deq, t, sizeof(*t)))
            return false;
        if (!!(t->control & TRB_C) != r->ccs)
            return false;
        if (TRB_TYPE_OF(t) == TRB_LINK) {
            if (t->control & TRB_TC)
                r->ccs = !r->ccs;
            r->deq = t->ptr & ~0xfULL;
            continue;
        }
        *trb_addr = r->deq;
        return true;
    }
    printf("xhci: link TRB loop at 0x%lx\n", r->deq);
    return false;
}

static void ring_advance(struct ring *r)
{
    r->deq += 16;
}

/* ------------------------------------------------------------------------------------------ */
/* Device: control requests                                                                     */

static u32 build_string_desc(u8 *out, u32 idx)
{
    if (idx == 0) {
        out[0] = 4;
        out[1] = 0x03;
        out[2] = 0x09;
        out[3] = 0x04;
        return 4;
    }
    if (idx >= ARRAY_SIZE(strings) || !strings[idx])
        return 0;
    const char *s = strings[idx];
    u32 n = strlen(s);
    if (n > 126)
        n = 126;
    out[0] = 2 + 2 * n;
    out[1] = 0x03;
    for (u32 i = 0; i < n; i++) {
        out[2 + 2 * i] = s[i];
        out[3 + 2 * i] = 0;
    }
    return 2 + 2 * n;
}

/* Returns false to stall. Fills slot->ctrl_buf/ctrl_len for IN requests. */
static bool handle_setup(struct slot *s)
{
    struct usb_setup *q = &s->setup;
    u8 *buf = s->ctrl_buf;
    u32 len = 0;
    bool in = q->bmRequestType & 0x80;
    u8 type = (q->bmRequestType >> 5) & 3; /* 0 std, 1 class, 2 vendor */
    u8 recip = q->bmRequestType & 0x1f;    /* 0 dev, 1 iface, 2 ep */

    s->ctrl_in = in;
    s->ctrl_pos = 0;
    s->ctrl_len = 0;

    xhci_dbg("setup %02x %02x val %04x idx %04x len %u\n", q->bmRequestType, q->bRequest, q->wValue,
             q->wIndex, q->wLength);

    if (type == 0) {
        switch (q->bRequest) {
        case 0x06: /* GET_DESCRIPTOR */
            if (recip == 0) {
                u8 dtype = q->wValue >> 8, didx = q->wValue & 0xff;
                switch (dtype) {
                case 0x01:
                    memcpy(buf, dev_desc, sizeof(dev_desc));
                    len = sizeof(dev_desc);
                    break;
                case 0x02:
                    memcpy(buf, cfg_desc, sizeof(cfg_desc));
                    len = sizeof(cfg_desc);
                    break;
                case 0x03:
                    len = build_string_desc(buf, didx);
                    if (!len)
                        return false;
                    break;
                case 0x06:
                    memcpy(buf, qualifier_desc, sizeof(qualifier_desc));
                    len = sizeof(qualifier_desc);
                    break;
                default:
                    return false;
                }
            } else if (recip == 1) {
                u8 dtype = q->wValue >> 8;
                u32 iface = q->wIndex & 0xff;
                if (iface > 1)
                    return false;
                if (dtype == 0x22) {
                    const u8 *d = iface == 0 ? kbd_report_desc : mouse_report_desc;
                    len = iface == 0 ? sizeof(kbd_report_desc) : sizeof(mouse_report_desc);
                    memcpy(buf, d, len);
                } else if (dtype == 0x21) {
                    memcpy(buf, cfg_desc + 9 + iface * 25 + 9, 9);
                    len = 9;
                } else {
                    return false;
                }
            } else {
                return false;
            }
            break;
        case 0x00: /* GET_STATUS */
            buf[0] = recip == 0 ? 0x01 : 0; /* self powered */
            buf[1] = 0;
            len = 2;
            break;
        case 0x01: /* CLEAR_FEATURE */
        case 0x03: /* SET_FEATURE */
            if (in)
                return false;
            break;
        case 0x09: /* SET_CONFIGURATION */
            usbdev.config = q->wValue & 0xff;
            break;
        case 0x08: /* GET_CONFIGURATION */
            buf[0] = usbdev.config;
            len = 1;
            break;
        case 0x0b: /* SET_INTERFACE */
            break;
        case 0x0a: /* GET_INTERFACE */
            buf[0] = 0;
            len = 1;
            break;
        default:
            return false;
        }
    } else if (type == 1 && recip == 1) {
        u32 iface = q->wIndex & 0xff;
        if (iface > 1)
            return false;
        switch (q->bRequest) {
        case 0x0a: /* SET_IDLE */
            usbdev.idle[iface] = q->wValue >> 8;
            break;
        case 0x02: /* GET_IDLE */
            buf[0] = usbdev.idle[iface];
            len = 1;
            break;
        case 0x0b: /* SET_PROTOCOL */
            usbdev.protocol[iface] = q->wValue & 1;
            break;
        case 0x03: /* GET_PROTOCOL */
            buf[0] = usbdev.protocol[iface];
            len = 1;
            break;
        case 0x09: /* SET_REPORT: data arrives in the data stage */
            if (q->wLength > sizeof(s->ctrl_buf))
                return false;
            s->ctrl_len = q->wLength;
            return true;
        case 0x01: /* GET_REPORT */
            len = iface == 0 ? KBD_REPORT_LEN : MOUSE_REPORT_LEN;
            if ((q->wValue >> 8) == 2 && iface == 0) {
                buf[0] = usbdev.leds;
                len = 1;
            } else {
                memcpy(buf, usbdev.last_report[iface], len);
            }
            break;
        default:
            return false;
        }
    } else {
        return false;
    }

    if (in) {
        if (len > q->wLength)
            len = q->wLength;
        s->ctrl_len = len;
    } else {
        s->ctrl_len = q->wLength;
    }
    return true;
}

/* OUT data stage finished: act on it */
static void handle_ctrl_out_data(struct slot *s)
{
    struct usb_setup *q = &s->setup;
    if ((q->bmRequestType & 0x7f) == 0x21 && q->bRequest == 0x09) {
        u32 iface = q->wIndex & 0xff;
        u8 rtype = q->wValue >> 8;
        if (iface == 0 && rtype == 2 && s->ctrl_pos >= 1) {
            usbdev.leds = s->ctrl_buf[0];
            xhci_dbg("keyboard LEDs 0x%02x\n", usbdev.leds);
        }
    }
}

/* ------------------------------------------------------------------------------------------ */
/* Transfers                                                                                    */

static void ep_halt(struct slot *s, u32 slot_id, u32 dci)
{
    struct endpoint *ep = &s->ep[dci];
    ep->state = EP_HALTED;
    if (s->out_ctx)
        guest_write32(s->out_ctx + dci * 32, (guest_read32(s->out_ctx + dci * 32) & ~7u) | EP_HALTED);
    UNUSED(slot_id);
}

/*
 * Execute one TD (TRBs up to the one without the chain bit) on an endpoint. Returns false if it
 * could not run (nothing to deliver on an interrupt IN endpoint, or the endpoint halted).
 */
static bool run_td(struct slot *s, u32 slot_id, u32 dci)
{
    struct endpoint *ep = &s->ep[dci];
    struct ring *r = &ep->ring;
    struct trb t;
    u64 taddr;

    if (!ring_peek(r, &t, &taddr))
        return false;

    bool is_ctrl = dci == 1;
    u8 report[8];
    u32 report_len = 0;
    u32 report_pos = 0;

    /* Interrupt IN endpoints: a TD consumes one report; wait if there is none */
    if (!is_ctrl) {
        int iface = dci == DCI_IN(KBD_EP) ? 0 : dci == DCI_IN(MOUSE_EP) ? 1 : -1;
        u8 ttype = TRB_TYPE_OF(&t);
        if (ttype == TRB_NORMAL || ttype == TRB_EVENT_DATA) {
            if (iface < 0)
                return false;
            if (rq_empty(&usbdev.rq[iface]))
                return false;
            report_len = rq_pop(&usbdev.rq[iface], report);
            memcpy(usbdev.last_report[iface], report, report_len);
        }
    }

    u32 td_total = 0;
    bool short_pkt = false;
    bool stall = false;
    bool skipping = false;

    for (int n = 0; n < 64; n++) {
        if (n > 0 && !ring_peek(r, &t, &taddr)) {
            /* TD continues past the enqueue pointer: should not happen, give up on it */
            printf("xhci: TD runs off the ring at 0x%lx (slot %u dci %u)\n", r->deq, slot_id, dci);
            return false;
        }
        u8 type = TRB_TYPE_OF(&t);
        u32 len = FIELD_GET(TRB_LEN, t.status);
        u32 done = 0;
        u32 cc = CC_SUCCESS;
        bool chain = t.control & TRB_CH;

        switch (type) {
        case TRB_SETUP:
            if (!is_ctrl) {
                cc = CC_TRB_ERR;
                break;
            }
            memcpy(&s->setup, &t.ptr, 8);
            memcpy(hc.setuplog[hc.setuplog_n % ARRAY_SIZE(hc.setuplog)], &t.ptr, 8);
            hc.setuplog_n++;
            s->have_setup = true;
            s->ctrl_stall = !handle_setup(s);
            done = 8;
            if (s->ctrl_stall)
                xhci_dbg("stalling request %02x %02x\n", s->setup.bmRequestType, s->setup.bRequest);
            break;

        case TRB_DATA:
        case TRB_NORMAL:
            if (skipping) {
                done = 0;
                break;
            }
            if (is_ctrl) {
                if (!s->have_setup) {
                    cc = CC_TRB_ERR;
                    break;
                }
                if (s->ctrl_stall) {
                    stall = true;
                    cc = CC_STALL;
                    break;
                }
                bool dir_in = type == TRB_DATA ? !!(t.control & TRB_DIR) : s->ctrl_in;
                if (dir_in) {
                    u32 avail = s->ctrl_len > s->ctrl_pos ? s->ctrl_len - s->ctrl_pos : 0;
                    done = min(len, avail);
                    if (done && !guest_write(t.ptr, s->ctrl_buf + s->ctrl_pos, done)) {
                        cc = CC_DATA_BUF_ERR;
                        break;
                    }
                    s->ctrl_pos += done;
                    if (done < len)
                        short_pkt = true;
                } else {
                    u32 room = sizeof(s->ctrl_buf) - s->ctrl_pos;
                    done = min(len, room);
                    if (t.control & TRB_IDT) {
                        /* immediate data: up to 8 bytes live in the TRB itself */
                        done = min(done, 8u);
                        memcpy(s->ctrl_buf + s->ctrl_pos, &t.ptr, done);
                    } else if (done && !guest_read(t.ptr, s->ctrl_buf + s->ctrl_pos, done)) {
                        cc = CC_DATA_BUF_ERR;
                        break;
                    }
                    s->ctrl_pos += done;
                    done = len;
                }
            } else {
                u32 avail = report_len - report_pos;
                done = min(len, avail);
                if (done) {
                    if (t.control & TRB_IDT) {
                        cc = CC_TRB_ERR;
                        break;
                    }
                    if (!guest_write(t.ptr, report + report_pos, done)) {
                        cc = CC_DATA_BUF_ERR;
                        break;
                    }
                }
                report_pos += done;
                if (done < len)
                    short_pkt = true;
            }
            break;

        case TRB_STATUS:
            if (!is_ctrl) {
                cc = CC_TRB_ERR;
                break;
            }
            if (s->ctrl_stall) {
                stall = true;
                cc = CC_STALL;
            } else if (!s->ctrl_in) {
                handle_ctrl_out_data(s);
            }
            s->have_setup = false;
            break;

        case TRB_EVENT_DATA:
            /* reports the TD so far; handled below */
            break;

        case TRB_NOOP:
            break;

        default:
            printf("xhci: unexpected transfer TRB type %u at 0x%lx\n", type, taddr);
            cc = CC_TRB_ERR;
            break;
        }

        td_total += done;
        ring_advance(r);
        hc.xfers++;
        hc.evlog_trb_len = len;
        hc.evlog_trb_done = done;

        if (type == TRB_EVENT_DATA) {
            if (t.control & TRB_IOC) {
                post_transfer_event(t.ptr, stall ? CC_STALL : short_pkt ? CC_SHORT_PACKET : CC_SUCCESS,
                                    td_total, slot_id, dci, true);
            }
        } else {
            if (cc == CC_SUCCESS && short_pkt && done < len)
                cc = CC_SHORT_PACKET;
            if (cc == CC_STALL || cc == CC_TRB_ERR || cc == CC_DATA_BUF_ERR) {
                post_transfer_event(taddr, cc, len - done, slot_id, dci, false);
                ep_halt(s, slot_id, dci);
                if (cc == CC_STALL)
                    s->have_setup = false;
                /* Skip the rest of this TD */
                while (chain && ring_peek(r, &t, &taddr)) {
                    chain = t.control & TRB_CH;
                    ring_advance(r);
                }
                return false;
            }
            if ((t.control & TRB_IOC) || (cc == CC_SHORT_PACKET && (t.control & TRB_ISP)))
                post_transfer_event(taddr, cc, len - done, slot_id, dci, false);
            if (short_pkt)
                skipping = true;
        }

        if (!chain)
            break;
    }
    return true;
}

static void run_endpoint(u32 slot_id, u32 dci)
{
    struct slot *s = &hc.slots[slot_id];
    struct endpoint *ep = &s->ep[dci];

    if (s->state == SLOT_DISABLED || ep->state == EP_DISABLED || ep->state == EP_HALTED)
        return;
    ep->state = EP_RUNNING;
    for (int i = 0; i < 256; i++)
        if (!run_td(s, slot_id, dci))
            break;
}

/* Complete waiting interrupt transfers once reports are queued */
static void kick_interrupt_endpoints(void)
{
    for (u32 slot_id = 1; slot_id <= MAX_SLOTS; slot_id++) {
        struct slot *s = &hc.slots[slot_id];
        if (s->state != SLOT_CONFIGURED)
            continue;
        if (!rq_empty(&usbdev.rq[0]))
            run_endpoint(slot_id, DCI_IN(KBD_EP));
        if (!rq_empty(&usbdev.rq[1]))
            run_endpoint(slot_id, DCI_IN(MOUSE_EP));
    }
}

/* ------------------------------------------------------------------------------------------ */
/* Contexts                                                                                     */

static void write_ep_ctx(struct slot *s, u32 dci)
{
    if (!s->out_ctx)
        return;
    struct endpoint *ep = &s->ep[dci];
    u64 ctx = s->out_ctx + dci * 32;
    u32 dw0 = guest_read32(ctx);
    dw0 = (dw0 & ~7u) | ep->state;
    guest_write32(ctx, dw0);
    u64 deq = (ep->ring.deq & ~0xfULL) | (ep->ring.ccs ? 1 : 0);
    guest_write(ctx + 8, &deq, 8);
}

static void write_slot_ctx(struct slot *s)
{
    if (!s->out_ctx)
        return;
    u32 dw3 = guest_read32(s->out_ctx + 12);
    dw3 = (dw3 & ~(0xffu | GENMASK(31, 27))) | s->addr | ((u32)s->state << 27);
    guest_write32(s->out_ctx + 12, dw3);
}

static void load_ep_from_input(struct slot *s, u64 in_ctx, u32 dci)
{
    struct endpoint *ep = &s->ep[dci];
    u64 ictx = in_ctx + 32 * (dci + 1);
    u32 dw0 = guest_read32(ictx);
    u32 dw1 = guest_read32(ictx + 4);
    u64 deq = guest_read64(ictx + 8);
    ep->type = FIELD_GET(GENMASK(5, 3), dw1);
    ep->mps = dw1 >> 16;
    ep->interval = FIELD_GET(GENMASK(23, 16), dw0);
    ep->ring.deq = deq & ~0xfULL;
    ep->ring.ccs = deq & 1;
    ep->state = EP_RUNNING;
    /* Copy the whole input EP context to the output one, then fix up the state */
    u8 tmp[32];
    if (guest_read(ictx, tmp, 32) && s->out_ctx)
        guest_write(s->out_ctx + dci * 32, tmp, 32);
    write_ep_ctx(s, dci);
    xhci_dbg("slot %ld ep dci %u type %u mps %u ring 0x%lx\n", s - hc.slots, dci, ep->type, ep->mps,
             ep->ring.deq);
}

static void slot_reset_eps(struct slot *s, bool keep_ep0)
{
    for (u32 dci = keep_ep0 ? 2 : 1; dci < 32; dci++) {
        s->ep[dci].state = EP_DISABLED;
        if (s->out_ctx)
            guest_write32(s->out_ctx + dci * 32, guest_read32(s->out_ctx + dci * 32) & ~7u);
    }
}

/* ------------------------------------------------------------------------------------------ */
/* Commands                                                                                     */

static void run_command(struct trb *t, u64 taddr)
{
    u8 type = TRB_TYPE_OF(t);
    u32 slot_id = FIELD_GET(TRB_SLOT_ID, t->control);
    u32 cc = CC_SUCCESS;
    u32 param = 0;
    struct slot *s = slot_id >= 1 && slot_id <= MAX_SLOTS ? &hc.slots[slot_id] : NULL;

    hc.cmds++;
    xhci_dbg("cmd type %u slot %u ptr 0x%lx\n", type, slot_id, t->ptr);

    switch (type) {
    case TRB_ENABLE_SLOT: {
        slot_id = 0;
        for (u32 i = 1; i <= MAX_SLOTS; i++) {
            if (hc.slots[i].state == SLOT_DISABLED) {
                slot_id = i;
                break;
            }
        }
        if (!slot_id) {
            cc = CC_NO_SLOTS;
            break;
        }
        s = &hc.slots[slot_id];
        memset(s, 0, sizeof(*s));
        s->state = SLOT_DEFAULT; /* "enabled"; Default once addressed with BSR */
        s->out_ctx = 0;
        break;
    }

    case TRB_DISABLE_SLOT:
        if (!s || s->state == SLOT_DISABLED) {
            cc = CC_SLOT_NOT_ENABLED;
            break;
        }
        slot_reset_eps(s, false);
        s->state = SLOT_DISABLED;
        write_slot_ctx(s);
        memset(s, 0, sizeof(*s));
        break;

    case TRB_ADDRESS_DEV: {
        if (!s || s->state == SLOT_DISABLED) {
            cc = CC_SLOT_NOT_ENABLED;
            break;
        }
        u64 in_ctx = t->ptr & ~0xfULL;
        if (!hc.dcbaap || !guest_range_ok(in_ctx, 96)) {
            cc = CC_PARAMETER_ERR;
            break;
        }
        s->out_ctx = guest_read64(hc.dcbaap + slot_id * 8) & ~0x3fULL;
        if (!guest_range_ok(s->out_ctx, 32 * 32)) {
            printf("xhci: bad output context 0x%lx for slot %u\n", s->out_ctx, slot_id);
            s->out_ctx = 0;
            cc = CC_PARAMETER_ERR;
            break;
        }
        /* slot context: copy input -> output */
        u8 tmp[32];
        guest_read(in_ctx + 32, tmp, 32);
        guest_write(s->out_ctx, tmp, 32);
        s->port = FIELD_GET(GENMASK(23, 16), guest_read32(in_ctx + 32 + 4));
        if (s->port != 1 || !(hc.portsc & PORTSC_PED)) {
            cc = CC_USB_TRANS_ERR;
            break;
        }
        load_ep_from_input(s, in_ctx, 1);
        if (t->control & TRB_BSR) {
            s->state = SLOT_DEFAULT;
            s->addr = 0;
        } else {
            s->state = SLOT_ADDRESSED;
            s->addr = slot_id;
            /* a new address resets the device's control state */
            usbdev.config = 0;
            usbdev.protocol[0] = usbdev.protocol[1] = 0;
            usbdev.idle[0] = usbdev.idle[1] = 0;
        }
        s->have_setup = false;
        write_slot_ctx(s);
        break;
    }

    case TRB_CONFIG_EP: {
        if (!s || s->state == SLOT_DISABLED || !s->out_ctx) {
            cc = CC_SLOT_NOT_ENABLED;
            break;
        }
        if (t->control & TRB_DC) {
            slot_reset_eps(s, true);
            s->state = SLOT_ADDRESSED;
            write_slot_ctx(s);
            break;
        }
        u64 in_ctx = t->ptr & ~0xfULL;
        if (!guest_range_ok(in_ctx, 33 * 32)) {
            cc = CC_PARAMETER_ERR;
            break;
        }
        u32 drop = guest_read32(in_ctx);
        u32 add = guest_read32(in_ctx + 4);
        for (u32 dci = 2; dci < 32; dci++) {
            if (drop & BIT(dci)) {
                s->ep[dci].state = EP_DISABLED;
                write_ep_ctx(s, dci);
            }
        }
        for (u32 dci = 2; dci < 32; dci++)
            if (add & BIT(dci))
                load_ep_from_input(s, in_ctx, dci);
        if (add & BIT(0)) {
            /* slot context: context entries field */
            u32 dw0 = guest_read32(in_ctx + 32);
            u32 out0 = guest_read32(s->out_ctx);
            out0 = (out0 & ~GENMASK(31, 27)) | (dw0 & GENMASK(31, 27));
            guest_write32(s->out_ctx, out0);
        }
        bool any = false;
        for (u32 dci = 2; dci < 32; dci++)
            any |= s->ep[dci].state != EP_DISABLED;
        s->state = any ? SLOT_CONFIGURED : SLOT_ADDRESSED;
        write_slot_ctx(s);
        break;
    }

    case TRB_EVAL_CTX: {
        if (!s || s->state == SLOT_DISABLED || !s->out_ctx) {
            cc = CC_SLOT_NOT_ENABLED;
            break;
        }
        u64 in_ctx = t->ptr & ~0xfULL;
        if (!guest_range_ok(in_ctx, 96)) {
            cc = CC_PARAMETER_ERR;
            break;
        }
        u32 add = guest_read32(in_ctx + 4);
        if (add & BIT(0)) {
            /* interrupter target / max exit latency live in dw1/dw2 of the slot context */
            u32 dw1 = guest_read32(in_ctx + 32 + 4);
            u32 out1 = guest_read32(s->out_ctx + 4);
            out1 = (out1 & ~0xffffu) | (dw1 & 0xffff);
            guest_write32(s->out_ctx + 4, out1);
        }
        if (add & BIT(1)) {
            u32 dw1 = guest_read32(in_ctx + 64 + 4);
            s->ep[1].mps = dw1 >> 16;
            u32 out1 = guest_read32(s->out_ctx + 32 + 4);
            out1 = (out1 & 0xffff) | (dw1 & 0xffff0000u);
            guest_write32(s->out_ctx + 32 + 4, out1);
        }
        break;
    }

    case TRB_RESET_EP: {
        u32 dci = FIELD_GET(TRB_EP_ID, t->control);
        if (!s || s->state == SLOT_DISABLED || dci < 1 || dci > 31) {
            cc = CC_SLOT_NOT_ENABLED;
            break;
        }
        struct endpoint *ep = &s->ep[dci];
        if (ep->state == EP_DISABLED) {
            cc = CC_EP_NOT_ENABLED;
            break;
        }
        if (ep->state != EP_HALTED) {
            cc = CC_CONTEXT_STATE;
            break;
        }
        ep->state = EP_STOPPED;
        if (dci == 1)
            s->have_setup = false;
        write_ep_ctx(s, dci);
        break;
    }

    case TRB_STOP_EP: {
        u32 dci = FIELD_GET(TRB_EP_ID, t->control);
        if (!s || s->state == SLOT_DISABLED || dci < 1 || dci > 31) {
            cc = CC_SLOT_NOT_ENABLED;
            break;
        }
        struct endpoint *ep = &s->ep[dci];
        if (ep->state == EP_DISABLED) {
            cc = CC_EP_NOT_ENABLED;
            break;
        }
        if (ep->state == EP_RUNNING)
            ep->state = EP_STOPPED;
        write_ep_ctx(s, dci);
        break;
    }

    case TRB_SET_TR_DEQ: {
        u32 dci = FIELD_GET(TRB_EP_ID, t->control);
        if (!s || s->state == SLOT_DISABLED || dci < 1 || dci > 31) {
            cc = CC_SLOT_NOT_ENABLED;
            break;
        }
        struct endpoint *ep = &s->ep[dci];
        if (ep->state == EP_DISABLED) {
            cc = CC_EP_NOT_ENABLED;
            break;
        }
        if (ep->state == EP_RUNNING) {
            cc = CC_CONTEXT_STATE;
            break;
        }
        ep->ring.deq = t->ptr & ~0xfULL;
        ep->ring.ccs = t->ptr & 1;
        ep->state = EP_STOPPED;
        write_ep_ctx(s, dci);
        break;
    }

    case TRB_RESET_DEV:
        if (!s || s->state == SLOT_DISABLED) {
            cc = CC_SLOT_NOT_ENABLED;
            break;
        }
        slot_reset_eps(s, true);
        s->ep[1].state = EP_DISABLED;
        s->state = SLOT_DEFAULT;
        s->addr = 0;
        s->have_setup = false;
        usbdev.config = 0;
        write_slot_ctx(s);
        break;

    case TRB_CMD_NOOP:
        break;

    case TRB_GET_PORT_BW:
    case TRB_FORCE_HDR:
    case TRB_NEG_BW:
    case TRB_SET_LAT:
        cc = CC_TRB_ERR;
        break;

    default:
        printf("xhci: unknown command TRB type %u\n", type);
        cc = CC_TRB_ERR;
        break;
    }

    post_cmd_completion(taddr, cc, param, slot_id);
}

static void run_command_ring(void)
{
    if (!(hc.usbcmd & CMD_RS))
        return;
    struct trb t;
    u64 taddr;
    hc.crcr |= CRCR_CRR;
    for (int i = 0; i < 64; i++) {
        if (!ring_peek(&hc.cmd_ring, &t, &taddr))
            break;
        ring_advance(&hc.cmd_ring);
        run_command(&t, taddr);
    }
    hc.crcr &= ~CRCR_CRR;
}

/* ------------------------------------------------------------------------------------------ */
/* Port                                                                                         */

static void port_attach(void)
{
    /* Device present, link in polling until the driver resets the port */
    hc.portsc = PORTSC_CCS | PORTSC_PP | FIELD_PREP(PORTSC_PLS, PLS_POLLING) |
                FIELD_PREP(PORTSC_SPEED, SPEED_HS) | PORTSC_CSC;
}

static void port_write(u32 val)
{
    u32 old = hc.portsc;
    u32 ps = old;

    /* W1C change bits */
    ps &= ~(val & PORTSC_CHANGE_BITS);

    if (val & PORTSC_PR || val & PORTSC_WPR) {
        /* Reset completes immediately: enabled, U0 */
        ps &= ~(PORTSC_PR | PORTSC_PLS);
        ps |= PORTSC_PED | FIELD_PREP(PORTSC_PLS, PLS_U0) | PORTSC_PRC;
        for (u32 i = 1; i <= MAX_SLOTS; i++)
            if (hc.slots[i].state != SLOT_DISABLED && hc.slots[i].port == 1)
                hc.slots[i].have_setup = false;
        usbdev.config = 0;
    }
    /*
     * Writing 1 to PED would disable the port. Drivers are supposed to mask it on read-modify-
     * write; a driver that does not would make our only device vanish, so it is ignored here.
     */

    if (val & PORTSC_LWS) {
        u32 pls = FIELD_GET(PORTSC_PLS, val);
        u32 cur = FIELD_GET(PORTSC_PLS, ps);
        if (pls == PLS_U3 && cur == PLS_U0) {
            ps = (ps & ~PORTSC_PLS) | FIELD_PREP(PORTSC_PLS, PLS_U3);
        } else if (pls == PLS_U0 && cur == PLS_U3) {
            ps = (ps & ~PORTSC_PLS) | FIELD_PREP(PORTSC_PLS, PLS_U0) | PORTSC_PLC;
        }
    }

    if (!(val & PORTSC_PP)) {
        /* port power off: keep the device "attached", it comes back when powered */
        ps &= ~PORTSC_PP;
    } else {
        ps |= PORTSC_PP;
    }

    /* wake enables: store as written */
    ps = (ps & ~(PORTSC_WCE | PORTSC_WDE | PORTSC_WOE)) | (val & (PORTSC_WCE | PORTSC_WDE | PORTSC_WOE));
    ps = (ps & ~PORTSC_PIC) | (val & PORTSC_PIC);

    hc.portsc = ps;
    if ((ps & PORTSC_CHANGE_BITS) & ~(old & PORTSC_CHANGE_BITS))
        post_port_status(1);
}

/* ------------------------------------------------------------------------------------------ */
/* Controller reset / run                                                                       */

static void hc_reset(void)
{
    hc.usbcmd = 0;
    hc.usbsts = STS_HCH;
    hc.dnctrl = 0;
    hc.config = 0;
    hc.crcr = 0;
    hc.dcbaap = 0;
    memset(&hc.cmd_ring, 0, sizeof(hc.cmd_ring));
    hc.iman = 0;
    hc.imod = 0;
    hc.erstsz = 0;
    hc.erstba = 0;
    hc.erdp = 0;
    memset(&hc.evt, 0, sizeof(hc.evt));
    hc.irq_level = false;
    hc.irq_injected = false;
    memset(hc.slots, 0, sizeof(hc.slots));
    memset(&usbdev, 0, sizeof(usbdev));
    port_attach();
}

static u32 mfindex(void)
{
    u64 freq = mrs(CNTFRQ_EL0);
    return (u32)((get_ticks() * 8000 / freq) & 0x3fff);
}

/* ------------------------------------------------------------------------------------------ */
/* MMIO                                                                                         */

static u32 reg_read(u32 off)
{
    /* Capability registers */
    switch (off) {
    case 0x00:
        return (0x0100 << 16) | CAP_LEN; /* HCIVERSION 1.0, CAPLENGTH */
    case 0x04: /* HCSPARAMS1 */
        return (MAX_PORTS << 24) | (1 << 8) | MAX_SLOTS;
    case 0x08: /* HCSPARAMS2: IST=1, ERST max 2^4, no scratchpad */
        return (ERST_MAX << 4) | 1;
    case 0x0c: /* HCSPARAMS3 */
        return 0;
    case 0x10: /* HCCPARAMS1: AC64, xECP */
        return ((XECP_BASE / 4) << 16) | BIT(0);
    case 0x14: /* DBOFF */
        return DB_BASE;
    case 0x18: /* RTSOFF */
        return RT_BASE;
    case 0x1c: /* HCCPARAMS2 */
        return 0;
    /* Extended capability: supported protocol USB 2.0, port 1, count 1 */
    case XECP_BASE + 0x0:
        return (0x02 << 24) | (0x00 << 16) | (0 << 8) | 0x02;
    case XECP_BASE + 0x4:
        return 0x20425355; /* "USB " */
    case XECP_BASE + 0x8:
        return (1 << 8) | 1;
    case XECP_BASE + 0xc:
        return 0;
    /* Operational */
    case OP_BASE + 0x00:
        return hc.usbcmd;
    case OP_BASE + 0x04:
        return hc.usbsts;
    case OP_BASE + 0x08:
        return 1; /* PAGESIZE: 4 KB */
    case OP_BASE + 0x14:
        return hc.dnctrl;
    case OP_BASE + 0x18:
        return hc.crcr & (CRCR_CRR); /* pointer reads as zero */
    case OP_BASE + 0x1c:
        return 0;
    case OP_BASE + 0x30:
        return (u32)hc.dcbaap;
    case OP_BASE + 0x34:
        return (u32)(hc.dcbaap >> 32);
    case OP_BASE + 0x38:
        return hc.config;
    case PORT_BASE + 0x0:
        return hc.portsc;
    case PORT_BASE + 0x4: /* PORTPMSC */
    case PORT_BASE + 0x8: /* PORTLI */
    case PORT_BASE + 0xc:
        return 0;
    /* Runtime */
    case RT_BASE + 0x0:
        return mfindex();
    case IR0_BASE + 0x00:
        return hc.iman;
    case IR0_BASE + 0x04:
        return hc.imod;
    case IR0_BASE + 0x08:
        return hc.erstsz;
    case IR0_BASE + 0x10:
        return (u32)hc.erstba;
    case IR0_BASE + 0x14:
        return (u32)(hc.erstba >> 32);
    case IR0_BASE + 0x18:
        return (u32)hc.erdp;
    case IR0_BASE + 0x1c:
        return (u32)(hc.erdp >> 32);
    default:
        if (off >= DB_BASE && off < DB_BASE + 4 * (MAX_SLOTS + 1))
            return 0;
        xhci_dbg("read of unknown register 0x%x\n", off);
        return 0;
    }
}

static void reg_write(u32 off, u32 val)
{
    switch (off) {
    case OP_BASE + 0x00: { /* USBCMD */
        u32 old = hc.usbcmd;
        if (val & CMD_HCRST) {
            xhci_dbg("host controller reset\n");
            hc_reset();
            return;
        }
        hc.usbcmd = val & (CMD_RS | CMD_INTE | CMD_HSEE | CMD_EWE | CMD_EU3S);
        if ((val & CMD_RS) && !(old & CMD_RS)) {
            hc.usbsts &= ~STS_HCH;
            if (!hc.evt.valid)
                evt_reset();
            xhci_dbg("run (dcbaap 0x%lx crcr 0x%lx erstba 0x%lx)\n", hc.dcbaap, hc.cmd_ring.deq,
                     hc.erstba);
            /* The device is attached: tell the driver about the port */
            if (hc.portsc & PORTSC_CSC)
                post_port_status(1);
        } else if (!(val & CMD_RS) && (old & CMD_RS)) {
            hc.usbsts |= STS_HCH;
            hc.crcr &= ~CRCR_CRR;
        }
        if (val & CMD_CSS)
            hc.usbsts &= ~(STS_SSS | STS_SRE);
        if (val & CMD_CRS)
            hc.usbsts &= ~(STS_RSS | STS_SRE);
        update_irq();
        break;
    }
    case OP_BASE + 0x04: /* USBSTS: W1C */
        hc.usbsts &= ~(val & (STS_HSE | STS_EINT | STS_PCD | STS_SRE));
        update_irq();
        break;
    case OP_BASE + 0x14:
        hc.dnctrl = val & 0xffff;
        break;
    case OP_BASE + 0x18: /* CRCR lo */
        if (hc.crcr & CRCR_CRR) {
            if (val & (CRCR_CS | CRCR_CA)) {
                hc.crcr &= ~CRCR_CRR;
                post_event(hc.cmd_ring.deq, FIELD_PREP(TRB_CC, CC_CMD_RING_STOPPED),
                           FIELD_PREP(TRB_TYPE, TRB_CMD_COMPLETE));
            }
            break;
        }
        hc.crcr = (hc.crcr & 0xffffffff00000000ULL) | (val & ~0x3fu);
        hc.cmd_ring.deq = hc.crcr & ~0x3fULL;
        hc.cmd_ring.ccs = val & CRCR_RCS;
        break;
    case OP_BASE + 0x1c: /* CRCR hi */
        hc.crcr = (hc.crcr & 0xffffffffULL) | ((u64)val << 32);
        hc.cmd_ring.deq = hc.crcr & ~0x3fULL;
        break;
    case OP_BASE + 0x30:
        hc.dcbaap = (hc.dcbaap & 0xffffffff00000000ULL) | (val & ~0x3fu);
        break;
    case OP_BASE + 0x34:
        hc.dcbaap = (hc.dcbaap & 0xffffffffULL) | ((u64)val << 32);
        break;
    case OP_BASE + 0x38:
        hc.config = val & 0xff;
        break;
    case PORT_BASE + 0x0:
        port_write(val);
        break;
    case PORT_BASE + 0x4:
    case PORT_BASE + 0x8:
    case PORT_BASE + 0xc:
        break;
    case IR0_BASE + 0x00: /* IMAN: IP is W1C, IE RW */
        if (val & IMAN_IP)
            hc.iman &= ~IMAN_IP;
        hc.iman = (hc.iman & ~IMAN_IE) | (val & IMAN_IE);
        update_irq();
        break;
    case IR0_BASE + 0x04:
        hc.imod = val;
        break;
    case IR0_BASE + 0x08:
        hc.erstsz = val & 0xffff;
        break;
    case IR0_BASE + 0x10:
        hc.erstba = (hc.erstba & 0xffffffff00000000ULL) | (val & ~0x3fu);
        break;
    case IR0_BASE + 0x14:
        hc.erstba = (hc.erstba & 0xffffffffULL) | ((u64)val << 32);
        evt_reset();
        break;
    case IR0_BASE + 0x18: /* ERDP lo: EHB is W1C */
        hc.erdp = (hc.erdp & 0xffffffff00000000ULL) | (val & ~0x8u);
        break;
    case IR0_BASE + 0x1c:
        hc.erdp = (hc.erdp & 0xffffffffULL) | ((u64)val << 32);
        break;
    default:
        if (off >= DB_BASE && off < DB_BASE + 4 * (MAX_SLOTS + 1)) {
            u32 db = (off - DB_BASE) / 4;
            if (db == 0) {
                run_command_ring();
            } else {
                u32 dci = val & 0xff;
                if (dci >= 1 && dci < 32)
                    run_endpoint(db, dci);
            }
            break;
        }
        xhci_dbg("write of unknown register 0x%x = 0x%x\n", off, val);
        break;
    }
}

static bool xhci_hook(struct exc_info *ctx, u64 addr, u64 *val, bool write, int width)
{
    UNUSED(ctx);
    int win = 0;
    for (int w = 0; w < HV_XHCI_NWIN; w++)
        if (addr >= hv_xhci_win_base[w] && addr < hv_xhci_win_base[w] + HV_XHCI_SIZE)
            win = w;
    u32 off = addr - hv_xhci_win_base[win];
    hc.win_access[win]++;

    spin_lock(&hc.lock);
    if (width <= 2) {
        /* byte/half/word accesses: emulate on the containing 32-bit register */
        u32 shift = (off & 3) * 8;
        u32 mask = width == 2 ? 0xffffffffu : width == 1 ? 0xffffu << shift : 0xffu << shift;
        off &= ~3u;
        if (write) {
            u32 cur = reg_read(off);
            u32 nv = (cur & ~mask) | (((u32)*val << shift) & mask);
            if (width == 2)
                nv = (u32)*val;
            hc.mmio_wr++;
            trace_access(off, nv, true);
            reg_write(off, nv);
        } else {
            u32 v = reg_read(off);
            hc.mmio_rd++;
            trace_access(off, v, false);
            *val = (v & mask) >> shift;
        }
    } else {
        /* 64-bit (or wider) accesses as consecutive 32-bit registers */
        u32 words = (1u << width) / 4;
        for (u32 i = 0; i < words; i++) {
            u32 o = (off & ~7u) + i * 4;
            if (write) {
                u32 v = (u32)(val[i / 2] >> ((i % 2) * 32));
                hc.mmio_wr++;
                trace_access(o, v, true);
                reg_write(o, v);
            } else {
                u64 v = reg_read(o);
                hc.mmio_rd++;
                trace_access(o, v, false);
                if (i % 2 == 0)
                    val[i / 2] = v;
                else
                    val[i / 2] |= v << 32;
            }
        }
    }
    spin_unlock(&hc.lock);
    return true;
}

/* ------------------------------------------------------------------------------------------ */
/* Public                                                                                       */

void hv_usbhid_push(int iface, const u8 *report, u32 len)
{
    if (iface < 0 || iface > 1)
        return;
    spin_lock(&hc.lock);
    rq_push(&usbdev.rq[iface], report, len);
    kick_interrupt_endpoints();
    spin_unlock(&hc.lock);
}

u8 hv_usbhid_leds(void)
{
    return usbdev.leds;
}

void hv_xhci_tick(void)
{
    if (!hc.inited)
        return;
    spin_lock(&hc.lock);
    kick_interrupt_endpoints();
    /* A level interrupt the guest had masked at the distributor when it was raised */
    if (hc.irq_level && !hc.irq_injected)
        update_irq();
    spin_unlock(&hc.lock);
}

static const char *slot_state_name(u8 st)
{
    static const char *const n[] = {"disabled", "default", "addressed", "configured"};
    return st < 4 ? n[st] : "?";
}

void hv_xhci_dump(void)
{
    if (!hc.inited) {
        printf("xhci: not initialised\n");
        return;
    }
    printf("xhci: cmd %08x sts %08x crcr %lx dcbaap %lx cfg %u port %08x\n", hc.usbcmd, hc.usbsts,
           hc.cmd_ring.deq, hc.dcbaap, hc.config, hc.portsc);
    printf("xhci: iman %x erstsz %u erstba %lx erdp %lx evt seg %lx/%u enq %u ccs %d\n", hc.iman,
           hc.erstsz, hc.erstba, hc.erdp, hc.evt.seg_base, hc.evt.seg_size, hc.evt.enq,
           hc.evt.ccs);
    printf("xhci: mmio rd %u wr %u, cmds %u, events %u, xfers %u, irqs %u, level %d inj %d\n", hc.mmio_rd,
           hc.mmio_wr, hc.cmds, hc.events, hc.xfers, hc.irqs, hc.irq_level, hc.irq_injected);
    printf("xhci: windows:");
    for (int w = 0; w < HV_XHCI_NWIN; w++)
        printf(" %s@0x%lx: %u accesses, spi%u %s;", hv_xhci_win_name[w], hv_xhci_win_base[w],
               hc.win_access[w], hv_xhci_win_irq[w], hv_vgic3_spi_enabled(hv_xhci_win_irq[w]) ? "enabled" : "masked");
    printf("\n");
    for (u32 i = 1; i <= MAX_SLOTS; i++) {
        struct slot *s = &hc.slots[i];
        if (s->state == SLOT_DISABLED)
            continue;
        printf("xhci: slot %u %s addr %u ep0 st %u ring %lx | kbd ep st %u ring %lx | mouse ep st %u ring %lx\n",
               i, slot_state_name(s->state), s->addr, s->ep[1].state, s->ep[1].ring.deq,
               s->ep[DCI_IN(KBD_EP)].state, s->ep[DCI_IN(KBD_EP)].ring.deq,
               s->ep[DCI_IN(MOUSE_EP)].state, s->ep[DCI_IN(MOUSE_EP)].ring.deq);
    }
    printf("xhci: usb config %u proto %u/%u idle %u/%u leds %02x, queued kbd %u mouse %u\n",
           usbdev.config, usbdev.protocol[0], usbdev.protocol[1], usbdev.idle[0], usbdev.idle[1],
           usbdev.leds, (usbdev.rq[0].tail - usbdev.rq[0].head) % RQ_SIZE,
           (usbdev.rq[1].tail - usbdev.rq[1].head) % RQ_SIZE);
    u32 ne = min(hc.evlog_n, (u32)ARRAY_SIZE(hc.evlog));
    printf("xhci: last %u events of %u (type/slot/ep cc len trb-done/len ptr):", ne, hc.evlog_n);
    for (u32 k = 0; k < ne; k++) {
        u32 i = (hc.evlog_n - ne + k) % ARRAY_SIZE(hc.evlog);
        printf("%s%lu/%lu/%lu cc%lu len%u trb%u/%u %lx", k % 3 ? "  " : "\n  ",
               FIELD_GET(TRB_TYPE, hc.evlog[i].control), FIELD_GET(TRB_SLOT_ID, hc.evlog[i].control),
               FIELD_GET(TRB_EP_ID, hc.evlog[i].control), FIELD_GET(TRB_CC, hc.evlog[i].status),
               hc.evlog[i].status & 0xffffff, hc.evlog[i].trb_done, hc.evlog[i].trb_len, hc.evlog[i].ptr);
    }
    printf("\n");
    u32 ns = min(hc.setuplog_n, (u32)ARRAY_SIZE(hc.setuplog));
    printf("xhci: last %u setup packets of %u (bmRequestType bRequest wValue wIndex wLength):", ns, hc.setuplog_n);
    for (u32 k = 0; k < ns; k++) {
        u8 *b = hc.setuplog[(hc.setuplog_n - ns + k) % ARRAY_SIZE(hc.setuplog)];
        printf("%s%02x %02x %04x %04x %04x", k % 4 ? "  |  " : "\n  ", b[0], b[1], b[2] | b[3] << 8, b[4] | b[5] << 8,
               b[6] | b[7] << 8);
    }
    printf("\n");
    u32 n = min(hc.trace_n, (u32)ARRAY_SIZE(hc.trace));
    printf("xhci: last %u register accesses (of %u):", n, hc.trace_n);
    for (u32 k = 0; k < n; k++) {
        u32 i = (hc.trace_n - n + k) % ARRAY_SIZE(hc.trace);
        printf("%s%s%x=%x", k % 6 ? " " : "\n  ", hc.trace[i].write ? "W" : "R", hc.trace[i].off,
               hc.trace[i].val);
    }
    printf("\n");
}

int hv_xhci_init(u64 guest_base, u64 guest_end)
{
    spin_init(&hc.lock);
    build_cfg_desc();
    hc.ram_base = guest_base;
    hc.ram_top = guest_end;
    hc_reset();

    for (int w = 0; w < HV_XHCI_NWIN; w++) {
        if (hv_map_hook(hv_xhci_win_base[w], xhci_hook, HV_XHCI_SIZE) < 0) {
            printf("xhci: failed to map MMIO hook for window %d\n", w);
            return -1;
        }
    }
    hc.inited = true;
    printf("xhci: emulated controller at 0x%lx, IRQ %u (aliases:", (u64)HV_XHCI_BASE, HV_XHCI_IRQ);
    for (int w = 1; w < HV_XHCI_NWIN; w++)
        printf(" 0x%lx/%u", hv_xhci_win_base[w], hv_xhci_win_irq[w]);
    printf(")\n");
    return 0;
}
