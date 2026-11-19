/* SPDX-License-Identifier: MIT */

/*
 * Apple SPI HID transport: the internal keyboard and trackpad of the 2021+ MacBooks ("ipd" node
 * under the spi3 controller in the ADT).
 *
 * Written from the protocol description in the Asahi Linux drivers (drivers/spi/spi-apple.c for
 * the controller, drivers/hid/spi-hid/spi-hid-apple-*.c for the transport, which in turn documents
 * the reverse engineered applespi.c protocol) and m1n1's own register maps
 * (proxyclient/m1n1/hw/spi.py, proxyclient/m1n1/trace/gpio.py). Polled, no DMA, no interrupts:
 * the EL2 hypervisor calls spihid_poll() from its tick.
 *
 * Wire protocol, in short:
 *   - every transfer is a 256 byte packet: {flags, device, offset, remain, length, data[246], crc16}
 *     flags 0x40 = write (host -> device, and the device's answers to it), 0x20 = read (input
 *     reports); long messages are split over several packets (offset/remain).
 *   - the packet data is a message: {u0, u1, u2, id, rsplen, length} + payload + crc16
 *   - after a write packet the device answers 4 status bytes (ac 27 68 d5) in the same chip select.
 *   - the device pulls its interrupt line low when it has a packet to read; a read is a 256 byte
 *     receive-only transfer.
 *   - power up: pulse the enable GPIO; the device then sends a 4 byte "booted" packet (a0 80 00 00).
 */

#include "spihid.h"
#include "adt.h"
#include "pmgr.h"
#include "string.h"
#include "utils.h"

/* ------------------------------------------------------------------------------------------ */
/* Apple SPI controller (PIO)                                                                   */

#define SPI_CTRL          0x000
#define SPI_CTRL_RUN      BIT(0)
#define SPI_CTRL_TX_RESET BIT(2)
#define SPI_CTRL_RX_RESET BIT(3)

#define SPI_CFG              0x004
#define SPI_CFG_CPHA         BIT(1)
#define SPI_CFG_CPOL         BIT(2)
#define SPI_CFG_MODE         GENMASK(6, 5)
#define SPI_CFG_MODE_POLLED  0
#define SPI_CFG_MODE_IRQ     1
#define SPI_CFG_LSB_FIRST    BIT(13)
#define SPI_CFG_WORD_SIZE    GENMASK(16, 15)
#define SPI_CFG_WORD_SIZE_8B 0
#define SPI_CFG_FIFO_THRESH  GENMASK(18, 17)

#define SPI_STATUS 0x008

#define SPI_PIN           0x00c
#define SPI_PIN_KEEP_MOSI BIT(0)
#define SPI_PIN_CS        BIT(1)

#define SPI_TXDATA     0x010
#define SPI_RXDATA     0x020
#define SPI_CLKDIV     0x030
#define SPI_CLKDIV_MAX 0x7ff
#define SPI_RXCNT      0x034
#define SPI_WORD_DELAY 0x038
#define SPI_TXCNT      0x04c

#define SPI_FIFOSTAT          0x10c
#define SPI_FIFOSTAT_TXFULL   BIT(4)
#define SPI_FIFOSTAT_LEVEL_TX GENMASK(15, 8)
#define SPI_FIFOSTAT_RXEMPTY  BIT(20)
#define SPI_FIFOSTAT_LEVEL_RX GENMASK(31, 24)

#define SPI_IE_XFER          0x130
#define SPI_IF_XFER          0x134
#define SPI_XFER_RXCOMPLETE  BIT(0)
#define SPI_XFER_TXCOMPLETE  BIT(1)

#define SPI_IE_FIFO           0x138
#define SPI_IF_FIFO           0x13c
#define SPI_FIFO_RXTHRESH     BIT(4)
#define SPI_FIFO_TXTHRESH     BIT(5)
#define SPI_FIFO_RXFULL       BIT(8)
#define SPI_FIFO_TXEMPTY      BIT(9)
#define SPI_FIFO_RXUNDERRUN   BIT(16)
#define SPI_FIFO_TXOVERFLOW   BIT(17)

#define SPI_SHIFTCFG             0x150
#define SPI_SHIFTCFG_CLK_ENABLE  BIT(0)
#define SPI_SHIFTCFG_CS_ENABLE   BIT(1)
#define SPI_SHIFTCFG_TX_ENABLE   BIT(10)
#define SPI_SHIFTCFG_RX_ENABLE   BIT(11)
#define SPI_SHIFTCFG_BITS        GENMASK(21, 16)
#define SPI_SHIFTCFG_OVERRIDE_CS BIT(24)

#define SPI_PINCFG             0x154
#define SPI_PINCFG_KEEP_CLK    BIT(0)
#define SPI_PINCFG_KEEP_CS     BIT(1)
#define SPI_PINCFG_KEEP_MOSI   BIT(2)
#define SPI_PINCFG_CS_IDLE_VAL BIT(9)

#define SPI_DELAY_PRE  0x160
#define SPI_DELAY_POST 0x168

#define SPI_FIFO_DEPTH 16
#define SPI_REFCLK_HZ  24000000
#define SPI_HID_HZ     8000000

/* Chip select timing: Apple's ADT says 20 us, the HID metadata 45 us; Linux adds both */
#define CS_SETUP_US    65
#define CS_HOLD_US     65
#define CS_INACTIVE_US 250
#define RW_CHG_US      200 /* between a write packet and reading its status bytes */

/* ------------------------------------------------------------------------------------------ */
/* Apple GPIO (pinctrl) registers                                                               */

#define GPIO_PIN(n)         (4 * (n))
#define GPIO_DATA           BIT(0)
#define GPIO_MODE           GENMASK(3, 1)
#define GPIO_MODE_OUT       1
#define GPIO_MODE_IN_IRQ_LO 3
#define GPIO_MODE_IN_OFF    7
#define GPIO_PERIPH         GENMASK(6, 5)
#define GPIO_PULL           GENMASK(8, 7)
#define GPIO_INPUT_ENABLE   BIT(9)

/* ------------------------------------------------------------------------------------------ */
/* Transport protocol                                                                           */

#define PKT_READ  0x20
#define PKT_WRITE 0x40
#define PKT_SIZE  256

struct spihid_pkt {
    u8 flags;
    u8 device;
    u16 offset;
    u16 remain;
    u16 length;
    u8 data[246];
    u16 crc16;
} PACKED;

struct spihid_msg_hdr {
    u8 u0; /* request type: 0x10 input report, 0x20 info/descriptor, 0x32 get, 0x51/0x52 set */
    u8 u1; /* report / sub-type */
    u8 u2; /* device number for info requests */
    u8 id;
    u16 rsplen;
    u16 length;
} PACKED;

struct spihid_device_info {
    u16 u0[2];
    u16 num_devices;
    u16 vendor_id;
    u16 product_id;
    u16 version_number;
    u16 vendor_str[2];
    u16 product_str[2];
    u16 serial_str[2];
} PACKED;

struct spihid_iface_info {
    u8 u_0;
    u8 interface_num;
    u8 u_2;
    u8 u_3;
    u8 u_4;
    u8 country_code;
    u16 max_input_report_len;
    u16 max_output_report_len;
    u16 max_control_report_len;
    u16 name_offset;
    u16 name_length;
} PACKED;

static const u8 msg_booted[4] = {0xa0, 0x80, 0x00, 0x00};
static const u8 msg_status_ok[4] = {0xac, 0x27, 0x68, 0xd5};

static struct {
    bool present;
    bool ready;
    u64 spi_base;
    u64 en_gpio_base;
    u32 en_pin;
    u64 irq_gpio_base;
    u32 irq_pin;
    char spi_path[64];
    char hid_path[80];

    u8 msg_id;
    bool booted;
    u32 num_devices;
    u16 vendor_id, product_id, version;
    char vendor[32], product[64], serial[32];
    struct spihid_iface iface[SPIHID_MAX_DEVICES];

    /* multi-packet message reassembly */
    u8 asm_buf[2048];
    u32 asm_off, asm_len;
    u8 asm_dev, asm_flags;

    /* reply to a get-report request */
    u8 reply_buf[SPIHID_DESC_MAX];
    u32 reply_len;
    u8 reply_dev;

    u8 tx_buf[PKT_SIZE] ALIGNED(64);
    u8 rx_buf[PKT_SIZE] ALIGNED(64);
    u8 status_buf[4];

    spihid_report_cb_t cb;
    u32 reports, bad_crc, short_pkts, empty_pkts;
} st;

/* ------------------------------------------------------------------------------------------ */

/* CRC-16/ARC (poly 0x8005 reflected), what lib/crc16.c computes */
static u16 crc16(u16 crc, const u8 *p, size_t len)
{
    while (len--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xA001 : 0);
    }
    return crc;
}

static inline u32 spi_read(u32 off)
{
    return read32(st.spi_base + off);
}

static inline void spi_write(u32 off, u32 val)
{
    write32(st.spi_base + off, val);
}

static inline void spi_mask(u32 off, u32 clear, u32 set)
{
    spi_write(off, (spi_read(off) & ~clear) | set);
}

static void gpio_rmw(u64 base, u32 pin, u32 clear, u32 set)
{
    u64 reg = base + GPIO_PIN(pin);
    write32(reg, (read32(reg) & ~clear) | set);
}

static void gpio_set_output(u64 base, u32 pin, bool val)
{
    gpio_rmw(base, pin, GPIO_PERIPH | GPIO_MODE | GPIO_DATA,
             FIELD_PREP(GPIO_MODE, GPIO_MODE_OUT) | (val ? GPIO_DATA : 0));
}

static void gpio_set_input(u64 base, u32 pin)
{
    gpio_rmw(base, pin, GPIO_PERIPH | GPIO_MODE | GPIO_DATA | GPIO_INPUT_ENABLE,
             FIELD_PREP(GPIO_MODE, GPIO_MODE_IN_OFF) | GPIO_INPUT_ENABLE);
}

static bool gpio_get(u64 base, u32 pin)
{
    return read32(base + GPIO_PIN(pin)) & GPIO_DATA;
}

static void gpio_pinmux(u64 base, u32 pin, u32 func)
{
    gpio_rmw(base, pin, GPIO_PERIPH | GPIO_INPUT_ENABLE,
             FIELD_PREP(GPIO_PERIPH, func) | GPIO_INPUT_ENABLE);
}

static bool irq_asserted(void)
{
    return !gpio_get(st.irq_gpio_base, st.irq_pin); /* level low */
}

/* ------------------------------------------------------------------------------------------ */
/* SPI controller                                                                               */

static void spi_init(void)
{
    /* CS high (inactive), driven from the PIN register */
    spi_write(SPI_PIN, SPI_PIN_CS);
    spi_mask(SPI_SHIFTCFG, SPI_SHIFTCFG_OVERRIDE_CS, 0);
    spi_mask(SPI_PINCFG, SPI_PINCFG_CS_IDLE_VAL, SPI_PINCFG_KEEP_CS);

    spi_write(SPI_CTRL, SPI_CTRL_RX_RESET | SPI_CTRL_TX_RESET);

    /* mode 0, MSB first, 8 bit words, polled */
    spi_write(SPI_CFG, FIELD_PREP(SPI_CFG_FIFO_THRESH, 0) |
                           FIELD_PREP(SPI_CFG_MODE, SPI_CFG_MODE_POLLED) |
                           FIELD_PREP(SPI_CFG_WORD_SIZE, SPI_CFG_WORD_SIZE_8B));
    spi_write(SPI_IE_FIFO, 0);
    spi_write(SPI_IE_XFER, 0);
    spi_write(SPI_DELAY_PRE, 0);
    spi_write(SPI_DELAY_POST, 0);

    spi_write(SPI_CLKDIV, (SPI_REFCLK_HZ + SPI_HID_HZ - 1) / SPI_HID_HZ);
    spi_mask(SPI_SHIFTCFG, SPI_SHIFTCFG_BITS, FIELD_PREP(SPI_SHIFTCFG_BITS, 8));
}

static void spi_cs(bool active)
{
    if (active) {
        spi_mask(SPI_PIN, SPI_PIN_CS, 0);
        udelay(CS_SETUP_US);
    } else {
        udelay(CS_HOLD_US);
        spi_mask(SPI_PIN, SPI_PIN_CS, SPI_PIN_CS);
        udelay(CS_INACTIVE_US);
    }
}

static void spi_fill_tx(const u8 **tx, u32 *left)
{
    if (!*tx || !*left)
        return;
    u32 inuse = FIELD_GET(SPI_FIFOSTAT_LEVEL_TX, spi_read(SPI_FIFOSTAT));
    u32 words = min(*left, SPI_FIFO_DEPTH - inuse);
    *left -= words;
    while (words--)
        spi_write(SPI_TXDATA, *(*tx)++);
}

static void spi_drain_rx(u8 **rx, u32 *left)
{
    if (!*rx)
        return;
    u32 words = FIELD_GET(SPI_FIFOSTAT_LEVEL_RX, spi_read(SPI_FIFOSTAT));
    while (words--) {
        u8 b = spi_read(SPI_RXDATA);
        if (*left) {
            *(*rx)++ = b;
            (*left)--;
        }
    }
}

/* One transfer (CS handled by the caller). tx or rx may be NULL. */
static int spi_xfer(const u8 *tx, u8 *rx, u32 len)
{
    u32 tx_left = tx ? len : 0;
    u32 rx_left = rx ? len : 0;
    u32 want = (tx ? SPI_XFER_TXCOMPLETE : 0) | (rx ? SPI_XFER_RXCOMPLETE : 0);

    spi_write(SPI_CTRL, SPI_CTRL_RX_RESET | SPI_CTRL_TX_RESET);
    spi_write(SPI_IF_XFER, ~0u);
    spi_write(SPI_IF_FIFO, ~0u);
    spi_write(SPI_TXCNT, tx_left);
    spi_write(SPI_RXCNT, rx_left);

    spi_fill_tx(&tx, &tx_left);
    spi_write(SPI_CTRL, SPI_CTRL_RUN);
    spi_fill_tx(&tx, &tx_left);

    u64 timeout = timeout_calculate(50000);
    int ret = 0;
    while (want) {
        want &= ~spi_read(SPI_IF_XFER);
        spi_fill_tx(&tx, &tx_left);
        spi_drain_rx(&rx, &rx_left);
        if (timeout_expired(timeout)) {
            printf("spihid: SPI transfer timed out (tx %u rx %u left, IF_XFER 0x%x FIFOSTAT 0x%x)\n",
                   tx_left, rx_left, spi_read(SPI_IF_XFER), spi_read(SPI_FIFOSTAT));
            ret = -1;
            break;
        }
    }
    for (int i = 0; i < 100 && rx_left; i++)
        spi_drain_rx(&rx, &rx_left);
    if (!ret && (tx_left || rx_left)) {
        printf("spihid: SPI transfer short (tx %u rx %u left)\n", tx_left, rx_left);
        ret = -1;
    }
    u32 fifo = spi_read(SPI_IF_FIFO);
    if (fifo & (SPI_FIFO_TXOVERFLOW | SPI_FIFO_RXUNDERRUN))
        printf("spihid: SPI FIFO flags 0x%x\n", fifo);
    spi_write(SPI_CTRL, 0);
    return ret;
}

/* ------------------------------------------------------------------------------------------ */
/* Packets                                                                                      */

static void hexdump_short(const u8 *p, u32 len)
{
    for (u32 i = 0; i < len; i++)
        printf("%02x%s", p[i], (i % 32 == 31 || i + 1 == len) ? "\n" : " ");
}

static int send_request(u8 target, u8 u0, u8 u1, u8 u2, u16 rsplen, const u8 *buf, u32 len)
{
    struct spihid_pkt *pkt = (void *)st.tx_buf;
    struct spihid_msg_hdr *hdr = (void *)pkt->data;

    if (len > sizeof(pkt->data) - sizeof(*hdr) - 2)
        return -1;

    memset(pkt, 0, sizeof(*pkt));
    pkt->flags = PKT_WRITE;
    pkt->device = target;
    pkt->length = sizeof(*hdr) + len + 2;
    hdr->u0 = u0;
    hdr->u1 = u1;
    hdr->u2 = u2;
    hdr->id = st.msg_id++;
    hdr->rsplen = rsplen;
    hdr->length = len;
    if (len)
        memcpy(pkt->data + sizeof(*hdr), buf, len);
    u16 crc = crc16(0, pkt->data, sizeof(*hdr) + len);
    pkt->data[sizeof(*hdr) + len] = crc & 0xff;
    pkt->data[sizeof(*hdr) + len + 1] = crc >> 8;
    pkt->crc16 = crc16(0, st.tx_buf, offsetof(struct spihid_pkt, crc16));

    memset(st.status_buf, 0, sizeof(st.status_buf));

    spi_cs(true);
    int ret = spi_xfer(st.tx_buf, NULL, PKT_SIZE);
    udelay(RW_CHG_US);
    if (!ret)
        ret = spi_xfer(NULL, st.status_buf, sizeof(st.status_buf));
    spi_cs(false);

    if (ret)
        return ret;
    if (memcmp(st.status_buf, msg_status_ok, 4)) {
        printf("spihid: status mismatch after write: %02x %02x %02x %02x\n", st.status_buf[0],
               st.status_buf[1], st.status_buf[2], st.status_buf[3]);
        return -1;
    }
    return 0;
}

static struct spihid_iface *get_iface(u32 num)
{
    if (num >= SPIHID_MAX_DEVICES)
        return NULL;
    return &st.iface[num];
}

static void process_device_info(const u8 *payload, u32 len)
{
    if (len < sizeof(struct spihid_device_info))
        return;
    const struct spihid_device_info *info = (const void *)payload;
    st.num_devices = info->num_devices;
    st.vendor_id = info->vendor_id;
    st.product_id = info->product_id;
    st.version = info->version_number;

    u16 off, slen;
    off = info->vendor_str[0], slen = info->vendor_str[1];
    if (off < len && slen <= len - off && slen < sizeof(st.vendor)) {
        memcpy(st.vendor, payload + off, slen);
        st.vendor[slen] = 0;
    }
    off = info->product_str[0], slen = info->product_str[1];
    if (off < len && slen <= len - off && slen < sizeof(st.product)) {
        memcpy(st.product, payload + off, slen);
        st.product[slen] = 0;
    }
    off = info->serial_str[0], slen = info->serial_str[1];
    if (off < len && slen <= len - off && slen < sizeof(st.serial)) {
        memcpy(st.serial, payload + off, slen);
        st.serial[slen] = 0;
    }
}

static void process_iface_info(u32 num, const u8 *payload, u32 len)
{
    struct spihid_iface *iface = get_iface(num);
    if (!iface || len < sizeof(struct spihid_iface_info))
        return;
    const struct spihid_iface_info *info = (const void *)payload;
    iface->max_input_len = info->max_input_report_len;
    iface->max_output_len = info->max_output_report_len;
    iface->max_control_len = info->max_control_report_len;
    iface->country = info->country_code;
    u16 off = info->name_offset, slen = info->name_length;
    if (off < len && slen <= len - off && slen < sizeof(iface->name)) {
        memcpy(iface->name, payload + off, slen);
        iface->name[slen] = 0;
    }
}

static void process_desc(u32 num, const u8 *payload, u32 len)
{
    struct spihid_iface *iface = get_iface(num);
    if (!iface || len > SPIHID_DESC_MAX)
        return;
    memcpy(iface->desc, payload, len);
    iface->desc_len = len;
    iface->ready = num > 0;
}

static void process_message(const u8 *data, u32 length, u8 device, u8 flags)
{
    if (length < sizeof(struct spihid_msg_hdr) + 2)
        return;
    u16 crc = crc16(0, data, length - 2);
    u16 msg_crc = data[length - 2] | (data[length - 1] << 8);
    if (crc != msg_crc) {
        st.bad_crc++;
        printf("spihid: message CRC mismatch (dev %02x)\n", device);
        return;
    }

    const struct spihid_msg_hdr *hdr = (const void *)data;
    u32 plen = hdr->length;
    if (plen == 0 || plen + sizeof(*hdr) + 2 > length)
        return;
    const u8 *payload = data + sizeof(*hdr);

    if (flags == PKT_READ) {
        if (hdr->u0 != 0x10)
            goto unhandled;
        if (device == SPIHID_DEV_MNGT) {
            /* status report from the management interface, nothing to do */
            return;
        }
        if (device < SPIHID_MAX_DEVICES) {
            st.reports++;
            if (st.cb)
                st.cb(device, payload, plen);
            return;
        }
        goto unhandled;
    }

    if (flags == PKT_WRITE) {
        if (hdr->u0 == 0x20) {
            switch (hdr->u1) {
            case 0x01:
                if (hdr->u2 == SPIHID_DEV_INFO)
                    process_device_info(payload, plen);
                return;
            case 0x02:
                process_iface_info(hdr->u2, payload, plen);
                return;
            case 0x10:
                process_desc(hdr->u2, payload, plen);
                return;
            default:
                break;
            }
        }
        if (hdr->u0 == 0x32) {
            if (plen <= sizeof(st.reply_buf)) {
                memcpy(st.reply_buf, payload, plen);
                st.reply_len = plen;
                st.reply_dev = device;
            }
            return;
        }
        if (hdr->u0 == 0x51 || hdr->u0 == 0x52)
            return; /* ack of a set/output report */
    }

unhandled:
    printf("spihid: unhandled msg flags %02x dev %02x: u0 %02x u1 %02x u2 %02x id %u len %u\n",
           flags, device, hdr->u0, hdr->u1, hdr->u2, hdr->id, plen);
    hexdump_short(payload, min(plen, 48u));
}

static void assemble_message(const struct spihid_pkt *pkt)
{
    u32 length = pkt->length, remain = pkt->remain, offset = pkt->offset;

    if (offset + length + remain > sizeof(st.asm_buf))
        return;

    if (pkt->device != st.asm_dev || pkt->flags != st.asm_flags || offset != st.asm_off) {
        st.asm_dev = 0;
        st.asm_flags = 0;
        st.asm_off = 0;
        st.asm_len = 0;
    }

    if (offset == 0) {
        memcpy(st.asm_buf, pkt->data, length);
        st.asm_off = length;
        st.asm_len = length + remain;
        st.asm_dev = pkt->device;
        st.asm_flags = pkt->flags;
    } else if (offset == st.asm_off) {
        if (offset + length + remain != st.asm_len) {
            st.asm_off = st.asm_len = 0;
            return;
        }
        memcpy(st.asm_buf + offset, pkt->data, length);
        st.asm_off += length;
        if (st.asm_off == st.asm_len) {
            process_message(st.asm_buf, st.asm_len, st.asm_dev, st.asm_flags);
            st.asm_dev = st.asm_flags = 0;
            st.asm_off = st.asm_len = 0;
        }
    }
}

static void process_read(void)
{
    const struct spihid_pkt *pkt = (const void *)st.rx_buf;

    u16 crc = crc16(0, st.rx_buf, offsetof(struct spihid_pkt, crc16));
    if (crc != pkt->crc16) {
        st.bad_crc++;
        /* An all-zero packet is what we get when the line was not actually ready */
        bool zero = true;
        for (int i = 0; i < PKT_SIZE && zero; i++)
            zero = st.rx_buf[i] == 0;
        if (!zero) {
            printf("spihid: packet CRC mismatch (flags %02x dev %02x len %u)\n", pkt->flags,
                   pkt->device, pkt->length);
            hexdump_short(st.rx_buf, 32);
        }
        return;
    }

    u32 length = pkt->length;
    if (length == 0) {
        /* all zeroes: the line was low but the device had nothing (or is not answering) */
        st.empty_pkts++;
        return;
    }
    st.empty_pkts = 0;
    if (length < sizeof(struct spihid_msg_hdr) + 2) {
        if (length == sizeof(msg_booted) && !memcmp(pkt->data, msg_booted, length)) {
            st.booted = true;
        } else {
            st.short_pkts++;
            printf("spihid: short packet (dev %02x, len %u):", pkt->device, length);
            for (u32 i = 0; i < min(length, 16u); i++)
                printf(" %02x", pkt->data[i]);
            printf("\n");
        }
        return;
    }
    if (length > sizeof(pkt->data))
        return;

    if (pkt->offset == 0 && pkt->remain == 0)
        process_message(pkt->data, length, pkt->device, pkt->flags);
    else
        assemble_message(pkt);
}

static int read_packet(void)
{
    memset(st.rx_buf, 0, PKT_SIZE);
    spi_cs(true);
    int ret = spi_xfer(NULL, st.rx_buf, PKT_SIZE);
    spi_cs(false);
    if (ret)
        return ret;
    process_read();
    return 0;
}

bool spihid_poll(void)
{
    if (!st.present)
        return false;
    if (!irq_asserted())
        return false;
    if (st.empty_pkts >= 64) {
        /* line stuck low and nothing comes back: don't hammer the bus */
        if (st.empty_pkts == 64) {
            printf("spihid: interrupt line stays low but reads return nothing, backing off\n");
            st.empty_pkts++;
        }
        return false;
    }
    read_packet();
    return true;
}

/* Poll until cond becomes true or the timeout (ms) expires */
#define WAIT_FOR(cond, ms)                                                                         \
    ({                                                                                             \
        u64 _t = timeout_calculate((ms) * 1000);                                                   \
        bool _ok;                                                                                  \
        while (!(_ok = (cond)) && !timeout_expired(_t)) {                                          \
            if (!spihid_poll())                                                                    \
                udelay(100);                                                                       \
        }                                                                                          \
        _ok;                                                                                       \
    })

/* ------------------------------------------------------------------------------------------ */
/* ADT lookup                                                                                   */

struct adt_gpio_function {
    u32 phandle;
    char four_cc[4];
    u32 gpio;
    u32 flags;
};

static int find_node_by_phandle(int node, u32 phandle)
{
    u32 v;
    if (ADT_GETPROP(adt, node, "AAPL,phandle", &v) >= 0 && v == phandle)
        return node;
    int child = node;
    ADT_FOREACH_CHILD(adt, child)
    {
        int r = find_node_by_phandle(child, phandle);
        if (r >= 0)
            return r;
    }
    return -1;
}

static int arm_io_node_base(int node, u64 *base)
{
    char path[64];
    snprintf(path, sizeof(path), "/arm-io/%s", adt_get_name(adt, node));
    int offsets[8];
    if (adt_path_offset_trace(adt, path, offsets) < 0)
        return -1;
    return adt_get_reg(adt, offsets, "reg", 0, base, NULL);
}

static int find_hardware(void)
{
    int arm_io = adt_path_offset(adt, "/arm-io");
    if (arm_io < 0)
        return -1;

    int spi = -1, hid = -1;
    int node = arm_io;
    ADT_FOREACH_CHILD(adt, node)
    {
        if (!adt_is_compatible(adt, node, "spi-1,spimc"))
            continue;
        int child = node;
        ADT_FOREACH_CHILD(adt, child)
        {
            if (adt_is_compatible(adt, child, "hid-transport,spi")) {
                spi = node;
                hid = child;
                break;
            }
        }
        if (hid >= 0)
            break;
    }
    if (hid < 0) {
        printf("spihid: no hid-transport,spi node under any SPI controller\n");
        return -1;
    }

    snprintf(st.spi_path, sizeof(st.spi_path), "/arm-io/%s", adt_get_name(adt, spi));
    snprintf(st.hid_path, sizeof(st.hid_path), "%s/%s", st.spi_path, adt_get_name(adt, hid));

    if (arm_io_node_base(spi, &st.spi_base) < 0) {
        printf("spihid: no reg for %s\n", st.spi_path);
        return -1;
    }

    /* Enable GPIO: function-spi_en = {gpio controller phandle, 'oipg', pin, flags} */
    struct adt_gpio_function fn;
    st.en_gpio_base = 0;
    if (ADT_GETPROP(adt, hid, "function-spi_en", &fn) >= 0) {
        int gpio = find_node_by_phandle(arm_io, fn.phandle);
        if (gpio >= 0 && arm_io_node_base(gpio, &st.en_gpio_base) == 0) {
            st.en_pin = fn.gpio;
            printf("spihid: enable GPIO: /arm-io/%s pin %u (flags 0x%x)\n",
                   adt_get_name(adt, gpio), fn.gpio, fn.flags);
        }
    }
    if (!st.en_gpio_base) {
        printf("spihid: function-spi_en not usable, falling back to gpio0 pin 194\n");
        int gpio = adt_path_offset(adt, "/arm-io/gpio0");
        if (gpio < 0)
            gpio = adt_path_offset(adt, "/arm-io/gpio");
        if (gpio < 0 || arm_io_node_base(gpio, &st.en_gpio_base) < 0)
            return -1;
        st.en_pin = 194;
    }

    /* Interrupt line: a pin of the "nub" GPIO controller (interrupt-parent / interrupts) */
    st.irq_gpio_base = 0;
    u32 parent = 0;
    u32 irqs_len = 0;
    const u32 *irqs = adt_getprop(adt, hid, "interrupts", &irqs_len);
    if (ADT_GETPROP(adt, hid, "interrupt-parent", &parent) >= 0 && irqs && irqs_len >= 4) {
        int gpio = find_node_by_phandle(arm_io, parent);
        if (gpio >= 0 && adt_is_compatible(adt, gpio, "gpio,t6000") &&
            arm_io_node_base(gpio, &st.irq_gpio_base) == 0) {
            st.irq_pin = irqs[0];
            printf("spihid: IRQ GPIO: /arm-io/%s pin %u\n", adt_get_name(adt, gpio), irqs[0]);
        } else if (gpio >= 0) {
            printf("spihid: interrupt-parent is /arm-io/%s (not a GPIO controller?), interrupts[0]=%u\n",
                   adt_get_name(adt, gpio), irqs[0]);
            if (arm_io_node_base(gpio, &st.irq_gpio_base) == 0 &&
                strstr(adt_get_name(adt, gpio), "gpio"))
                st.irq_pin = irqs[0];
            else
                st.irq_gpio_base = 0;
        }
    }
    if (!st.irq_gpio_base) {
        printf("spihid: falling back to nub-gpio0 pin 6 for the interrupt line\n");
        int gpio = adt_path_offset(adt, "/arm-io/nub-gpio0");
        if (gpio < 0)
            gpio = adt_path_offset(adt, "/arm-io/nub-gpio");
        if (gpio < 0 || arm_io_node_base(gpio, &st.irq_gpio_base) < 0)
            return -1;
        st.irq_pin = 6;
    }

    printf("spihid: %s at 0x%lx, enable gpio 0x%lx/%u, irq gpio 0x%lx/%u\n", st.hid_path,
           st.spi_base, st.en_gpio_base, st.en_pin, st.irq_gpio_base, st.irq_pin);
    return 0;
}

/* ------------------------------------------------------------------------------------------ */

static const char *iface_name(u32 i)
{
    return st.iface[i].name[0] ? st.iface[i].name : "?";
}

int spihid_init(void)
{
    if (st.present)
        return 0;

    if (find_hardware() < 0)
        return -1;

    if (pmgr_adt_power_enable(st.spi_path) < 0)
        printf("spihid: power enable for %s failed (continuing)\n", st.spi_path);

    /* SPI pins (SCK/MOSI/MISO/CS) to the SPI peripheral function, like Linux's spi3_pins */
    if (st.spi_base == 0x39b10c000 || st.spi_base == 0x23510c000) {
        /* T600x / T8103: spi3 = gpio pins 52..55 (t600x-gpio-pins.dtsi / t8103-pinctrl.dtsi) */
        for (u32 pin = 52; pin <= 55; pin++)
            gpio_pinmux(st.en_gpio_base, pin, 1);
    }

    spi_init();

    /* Interrupt line as a plain input */
    gpio_set_input(st.irq_gpio_base, st.irq_pin);

    /* Power cycle the device: on, off, on (spi-hid-apple-of.c power_on) */
    gpio_set_output(st.en_gpio_base, st.en_pin, 1);
    mdelay(5);
    gpio_set_output(st.en_gpio_base, st.en_pin, 0);
    mdelay(5);
    gpio_set_output(st.en_gpio_base, st.en_pin, 1);
    mdelay(50);

    st.present = true;

    if (!WAIT_FOR(st.booted, 1000)) {
        printf("spihid: no boot message (irq line %s, bad crc %u, short %u, empty %u)\n",
               irq_asserted() ? "asserted" : "idle", st.bad_crc, st.short_pkts, st.empty_pkts);
        st.empty_pkts = 0;
        /* Some firmware revisions may not send it; try to talk anyway */
    } else {
        printf("spihid: device booted\n");
    }

    if (send_request(SPIHID_DEV_INFO, 0x20, 0x01, SPIHID_DEV_INFO, 0, NULL, 0) < 0)
        printf("spihid: device info request failed\n");
    if (!WAIT_FOR(st.vendor_id, 1000)) {
        printf("spihid: no device info (irq line %s, bad crc %u)\n",
               irq_asserted() ? "asserted" : "idle", st.bad_crc);
        st.present = false;
        return -1;
    }
    printf("spihid: %04x:%04x v%04x '%s' '%s' serial '%s', %u devices\n", st.vendor_id,
           st.product_id, st.version, st.vendor, st.product, st.serial, st.num_devices);
    if (st.num_devices > SPIHID_MAX_DEVICES)
        st.num_devices = SPIHID_MAX_DEVICES;

    for (u32 i = 0; i < st.num_devices; i++) {
        send_request(SPIHID_DEV_INFO, 0x20, 0x02, i, SPIHID_DESC_MAX, NULL, 0);
        if (!WAIT_FOR(st.iface[i].max_input_len, 1000))
            printf("spihid: no interface info for device %u\n", i);
        else
            printf("spihid: dev %u '%s' country %u, report len in %u out %u ctl %u\n", i,
                   iface_name(i), st.iface[i].country, st.iface[i].max_input_len,
                   st.iface[i].max_output_len, st.iface[i].max_control_len);
    }

    for (u32 i = 1; i < st.num_devices; i++) {
        send_request(SPIHID_DEV_INFO, 0x20, 0x10, i, SPIHID_DESC_MAX, NULL, 0);
        if (!WAIT_FOR(st.iface[i].desc_len, 1000))
            printf("spihid: no report descriptor for device %u\n", i);
        else
            printf("spihid: dev %u '%s' report descriptor: %u bytes\n", i, iface_name(i),
                   st.iface[i].desc_len);
    }

    st.ready = st.iface[SPIHID_DEV_KBD].desc_len > 0;
    return st.ready ? 0 : -1;
}

bool spihid_is_ready(void)
{
    return st.ready;
}

void spihid_set_report_cb(spihid_report_cb_t cb)
{
    st.cb = cb;
}

const struct spihid_iface *spihid_get_iface(u8 device)
{
    if (!st.present || device >= SPIHID_MAX_DEVICES)
        return NULL;
    return &st.iface[device];
}

int spihid_set_report(u8 device, u8 report_id, const u8 *buf, u32 len)
{
    if (!st.present)
        return -1;
    return send_request(device, 0x52, report_id, 0x00, 2, buf, len);
}

int spihid_output_report(u8 device, const u8 *buf, u32 len)
{
    if (!st.present)
        return -1;
    return send_request(device, 0x51, device, 0x00, 0, buf, len);
}

/* ------------------------------------------------------------------------------------------ */

static void selftest_report(u8 device, const u8 *report, u32 len)
{
    printf("%s[%u]:", device == SPIHID_DEV_KBD ? "kbd" : device == SPIHID_DEV_TP ? "tp" : "dev",
           len);
    for (u32 i = 0; i < min(len, 24u); i++)
        printf(" %02x", report[i]);
    if (len > 24)
        printf(" ...");
    if (device == SPIHID_DEV_TP && report[0] == 0x02 && len >= 8 + 38 + 30) {
        /* multitouch payload: tp_header at +8 (num_fingers at +22, buttons +23), fingers after */
        const u8 *f = report + 8 + 38;
        printf(" | fingers %u btn %02x f0 x %d y %d major %d", report[8 + 22], report[8 + 23],
               (s16)(f[4] | f[5] << 8), (s16)(f[6] | f[7] << 8), (s16)(f[18] | f[19] << 8));
    }
    printf("\n");
}

void spihid_selftest(void)
{
    printf("spihid: self test (kbdtest=1)\n");
    int ret = spihid_init();
    printf("spihid: init %s\n", ret ? "FAILED" : "ok");

    for (u32 i = 1; i < SPIHID_MAX_DEVICES && !ret; i++) {
        if (!st.iface[i].desc_len)
            continue;
        printf("spihid: dev %u descriptor:\n", i);
        hexdump_short(st.iface[i].desc, st.iface[i].desc_len);
    }

    if (!ret) {
        /* trackpad into multitouch mode (hid-magicmouse: feature report {0x02, 0x01}) */
        u8 mt[2] = {0x02, 0x01};
        printf("spihid: trackpad multitouch mode: %s\n",
               spihid_set_report(SPIHID_DEV_TP, 0x02, mt, sizeof(mt)) ? "FAILED" : "ok");
    }

    spihid_set_report_cb(selftest_report);
    printf("spihid: press keys / touch the trackpad; reports are printed below\n");
    while (1) {
        if (!spihid_poll())
            udelay(500);
    }
}
