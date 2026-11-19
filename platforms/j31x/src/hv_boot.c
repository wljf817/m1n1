/* SPDX-License-Identifier: MIT */

/*
 * Standalone hypervisor boot ("hv=1" payload variable).
 *
 * Normally the m1n1 hypervisor is driven from a host computer: proxyclient/m1n1/hv/__init__.py
 * sets up EL2 over USB, uploads a guest image and answers every exception the C code does not
 * handle itself (tools/run_guest.py -r m1n1.bin -a ... boots a *second* m1n1 as the guest, which
 * then loads UEFI at EL1). This file does the same job without a host: the m1n1 that iBoot (or the
 * chainload stub) started stays at EL2 as the hypervisor, and a copy of its own image -- m1n1 +
 * devicetrees + firmware + initramfs, exactly the file on disk -- is run again as the EL1 guest.
 * Everything the Python side used to do interactively (MMIO tracing, the shell) is gone; what it
 * did automatically is ported below, and anything that would have dropped into the shell now dumps
 * the guest context to the screen and halts, so it can be photographed.
 */

#include "hv.h"
#include "adt.h"
#include "hv_input.h"
#include "hv_xhci.h"
#include "cpu_regs.h"
#include "exception.h"
#include "fb.h"
#include "gxf.h"
#include "heapblock.h"
#include "malloc.h"
#include "mcc.h"
#include "memory.h"
#include "pmgr.h"
#include "smp.h"
#include "soc.h"
#include "nvme.h"
#include "selfboot.h"
#include "string.h"
#include "uart_regs.h"
#include "utils.h"
#include "xnuboot.h"

extern char _bss_start[];

/* Set once the guest is running; makes the exception paths avoid the proxy. */
bool hv_standalone = false;
/*
 * hv=2: debugging mode. The host keeps its framebuffer console, echoes everything the guest
 * writes to the UART onto it, and turns the guest's reboot requests (watchdog) and its own
 * panics into a halt, so whatever went wrong stays on the screen.
 */
static bool hv_verbose = false;

extern u64 hv_tick_interval;

/* RAM the hypervisor keeps for itself above its current heap top (page tables, fb shadow...) */
#define HV_HOST_SLACK (256UL << 20)
/* The guest region starts 16 MB above that, like the Python code does */
#define HV_GUEST_GAP (16UL << 20)
#define HV_BOOTARGS_SIZE SZ_16K
#define HV_TICK_RATE_STANDALONE 20  /* Hz; nothing to poll without a host... */
#define HV_TICK_RATE_INPUT      100 /* ...except the internal keyboard/trackpad, once it is up */
static u32 hv_tick_hz = HV_TICK_RATE_STANDALONE;

/*
 * Payload variables parsed by payload.c: check_var() replaces their '\n' with a NUL in place,
 * so the guest copy of the image has to get the newlines back (and its own hv= turned off).
 */
#define MAX_TRACKED_VARS 32
static char *tracked_vars[MAX_TRACKED_VARS];
static int tracked_var_cnt;
static char *hv_var_val;
static bool hv_boot_wanted;

void hv_boot_note_var(char *var, char *val)
{
    if (tracked_var_cnt < MAX_TRACKED_VARS)
        tracked_vars[tracked_var_cnt++] = var;

    if (!strncmp(var, "hv=", 3)) {
        hv_var_val = val;
        hv_boot_wanted = val[0] == '1' || val[0] == '2';
        hv_verbose = val[0] == '2';
    }
}

bool hv_boot_requested(void)
{
    return hv_boot_wanted;
}

/* ------------------------------------------------------------------------------------------ */
/* Guest secondary CPU start (Python: map_essential() cpustart_wh / start_secondary())         */

static u64 cpu_start_base[4];
static int cpu_start_dies;

static u64 cpu_start_offset(void)
{
    switch (chip_id) {
        case T8103:
        case T6000:
        case T6001:
        case T6002:
            return 0x54000;
        case T8112:
        case T8122:
        case T8132:
        case T8140:
        case T6030:
            return 0x34000;
        case T6020:
        case T6021:
        case T6022:
            return 0x28000;
        case T6031:
        case T6034:
        case T6040:
        case T6041:
            return 0x88000;
        default:
            return 0;
    }
}

static void start_guest_secondary(int die, int cluster, int core)
{
    int cpus = adt_path_offset(adt, "/cpus");
    if (cpus < 0)
        return;

    u32 want = (die << 11) | (cluster << 8) | core;
    int node = cpus;
    ADT_FOREACH_CHILD(adt, node)
    {
        u32 reg, cpu_id;
        if (ADT_GETPROP(adt, node, "reg", &reg) < 0 || reg != want)
            continue;
        if (ADT_GETPROP(adt, node, "cpu-id", &cpu_id) < 0)
            cpu_id = reg;
        if (cpu_id >= MAX_CPUS)
            return;

        u64 impl[2];
        if (ADT_GETPROP_ARRAY(adt, node, "cpu-impl-reg", impl) < 0) {
            printf("HV: cpu%d has no cpu-impl-reg, cannot start it\n", cpu_id);
            return;
        }

        /* The guest wrote its own entry point into RVBAR just before poking CPU_START */
        /* RVBAR carries status bits above bit 43 (run 18 saw bit 44 set); Python masks to 44 bits */
        u64 entry = read64(impl[0]) & GENMASK(43, 12);
        u64 regs[4] = {0, 0, 0, 0};
        printf("HV: guest starts cpu%d (%d:%d:%d) at 0x%lx\n", cpu_id, die, cluster, core, entry);
        hv_start_secondary(cpu_id, (void *)entry, regs);
        return;
    }
    printf("HV: guest asked to start unknown CPU %d:%d:%d\n", die, cluster, core);
}

static bool hook_cpu_start(struct exc_info *ctx, u64 addr, u64 *val, bool write, int width)
{
    UNUSED(ctx);

    int die = 0;
    for (int i = 0; i < cpu_start_dies; i++)
        if (addr >= cpu_start_base[i] && addr < cpu_start_base[i] + 0x20)
            die = i;
    u64 off = addr - cpu_start_base[die];

    if (!write) {
        /* Nothing to hide here, read the real register */
        switch (width) {
            case 0:
                *val = read8(addr);
                break;
            case 1:
                *val = read16(addr);
                break;
            case 2:
                *val = read32(addr);
                break;
            default:
                *val = read64(addr);
                break;
        }
        return true;
    }

    /* +0: stop request, +4: "system level startup bit", +8 + 4*cluster: start core bits */
    if (off >= 8) {
        int cluster = (off - 8) / 4;
        for (int core = 0; core < 32; core++)
            if (*val & BIT(core))
                start_guest_secondary(die, cluster, core);
    }
    return true;
}

static void map_cpu_start_hooks(void)
{
    int pmgr_path[8];
    u64 pmgr_base;
    u64 off = cpu_start_offset();

    if (!off) {
        printf("HV: CPU_START offset unknown for this SoC, guest cannot start secondaries\n");
        return;
    }
    if (adt_path_offset_trace(adt, "/arm-io/pmgr", pmgr_path) < 0 ||
        adt_get_reg(adt, pmgr_path, "reg", 0, &pmgr_base, NULL) < 0) {
        printf("HV: no /arm-io/pmgr, guest cannot start secondaries\n");
        return;
    }

    u32 die_count = 1;
    int arm_io = adt_path_offset(adt, "/arm-io");
    if (arm_io >= 0)
        ADT_GETPROP(adt, arm_io, "die-count", &die_count);
    if (die_count > 4)
        die_count = 4;

    cpu_start_dies = die_count;
    for (u32 die = 0; die < die_count; die++) {
        cpu_start_base[die] = pmgr_base + off + die * PMGR_DIE_OFFSET;
        hv_map_hook(cpu_start_base[die], hook_cpu_start, 0x20);
        printf("HV: CPU_START hook at 0x%lx\n", cpu_start_base[die]);
    }
}

/* ------------------------------------------------------------------------------------------ */
/* hv=2 debugging hooks: guest UART echoed to the host console, guest reboots turned into halts */

static void die_dump(struct exc_info *ctx, const char *why) __attribute__((noreturn));

static char echo_buf[MAX_CPUS][200];
static int echo_len[MAX_CPUS];

static bool hook_uart(struct exc_info *ctx, u64 addr, u64 *val, bool write, int width)
{
    UNUSED(ctx);
    UNUSED(width);
    u64 off = addr & 0xfff;

    if (write) {
        if (off == UTXH) {
            int cpu = smp_id();
            char *buf = echo_buf[cpu];
            int *len = &echo_len[cpu];
            char c = *val;
            if (c == '\n' || *len >= (int)sizeof(echo_buf[0]) - 1) {
                buf[*len] = 0;
                printf("%s%s", buf, c == '\n' ? "\n" : "");
                *len = 0;
                if (c != '\n')
                    buf[(*len)++] = c;
            } else if (c != '\r') {
                buf[(*len)++] = c;
            }
        }
        return true;
    }

    switch (off) {
        case UTRSTAT:
            *val = UTRSTAT_TXBE | UTRSTAT_TXE;
            break;
        default:
            *val = 0;
            break;
    }
    return true;
}

static u64 wdt_hook_base;

static bool hook_wdt(struct exc_info *ctx, u64 addr, u64 *val, bool write, int width)
{
    UNUSED(width);
    if (!write) {
        *val = read32(addr);
        return true;
    }
    /* wdt_reboot(): ALARM, COUNT, then CTL = 4. Disabling (CTL = 0) is fine to pass through. */
    if (addr - wdt_hook_base == 0x1c && (*val & 4))
        die_dump(ctx, "guest asked the watchdog to reboot the machine");
    write32(addr, *val);
    return true;
}

static void map_debug_hooks(void)
{
    int path[8];
    u64 base;

    if (adt_path_offset_trace(adt, "/arm-io/uart0", path) >= 0 &&
        adt_get_reg(adt, path, "reg", 0, &base, NULL) >= 0) {
        hv_map_hook(base, hook_uart, 0x1000);
        printf("HV: guest UART at 0x%lx echoed to this console\n", base);
    }
    if (adt_path_offset_trace(adt, "/arm-io/wdt", path) >= 0 &&
        adt_get_reg(adt, path, "reg", 0, &base, NULL) >= 0) {
        wdt_hook_base = base;
        hv_map_hook(base, hook_wdt, 0x1000);
        printf("HV: guest WDT at 0x%lx trapped (reboot -> halt)\n", base);
    }
    halt_on_reboot = true;
}

/* ------------------------------------------------------------------------------------------ */
/* 32 bpp shadow framebuffer for the guest                                                     */
/*
 * iBoot hands laptops a 10-bit (x2r10g10b10) framebuffer. Windows' boot graphics, bugcheck
 * screen and basic display driver only draw into 8:8:8:8 framebuffers, so the guest gets a
 * 32 bpp shadow buffer (its boot args say depth 32) and the hypervisor converts changed rows
 * into the real framebuffer from its timer tick -- a tiny display server at EL2.
 */

static u32 *shadow_fb;      /* guest-visible, xRGB8888 */
static u64 *shadow_hash;    /* per row: hash of the pixels last converted */
static u32 *real_fb;        /* iBoot's framebuffer, x2r10g10b10 */
static u32  fb_w, fb_h, fb_stride_px;
static bool shadow_run;     /* converter active */
static u32  shadow_pass;    /* passes done; every pass force-converts one slice as a safety net */
#define SHADOW_SLICES 8
#define SHADOW_FPS    60

static inline u32 pix8to10(u32 p)
{
    u32 r = (p >> 16) & 0xff, g = (p >> 8) & 0xff, b = p & 0xff;
    return ((r << 2 | r >> 6) << 20) | ((g << 2 | g >> 6) << 10) | (b << 2 | b >> 6);
}

static inline u32 pix10to8(u32 p)
{
    return (((p >> 22) & 0xff) << 16) | (((p >> 12) & 0xff) << 8) | ((p >> 2) & 0xff);
}

/*
 * Change detection by a per-row hash of the guest's pixels: one read of the shadow buffer per
 * pass (a Retina frame is 24 MB; comparing against a saved copy read twice that). Two
 * multiply-xor lanes so the loop is memory bound. A collision would leave a row stale, which
 * the forced slice in shadow_tick() repairs within SHADOW_SLICES passes.
 */
static inline u64 shadow_row_hash(const u32 *row)
{
    const u64 *q = (const u64 *)row;
    u64 h0 = 0x9e3779b97f4a7c15ULL, h1 = 0xc2b2ae3d27d4eb4fULL;
    u32 n = fb_w / 2, i = 0;
    for (; i + 1 < n; i += 2) {
        h0 = (h0 ^ q[i]) * 0x100000001b3ULL;
        h1 = (h1 ^ q[i + 1]) * 0x100000001b3ULL;
    }
    for (; i < n; i++)
        h0 = (h0 ^ q[i]) * 0x100000001b3ULL;
    if (fb_w & 1)
        h1 ^= row[fb_w - 1];
    return h0 ^ (h1 * 0x9e3779b97f4a7c15ULL);
}

static void shadow_convert_rows(u32 from, u32 to, bool force)
{
    for (u32 y = from; y < to; y++) {
        u32 *src = shadow_fb + y * fb_stride_px;
        u32 *dst = real_fb + y * fb_stride_px;
        u64 h = shadow_row_hash(src);
        if (!force && h == shadow_hash[y])
            continue;
        shadow_hash[y] = h;
        for (u32 x = 0; x < fb_w; x++)
            dst[x] = pix8to10(src[x]);
    }
}

static void shadow_tick(void);

static void shadow_start(void)
{
    if (!shadow_fb || shadow_run)
        return;
    shadow_run = true;
    fb_set_active(false); /* the host console would overwrite the guest's picture */
    printf("HV: shadow framebuffer converter running (%ux%u)\n", fb_w, fb_h);
    shadow_convert_rows(0, fb_h, true);
}

static void shadow_stop(void)
{
    shadow_run = false;
}

/*
 * Called on every hypervisor tick of the interruptible CPU, before it takes the big hypervisor
 * lock (run 44: a slice of a Retina frame is ~2 ms of pixel conversion; done under the lock it
 * stalled every other CPU's MMIO trap for that long). Converts one slice every 1/(20*SLICES) s.
 */
void hv_standalone_shadow_tick(void)
{
    shadow_tick();
}

static void shadow_tick(void)
{
    static u64 next;
    if (!shadow_run)
        return;
    u64 now = get_ticks();
    if (now < next)
        return;
    next = now + mrs(CNTFRQ_EL0) / SHADOW_FPS;
    /*
     * Whole frame every pass, changed rows only (run 44: one slice per 100 Hz tick made a full
     * frame take 80 ms, so the pointer and the console lagged by that much). One slice per pass
     * is converted unconditionally as a safety net against hash collisions.
     */
    u32 rows = (fb_h + SHADOW_SLICES - 1) / SHADOW_SLICES;
    u32 slice = shadow_pass++ % SHADOW_SLICES;
    u32 f0 = slice * rows, f1 = min(f0 + rows, fb_h);
    shadow_convert_rows(0, f0, false);
    shadow_convert_rows(f0, f1, true);
    shadow_convert_rows(f1, fb_h, false);
}

/*
 * Debug overlay (Fn+Esc on the internal keyboard, hv_input.c): take the screen back from the
 * guest, print the input/xHCI state, and give the picture back on the next toggle.
 */
static bool overlay_had_shadow;
static volatile bool overlay_active; /* the sampler's periodic prints stay off the overlay (run 41: garbled text) */

void hv_debug_overlay(bool on)
{
    if (on) {
        overlay_active = true;
        overlay_had_shadow = shadow_run;
        shadow_stop();
        if (!hv_verbose)
            fb_init(false);
        fb_set_active(true);
        fb_clear(int2rgb(0)); /* the guest's text would otherwise show through ours */
        printf("HV: ---- debug overlay (Fn+Esc again to resume) ----\n");
        hv_standalone_dump_cpus();
        hv_prof_dump();
        hv_input_stats();
        hv_xhci_dump();
    } else {
        if (!hv_verbose) {
            fb_set_active(false);
            fb_shutdown(true);
        }
        if (overlay_had_shadow)
            shadow_start();
        else
            fb_set_active(hv_verbose);
        overlay_active = false;
    }
}

/* ------------------------------------------------------------------------------------------ */
/* Guest memory image (Python: load_raw())                                                     */

static u64 memmap_get(int memmap, const char *name, u64 *addr, u64 *size)
{
    u64 v[2];
    if (ADT_GETPROP_ARRAY(adt, memmap, name, v) < 0)
        return 0;
    *addr = v[0];
    *size = v[1];
    return 1;
}

static void memmap_set(int memmap, const char *name, u64 addr, u64 size)
{
    u64 v[2] = {addr, size};
    if (adt_setprop(adt, memmap, name, v, sizeof(v)) < 0)
        printf("HV: failed to set /chosen/memory-map/%s\n", name);
}

/* J31x WinPE uses a single FAT32 partition in an MBR disk image. */
static int j31x_check_disk(const void *disk, u64 size)
{
    const u8 *p = disk;
    if (size >= 1024 && !memcmp(p + 512, "EFI PART", 8))
        return selfboot_check_disk(disk, size);
    if (!size || size % SELFBOOT_DISK_PAGE || p[510] != 0x55 || p[511] != 0xaa)
        return -1;
    u32 first, sectors;
    memcpy(&first, p + 454, 4);
    memcpy(&sectors, p + 458, 4);
    return (p[450] == 0x0c || p[450] == 0x0b) && first && sectors &&
                   (u64)first + sectors <= size / 512 ? 0 : -1;
}

int hv_boot(const char *partition)
{
    if (!in_el2()) {
        printf("HV: hv=1 but we are not in EL2 (already a guest?), ignoring\n");
        return -1;
    }
    /* (boot_cpu_idx is only known after hv_init() -> smp_start_secondaries(); hv_start() checks it) */

    printf("HV: standalone hypervisor boot\n");

    /* --- Memory layout ------------------------------------------------------------------ */

    u64 heap_top = (u64)heapblock_alloc(0);
    u64 phys_base = ALIGN_UP(heap_top + HV_HOST_SLACK, SZ_16K);
    heapblock_set_limit((void *)phys_base);

    /* Shadow framebuffer: first thing in the guest region, outside the guest's RAM range */
    u64 shadow_base = 0, shadow_size = 0;
    if ((cur_boot_args.video.depth & 0xff) == 30 && cur_boot_args.video.base) {
        fb_w = cur_boot_args.video.width;
        fb_h = cur_boot_args.video.height;
        fb_stride_px = cur_boot_args.video.stride / 4;
        shadow_size = ALIGN_UP(cur_boot_args.video.stride * fb_h, SZ_16K);
        shadow_base = phys_base;
        phys_base += shadow_size;
        shadow_fb = (u32 *)shadow_base;
        real_fb = (u32 *)cur_boot_args.video.base;
        shadow_hash = malloc(fb_h * sizeof(u64));
        if (!shadow_hash) {
            printf("HV: no memory for the shadow framebuffer cache\n");
            shadow_fb = NULL;
        }
    }

    u64 mem_top = cur_boot_args.phys_base + cur_boot_args.mem_size;
    u64 ram_top = ram_base + mem_size_actual;
    if (ram_top < mem_top) /* boot_args revision without mem_size_actual */
        ram_top = mem_top;

    /* The image we are running from: m1n1 + payloads (+ chainload vars/SEPFW/bootargs) */
    u64 image_base = (u64)_base;
    u64 image_len = image_end - image_base;

    int memmap = adt_path_offset(adt, "/chosen/memory-map");
    if (memmap < 0) {
        printf("HV: no /chosen/memory-map\n");
        return -1;
    }
    u64 sepfw_addr = 0, sepfw_len = 0, tc_addr = 0, tc_len = 0, pol_addr = 0, pol_len = 0;
    memmap_get(memmap, "SEPFW", &sepfw_addr, &sepfw_len);
    bool have_tc = memmap_get(memmap, "TrustCache", &tc_addr, &tc_len);
    bool have_pol = memmap_get(memmap, "preoslog", &pol_addr, &pol_len);

    u64 image_size = ALIGN_UP(image_len, SZ_16K);
    u64 sepfw_off = image_size;
    image_size += ALIGN_UP(sepfw_len, SZ_16K);
    u64 pol_off = image_size;
    image_size += have_pol ? ALIGN_UP(pol_len, SZ_16K) : 0;
    u64 bootargs_off = image_size;
    image_size += HV_BOOTARGS_SIZE;

    u64 adt_base = phys_base + HV_GUEST_GAP;
    u64 adt_size = ALIGN_UP(cur_boot_args.devtree_size, SZ_16K);
    u64 tc_base = adt_base + adt_size;
    u64 guest_base = tc_base + (have_tc ? ALIGN_UP(tc_len, SZ_16K) : 0);
    u64 entry = guest_base + 0x800;

    if (guest_base + image_size > mem_top) {
        printf("HV: guest image (0x%lx bytes) does not fit above 0x%lx\n", image_size, guest_base);
        return -1;
    }

    printf("HV: host image  0x%lx..0x%lx (0x%lx bytes)\n", image_base, image_base + image_len,
           image_len);
    printf("HV: host heap top 0x%lx, guest RAM 0x%lx..0x%lx\n", heap_top, phys_base, mem_top);
    printf("HV: guest ADT 0x%lx, image 0x%lx (0x%lx bytes incl. SEPFW/bootargs), entry 0x%lx\n",
           adt_base, guest_base, image_size, entry);

    struct selfboot_layout files = {0};
    if (partition) {
        /* Leave the EL1 trampoline, SEPFW, TrustCache and boot args intact. */
        if (selfboot_layout_init(&files, ALIGN_UP(guest_base + image_size, SZ_16K),
                                 &cur_boot_args) < 0)
            return -1;
        /* The board firmware copies boot args and its ADT to this fixed address. */
        files.uefi_base = min(files.uefi_base, 0x10220000000ULL);
        for (size_t i = 0; i < mcc_carveout_count; i++) {
            u64 base = mcc_carveouts[i].base;
            if (base <= files.image_base && files.image_base - base < mcc_carveouts[i].size)
                return -1;
            if (base > files.image_base && mcc_carveouts[i].size)
                files.uefi_base = min(files.uefi_base, base);
        }
        if (files.image_base + SELFBOOT_BOOTARGS_SIZE >= files.uefi_base || !nvme_init())
            return -1;
        int ret = selfboot_read_files(&files, partition, j31x_check_disk);
        nvme_shutdown();
        if (ret < 0)
            return -1;
        dc_cvau_range((void *)files.image_base, files.image_size);
        ic_ivau_range((void *)files.image_base, files.image_size);
    }

    /* --- Hypervisor init (Python: HV.init()) --------------------------------------------- */

    hv_init();
    hv_tick_interval = mrs(CNTFRQ_EL0) / HV_TICK_RATE_STANDALONE;

    /* Trap dangerous implementation-defined things */
    /*
     * Python also traps HID/EHID (to log them); here that only made every chicken-bit access
     * of the guest's secondary CPUs a trap + print, slow enough for the guest's 100 ms
     * spin-table timeout to expire (run 19). They pass straight through instead.
     */
    u64 hacr = mrs(HACR_EL2);
    hacr |= BIT(14) /* CTRR */ | BIT(16) /* IPI */ | BIT(48) /* SERROR_INFO (M1RACLES) */ |
            BIT(52) /* ACC */ | BIT(56) /* PMUv3 */ | BIT(57) /* PM */;
    msr(HACR_EL2, hacr);

    /*
     * Python routes debug exceptions to EL2 (MDCR TDE/TDA/TDOSA/TDRA, MDSCR MDE) for its own
     * breakpoint support. We have no debugger here, and the NT kernel's first 'brk #0xF002'
     * (run 25) must reach the kernel's own vector, not us. HPMN = 0: the PMU is emulated anyway.
     */
    msr(MDCR_EL2, 0);

    if (cpu_features->apple_sysregs_unlocked) {
        reg_set(SYS_IMP_APL_AMX_CTL_EL1, AMX_CTL_EN_EL1);
        msr(SYS_IMP_APL_APVMKEYLO_EL2, 0x4E7672476F6E6147);
        msr(SYS_IMP_APL_APVMKEYHI_EL2, 0x697665596F755570);
        msr(SYS_IMP_APL_APSTS_EL12, 1);
    }

    if (cpu_features->actlr_el2)
        reg_set(SYS_ACTLR_EL12, BIT(12) /* EnMDSB */);
    else
        reg_set(SYS_IMP_APL_ACTLR_EL12, BIT(12) /* EnMDSB */);

    /* --- Stage 2 map: MMIO as hardware, RAM as identity, minus carveouts ---------------- */

    int arm_io = adt_path_offset(adt, "/arm-io");
    u32 ranges_len = 0;
    const u64 *ranges = arm_io >= 0 ? adt_getprop(adt, arm_io, "ranges", &ranges_len) : NULL;
    if (!ranges) {
        printf("HV: no /arm-io ranges\n");
        return -1;
    }
    for (u32 i = 0; i + 3 <= ranges_len / 8; i += 3) {
        u64 parent = ALIGN_DOWN(ranges[i + 1], SZ_16K);
        u64 size = ALIGN_UP(ranges[i + 1] + ranges[i + 2], SZ_16K) - parent;
        printf("HV: MMIO 0x%lx..0x%lx\n", parent, parent + size);
        if (hv_map_hw(parent, parent, size) < 0)
            printf("HV: failed to map MMIO range\n");
    }

    printf("HV: RAM  0x%lx..0x%lx, 0x%lx..0x%lx\n", ram_base, cur_boot_args.phys_base, phys_base,
           ram_top);
    if (cur_boot_args.phys_base > ram_base &&
        hv_map_hw(ram_base, ram_base, cur_boot_args.phys_base - ram_base) < 0)
        printf("HV: failed to map low RAM\n");
    u64 map_from = shadow_fb ? shadow_base : phys_base;
    if (hv_map_hw(map_from, map_from, ram_top - map_from) < 0) {
        printf("HV: failed to map guest RAM\n");
        return -1;
    }
    for (size_t i = 0; i < mcc_carveout_count; i++) {
        printf("HV: unmap carveout 0x%lx..0x%lx\n", mcc_carveouts[i].base,
               mcc_carveouts[i].base + mcc_carveouts[i].size);
        hv_unmap(mcc_carveouts[i].base, mcc_carveouts[i].size);
    }

    map_cpu_start_hooks();
    if (hv_verbose)
        map_debug_hooks();

    /* Internal keyboard/trackpad for the guest: real SPI HID at EL2, emulated xHCI + USB HID */
    if (hv_input_init(phys_base, mem_top) == 0) {
        hv_tick_hz = HV_TICK_RATE_INPUT;
        hv_tick_interval = mrs(CNTFRQ_EL0) / hv_tick_hz;
    }

    /* --- Guest image ---------------------------------------------------------------------- */

    /*
     * Rebuild the file from what is in memory: text/rodata as they are (the only patches,
     * pan_fixup and the SError vector NOP, are idempotent), .data from the pristine copy
     * _start_c() saved, .bss/stacks zeroed (mmu_init() unmapped guard pages in there, and the
     * guest clears them anyway), payloads as they are.
     */
    printf("HV: copying image...\n");
    u64 text_len = (u64)_rodata_end - image_base;
    u64 data_len = (u64)_bss_start - (u64)_rodata_end;
    u64 bss_len = (u64)_payload_start - (u64)_bss_start;
    memcpy((void *)guest_base, (void *)image_base, text_len);
    memcpy((void *)(guest_base + text_len), pristine_data, data_len);
    memset((void *)(guest_base + text_len + data_len), 0, bss_len);
    memcpy((void *)(guest_base + text_len + data_len + bss_len), _payload_start,
           image_len - (text_len + data_len + bss_len));

    /* Give the payload variables their newlines back, and turn hv= off for the guest */
    for (int i = 0; i < tracked_var_cnt; i++) {
        char *v = tracked_vars[i];
        if ((u64)v < image_base || (u64)v >= image_base + image_len)
            continue; /* came from inside a decompressed payload, not part of the file */
        char *g = (char *)(guest_base + ((u64)v - image_base));
        g[strlen(v)] = '\n';
    }
    if (hv_var_val && (u64)hv_var_val >= image_base && (u64)hv_var_val < image_base + image_len)
        *(char *)(guest_base + ((u64)hv_var_val - image_base)) = '0';
    if ((u64)adt & (SZ_16K - 1))
        printf("HV: warning: host ADT at %p is not page aligned\n", adt);

    dc_cvau_range((void *)guest_base, image_len);
    ic_ivau_range((void *)guest_base, image_len);

    if (sepfw_len)
        memcpy((void *)(guest_base + sepfw_off), (void *)sepfw_addr, sepfw_len);
    if (have_tc)
        memcpy((void *)tc_base, (void *)tc_addr, tc_len);
    if (have_pol)
        memcpy((void *)(guest_base + pol_off), (void *)pol_addr, pol_len);

    /* Guest ADT: a copy with the memory map pointing at the guest's own copies */
    if (partition) {
        files.adt_base = adt_base;
        files.adt_size = adt_size;
        files.bootargs = guest_base + bootargs_off;
        if (selfboot_adt((void *)adt_base, cur_boot_args.devtree_size, &files) < 0 ||
            selfboot_adt_add_range((void *)adt_base, "SiliconNTFirmware", files.image_base,
                                   files.image_size) < 0)
            return -1;
    } else {
        memcpy((void *)adt_base, adt, cur_boot_args.devtree_size);
    }
    void *host_adt = adt;
    adt = (void *)adt_base; /* adt_setprop() only works on the global */
    memmap_set(memmap, "SEPFW", guest_base + sepfw_off, sepfw_len);
    if (have_tc)
        memmap_set(memmap, "TrustCache", tc_base, tc_len);
    memmap_set(memmap, "DeviceTree", adt_base, adt_size);
    memmap_set(memmap, "BootArgs", guest_base + bootargs_off, HV_BOOTARGS_SIZE);
    if (have_pol)
        memmap_set(memmap, "preoslog", guest_base + pol_off, pol_len);
    adt = host_adt;

    /* Guest boot args */
    struct boot_args *tba = (void *)(guest_base + bootargs_off);
    *tba = cur_boot_args;
    tba->mem_size = mem_top - phys_base;
    tba->phys_base = phys_base;
    tba->virt_base = 0xfffffe0010000000ULL + (phys_base & (SZ_32M - 1));
    tba->devtree = (void *)(adt_base - phys_base + tba->virt_base);
    tba->top_of_kernel_data = partition ? files.top : guest_base + image_size;
    if (shadow_fb) {
        /* Start from what is on the screen now (m1n1 logo), converted to 8 bits per channel */
        for (u32 y = 0; y < fb_h; y++)
            for (u32 x = 0; x < fb_w; x++)
                shadow_fb[y * fb_stride_px + x] = pix10to8(real_fb[y * fb_stride_px + x]);
        for (u32 y = 0; y < fb_h; y++)
            shadow_hash[y] = shadow_row_hash(shadow_fb + y * fb_stride_px);
        tba->video.base = shadow_base;
        tba->video.depth = 32 | (cur_boot_args.video.depth & ~0xffULL);
        printf("HV: guest framebuffer: 32 bpp shadow at 0x%lx (%ux%u), real one at 0x%lx is 30 bpp\n",
               shadow_base, fb_w, fb_h, cur_boot_args.video.base);
    }
    dc_cvau_range(tba, sizeof(*tba));

    /* Secondary CPU RVBARs point at the guest entry page (the guest rewrites them anyway) */
    if (cpu_features->apple_sysregs_unlocked) {
        int cpus = adt_path_offset(adt, "/cpus");
        int node = cpus;
        if (cpus >= 0) {
            ADT_FOREACH_CHILD(adt, node)
            {
                const char *state = adt_getprop(adt, node, "state", NULL);
                if (state && !strcmp(state, "running"))
                    continue;
                u64 impl[2];
                if (ADT_GETPROP_ARRAY(adt, node, "cpu-impl-reg", impl) < 0)
                    continue;
                write64(impl[0], entry & ~0xfffULL);
            }
        }
    }

    /* --- Go (Python: HV.start()) ---------------------------------------------------------- */

    printf("HV: entering guest at 0x%lx (bootargs 0x%lx)\n", entry, guest_base + bootargs_off);
    iodev_console_flush();

    if (!hv_verbose) {
        fb_improve_logo();
        fb_shutdown(true);
        shadow_start();
    }

    /*
     * Python's start(): SPRR and GXF on at EL2 before entering the guest. m1n1's own init left
     * both off; without GXF enabled here the guest's gxf_init() faults on GXF_CONFIG_EL1.
     */
    if (supports_gxf()) {
        printf("HV: enabling SPRR and GXF\n");
        msr(SYS_IMP_APL_SPRR_CONFIG_EL1, 1);
        sysop("isb");
        msr(SYS_IMP_APL_GXF_CONFIG_EL1, 1);
        sysop("isb");
    }

    hv_standalone = true;

    u64 regs[4] = {guest_base + bootargs_off, 0, 0, 0};
    hv_start((void *)entry, regs);

    /* Only reached if the guest exits on every CPU */
    hv_standalone = false;
    if (!hv_verbose) {
        fb_init(false);
        fb_set_active(true);
    }
    printf("HV: guest exited\n");
    return -1;
}

/* ------------------------------------------------------------------------------------------ */
/* Exceptions the C hypervisor would have sent to the host                                     */

/* Pass a trapped MRS/MSR straight through to the real register (Python: "Pass: mrs/msr") */
static u64 sysreg_passthrough(u64 iss, u64 value, bool *ok)
{
    static u32 *code;
    if (!code)
        code = memalign(64, 64);

    u32 op0 = FIELD_GET(ESR_ISS_MSR_OP0, iss);
    u32 op1 = FIELD_GET(ESR_ISS_MSR_OP1, iss);
    u32 crn = FIELD_GET(ESR_ISS_MSR_CRn, iss);
    u32 crm = FIELD_GET(ESR_ISS_MSR_CRm, iss);
    u32 op2 = FIELD_GET(ESR_ISS_MSR_OP2, iss);
    bool is_read = iss & ESR_ISS_MSR_DIR;

    u32 insn = (is_read ? 0xd5300000 : 0xd5100000) | ((op0 & 1) << 19) | (op1 << 16) |
               (crn << 12) | (crm << 8) | (op2 << 5) | 0; /* x0 */
    code[0] = insn;
    code[1] = 0xd65f03c0; /* ret */
    dc_cvau_range(code, 8);
    ic_ivau_range(code, 8);
    sysop("isb");

    u64 (*fn)(u64) = (void *)(((u64)code) | REGION_RX_EL1);

    exc_count = 0;
    exc_guard = GUARD_SKIP | GUARD_SILENT;
    u64 ret = fn(value);
    exc_guard = GUARD_OFF;
    *ok = exc_count == 0;
    exc_count = 0;

    return is_read ? ret : 0;
}

static bool handle_sysreg(struct exc_info *ctx, u64 iss)
{
    u32 op0 = FIELD_GET(ESR_ISS_MSR_OP0, iss);
    u32 op1 = FIELD_GET(ESR_ISS_MSR_OP1, iss);
    u32 crn = FIELD_GET(ESR_ISS_MSR_CRn, iss);
    u32 crm = FIELD_GET(ESR_ISS_MSR_CRm, iss);
    u32 op2 = FIELD_GET(ESR_ISS_MSR_OP2, iss);
    u32 rt = FIELD_GET(ESR_ISS_MSR_Rt, iss);
    bool is_read = iss & ESR_ISS_MSR_DIR;
    u64 value = rt == 31 ? 0 : ctx->regs[rt];

    /* Debug breakpoint/watchpoint registers and VMSA_LOCK: keep them off the real hardware */
    bool shadow = (op0 == 2 && op1 == 0 && crn == 0 && (op2 >= 4 && op2 <= 7)) || /* DBGB/DBGW */
                  (op0 == 3 && op1 == 4 && crn == 15 && crm == 1 && op2 == 2);      /* VMSA_LOCK */

    if (shadow) {
        if (is_read && rt != 31)
            ctx->regs[rt] = 0;
        return true;
    }

    bool ok;
    u64 ret = sysreg_passthrough(iss, value, &ok);
    if (is_read && rt != 31)
        ctx->regs[rt] = ok ? ret : 0;

    /* Say it once per register (and every time it fails), not on every access */
    static u32 seen[64];
    static int nseen;
    u32 enc = (iss & (ESR_ISS_MSR_OP0 | ESR_ISS_MSR_OP2 | ESR_ISS_MSR_OP1 | ESR_ISS_MSR_CRn |
                      ESR_ISS_MSR_CRm)) | (is_read ? 1 : 0);
    bool first = true;
    for (int i = 0; i < nseen; i++)
        if (seen[i] == enc)
            first = false;
    if (first && nseen < 64)
        seen[nseen++] = enc;

    if (first || !ok)
        printf("HV: %s S%u_%u_C%u_C%u_%u %s 0x%lx (guest pc 0x%lx)%s\n", is_read ? "mrs" : "msr",
               op0, op1, crn, crm, op2, is_read ? "->" : "<-", is_read ? ctx->regs[rt] : value,
               ctx->elr, ok ? " [first time, later ones silent]" : " [UNDEF at EL2 too, ignored]");
    return true;
}

static bool handle_sync(struct exc_info *ctx)
{
    u32 ec = FIELD_GET(ESR_EC, ctx->esr);

    switch (ec) {
        case ESR_EC_MSR:
            if (handle_sysreg(ctx, FIELD_GET(ESR_ISS, ctx->esr))) {
                ctx->elr += 4;
                return true;
            }
            break;
        case ESR_EC_IMPDEF:
            if (FIELD_GET(ESR_ISS, ctx->esr) == ESR_ISS_IMPDEF_MSR &&
                handle_sysreg(ctx, ctx->afsr1)) {
                ctx->elr += 4;
                return true;
            }
            break;
        case ESR_EC_HVC:
            /* PSCI over the HVC conduit; ELR already points past the HVC */
            printf("HV: guest HVC 0x%lx (x0=0x%lx)\n", FIELD_GET(ESR_ISS, ctx->esr), ctx->regs[0]);
            return hv_handle_psci_smc(ctx);
        case ESR_EC_SMC:
            printf("HV: guest SMC (x0=0x%lx)\n", ctx->regs[0]);
            if (hv_handle_psci_smc(ctx)) {
                ctx->elr += 4;
                return true;
            }
            break;
    }
    return false;
}

static const char *reason_name(uartproxy_boot_reason_t reason, u32 type)
{
    if (reason == START_EXCEPTION_LOWER || reason == START_EXCEPTION) {
        switch (type) {
            case EXC_SYNC:
                return "SYNC";
            case EXC_IRQ:
                return "IRQ";
            case EXC_FIQ:
                return "FIQ";
            case EXC_SERROR:
                return "SERROR";
        }
    } else if (reason == START_HV) {
        switch (type) {
            case HV_HOOK_VM:
                return "HOOK_VM";
            case HV_VTIMER:
                return "VTIMER";
            case HV_USER_INTERRUPT:
                return "USER_INTERRUPT";
            case HV_WDT_BARK:
                return "WDT_BARK";
            case HV_CPU_SWITCH:
                return "CPU_SWITCH";
            case HV_VIRTIO:
                return "VIRTIO";
            case HV_PANIC:
                return "PANIC";
        }
    }
    return "?";
}

/*
 * Guest address -> "image.pdb+offset". UEFI drivers, bootmgfw/winload and the NT kernel are all
 * PE/COFF images loaded page aligned, with a CodeView (RSDS) entry in the debug directory that
 * names the PDB -- the same lookup the firmware's exception dump does. Walk back from the
 * address page by page until an "MZ" header whose SizeOfImage covers it turns up.
 */
/*
 * hv_pt_is_ram() only says the IPA has a direct (HW) stage-2 mapping, which pass-through MMIO
 * has too: run 43 died at EL2 with a bus error reading 0x1400006000 while the stack scan walked
 * back from a guest pointer. Only touch what is really RAM.
 */
static bool guest_read32(u64 va, u32 *out)
{
    u64 pa = hv_translate(va, false, false, NULL);
    if (!pa || pa < ram_base || pa >= ram_base + mem_size_actual || !hv_pt_is_ram(pa & ~0x3fffULL))
        return false;
    *out = read32(pa);
    return true;
}

static bool guest_read64(u64 va, u64 *out)
{
    u32 lo, hi;
    if (!guest_read32(va, &lo) || !guest_read32(va + 4, &hi))
        return false;
    *out = ((u64)hi << 32) | lo;
    return true;
}

static u64 guest_image_base_found; /* base of the image guest_image_name() last resolved */

static int guest_image_name(u64 addr, char *buf, size_t len)
{
    u32 w, e_lfanew, size_of_image;
    for (u64 base = addr & ~0xfffULL; addr - base < 32 * 1024 * 1024; base -= 0x1000) {
        if (!guest_read32(base, &w))
            return -1; /* fell off RAM */
        if ((w & 0xffff) != 0x5a4d)
            continue;
        if (!guest_read32(base + 0x3c, &e_lfanew) || e_lfanew > 0x1000 ||
            !guest_read32(base + e_lfanew, &w) || w != 0x00004550)
            continue;
        u64 opt = base + e_lfanew + 24;
        if (!guest_read32(opt + 56, &size_of_image) || addr - base >= size_of_image)
            continue;
        if (!guest_read32(opt, &w) || (w & 0xffff) != 0x20b) /* PE32+ */
            continue;
        guest_image_base_found = base;
        snprintf(buf, len, "<MZ@0x%lx>+0x%lx", base, addr - base);
        u32 nrva, dbg_rva, dbg_size;
        if (!guest_read32(opt + 108, &nrva) || nrva <= 6 || !guest_read32(opt + 112 + 6 * 8, &dbg_rva) ||
            !guest_read32(opt + 112 + 6 * 8 + 4, &dbg_size) || !dbg_rva)
            return 0;
        for (u32 off = 0; off + 28 <= dbg_size && off < 28 * 8; off += 28) {
            u32 type, raw_rva, raw_ptr;
            if (!guest_read32(base + dbg_rva + off + 12, &type) || type != 2) /* CODEVIEW */
                continue;
            if (!guest_read32(base + dbg_rva + off + 20, &raw_rva) ||
                !guest_read32(base + dbg_rva + off + 24, &raw_ptr))
                break;
            u64 cv = base + (raw_rva ? raw_rva : raw_ptr);
            u32 sig;
            if (!guest_read32(cv, &sig))
                break;
            u64 path = sig == 0x53445352 ? cv + 24 : sig == 0x3031424e ? cv + 16 : 0; /* RSDS / NB10 */
            if (!path)
                break;
            char name[96];
            int n = 0, start = 0;
            for (; n < (int)sizeof(name) - 1; n++) {
                u32 c;
                if (!guest_read32((path + n) & ~3ULL, &c))
                    break;
                c = (c >> (8 * ((path + n) & 3))) & 0xff;
                if (!c)
                    break;
                name[n] = c;
                if (c == '\\' || c == '/')
                    start = n + 1;
            }
            name[n] = 0;
            if (n > start)
                snprintf(buf, len, "%s+0x%lx", name + start, addr - base);
            return 0;
        }
        return 0;
    }
    return -1;
}

static void guest_stack_images_filtered(u64 sp, int words, int max_shown, const char *skip_prefix);

/* Words on the guest stack that point into some image: a poor man's backtrace */
static void guest_stack_images_n(u64 sp, int words, int max_shown)
{
    guest_stack_images_filtered(sp, words, max_shown, NULL);
}

/* As above, skipping pointers into the image whose name starts with skip_prefix (e.g. the kernel) */
static void guest_stack_images_filtered(u64 sp, int words, int max_shown, const char *skip_prefix)
{
    char name[128];
    int shown = 0;
    for (int i = 0; i < words; i++) {
        u32 lo, hi;
        if (!guest_read32(sp + 8 * i, &lo) || !guest_read32(sp + 8 * i + 4, &hi))
            break;
        u64 v = ((u64)hi << 32) | lo;
        if (v < 0x10000 || (v & 3))
            continue;
        if (guest_image_name(v, name, sizeof(name)) < 0)
            continue;
        if (skip_prefix && !strncmp(name, skip_prefix, strlen(skip_prefix)))
            continue;
        printf("HV sample:   [sp+0x%03x] 0x%lx %s\n", 8 * i, v, name);
        if (++shown >= max_shown)
            break;
    }
}

static void guest_stack_images(u64 sp, int words)
{
    guest_stack_images_n(sp, words, 12);
}

/*
 * Walk the guest's frame-pointer chain (Windows on ARM64 keeps x29 chains everywhere: [fp] = caller's
 * fp, [fp+8] = return address) and name each return address. Stops at the first frame that does not
 * read, does not move up the stack, or does not point into an image.
 */
static void guest_fp_backtrace(u64 fp, u64 lr, int max_frames)
{
    char name[128];
    u64 prev = 0;
    if (lr && guest_image_name(lr, name, sizeof(name)) == 0)
        printf("HV sample:   lr   %s\n", name);
    for (int i = 0; i < max_frames; i++) {
        u32 lo, hi;
        if (!fp || (fp & 7) || fp <= prev)
            break;
        if (!guest_read32(fp + 8, &lo) || !guest_read32(fp + 12, &hi))
            break;
        u64 ret = ((u64)hi << 32) | lo;
        if (!guest_read32(fp, &lo) || !guest_read32(fp + 4, &hi))
            break;
        u64 next = ((u64)hi << 32) | lo;
        if (guest_image_name(ret, name, sizeof(name)) < 0)
            break;
        printf("HV sample:   fp%-2d %s\n", i, name);
        prev = fp;
        fp = next;
    }
}

/*
 * Every CPU records where its guest was at its last host timer tick (the sampler itself only
 * runs on the interruptible CPU). The Fn+Esc overlay prints them all with image names and the
 * return addresses on each stack: when Windows bugchecks, the crashing CPU sits in
 * KeBugCheckEx and the frames below it name the driver that faulted.
 */
static struct {
    u64 pc, spsr, sp1, sp0, fp, lr;
    u64 pmc0, cnt, pmc0_hz; /* PMC0 (the guest's PMCCNTR) and CNTPCT at the last tick, and the rate between ticks */
    u32 n;
} cpu_last[MAX_CPUS];

void hv_standalone_note_cpu(int cpu, u64 pc, u64 spsr, u64 sp1, u64 sp0, u64 fp, u64 lr)
{
    if (cpu < 0 || cpu >= MAX_CPUS)
        return;
    u64 pmc0 = mrs(SYS_IMP_APL_PMC0), cnt = mrs(CNTPCT_EL0);
    if (cpu_last[cpu].n && cnt > cpu_last[cpu].cnt)
        cpu_last[cpu].pmc0_hz = (pmc0 - cpu_last[cpu].pmc0) * mrs(CNTFRQ_EL0) / (cnt - cpu_last[cpu].cnt);
    cpu_last[cpu].pmc0 = pmc0;
    cpu_last[cpu].cnt = cnt;
    cpu_last[cpu].pc = pc;
    cpu_last[cpu].spsr = spsr;
    cpu_last[cpu].sp1 = sp1;
    cpu_last[cpu].sp0 = sp0;
    cpu_last[cpu].fp = fp;
    cpu_last[cpu].lr = lr;
    cpu_last[cpu].n++;
}

/*
 * KeBugCheckEx leaves its five arguments in the kernel's KiBugCheckData[] (and sets
 * KiBugCheckActive). Those are the numbers the blue screen does not show: for 0x7E
 * SYSTEM_THREAD_EXCEPTION_NOT_HANDLED they are the exception code, the faulting PC, the
 * EXCEPTION_RECORD and the CONTEXT -- the context is the complete register file at the fault.
 * The variables are not exported, so their RVAs are per kernel build, keyed by the PE
 * TimeDateStamp (from the public ntkrnlmp.pdb of that build).
 */
static const struct {
    u32 timestamp;
    u32 active_rva, data_rva;
    const char *build;
} kernel_builds[] = {
    {0x409ce488, 0xdbbf08, 0xdbbf40, "26300.9457 arm64 (Windows11_Client_arm64_en-us_26300_9457.iso WinPE)"},
};

static void guest_name_or_hex(u64 v, char *buf, size_t len)
{
    if (v < 0x10000 || guest_image_name(v, buf, len) < 0)
        snprintf(buf, len, "0x%lx", v);
}

static void guest_dump_bugcheck(u64 kbase)
{
    u32 e_lfanew, ts;
    if (!guest_read32(kbase + 0x3c, &e_lfanew) || !guest_read32(kbase + e_lfanew + 8, &ts))
        return;
    int b = -1;
    for (unsigned i = 0; i < sizeof(kernel_builds) / sizeof(kernel_builds[0]); i++)
        if (kernel_builds[i].timestamp == ts)
            b = i;
    if (b < 0) {
        printf("HV bugcheck: kernel TimeDateStamp 0x%x unknown, add it to kernel_builds[]\n", ts);
        return;
    }
    u32 active;
    u64 p[5];
    if (!guest_read32(kbase + kernel_builds[b].active_rva, &active))
        return;
    for (int i = 0; i < 5; i++)
        if (!guest_read64(kbase + kernel_builds[b].data_rva + 8 * i, &p[i]))
            return;
    if (!active && !p[0]) {
        printf("HV bugcheck: none (KiBugCheckActive 0, kernel %s)\n", kernel_builds[b].build);
        return;
    }
    char n1[128], n2[128], n3[128];
    guest_name_or_hex(p[1], n1, sizeof(n1));
    guest_name_or_hex(p[2], n2, sizeof(n2));
    guest_name_or_hex(p[3], n3, sizeof(n3));
    printf("HV bugcheck: 0x%lx (active %u) P1 %s P2 %s P3 %s P4 0x%lx\n", p[0], active, n1, n2, n3, p[4]);

    u64 rec = 0, ctx = 0;
    if (p[0] == 0x7e || p[0] == 0x1000007e) { /* code, address, EXCEPTION_RECORD*, CONTEXT* */
        rec = p[3];
        ctx = p[4];
    } else if (p[0] == 0x1e) { /* code, address, param0, param1 */
        return;
    } else if (p[0] == 0x139 || p[0] == 0x3b || p[0] == 0x7f) { /* ..., trap frame / context */
        ctx = p[0] == 0x3b ? p[3] : 0;
    }
    if (rec) {
        u32 code, nparams;
        u64 addr, info0, info1;
        if (guest_read32(rec, &code) && guest_read64(rec + 0x10, &addr) && guest_read32(rec + 0x18, &nparams) &&
            guest_read64(rec + 0x20, &info0) && guest_read64(rec + 0x28, &info1)) {
            guest_name_or_hex(addr, n1, sizeof(n1));
            printf("HV exception: code 0x%x at %s nparams %u info[0] 0x%lx info[1] 0x%lx\n", code, n1, nparams,
                   info0, info1);
            if (code == 0xc0000005)
                printf("HV exception: access violation: %s address 0x%lx\n",
                       info0 == 0 ? "read" : info0 == 1 ? "write" : info0 == 8 ? "execute" : "?", info1);
        }
    }
    if (ctx) {
        /* ARM64 CONTEXT: ContextFlags, Cpsr, X0..X28 at +0x8, Fp +0xf0, Lr +0xf8, Sp +0x100, Pc +0x108 */
        u64 x[31], sp, pc;
        u32 cpsr;
        bool ok = guest_read32(ctx + 4, &cpsr) && guest_read64(ctx + 0x100, &sp) && guest_read64(ctx + 0x108, &pc);
        for (int i = 0; ok && i < 31; i++)
            ok = guest_read64(ctx + 8 + 8 * i, &x[i]);
        if (!ok) {
            printf("HV context: 0x%lx unreadable\n", ctx);
            return;
        }
        guest_name_or_hex(pc, n1, sizeof(n1));
        guest_name_or_hex(x[30], n2, sizeof(n2));
        printf("HV context: pc %s lr %s sp 0x%lx cpsr 0x%x\n", n1, n2, sp, cpsr);
        for (int i = 0; i < 30; i += 5)
            printf("HV context: x%-2d %016lx %016lx %016lx %016lx %016lx\n", i, x[i], x[i + 1], x[i + 2], x[i + 3],
                   x[i + 4]);
        u32 insn[6];
        bool code_ok = true;
        for (int i = 0; i < 6; i++)
            code_ok = code_ok && guest_read32(pc - 20 + 4 * i, &insn[i]);
        if (code_ok)
            printf("HV context: code %08x %08x %08x %08x %08x [%08x]\n", insn[0], insn[1], insn[2], insn[3], insn[4],
                   insn[5]);
        guest_fp_backtrace(x[29], x[30], 12);
        guest_stack_images_n(sp, 256, 10);
    }
}

void hv_standalone_dump_cpus(void)
{
    char name[128];
    u64 kbase = 0;
    for (int c = 0; c < MAX_CPUS; c++) {
        if (!cpu_last[c].n)
            continue;
        u64 pc = cpu_last[c].pc, spsr = cpu_last[c].spsr;
        if (guest_image_name(pc, name, sizeof(name)) < 0)
            snprintf(name, sizeof(name), "?");
        else if (!kbase && !strncmp(name, "ntkrnlmp", 8))
            kbase = guest_image_base_found;
        printf("HV cpu%d: EL%lu%s pc 0x%lx %s", c, (spsr >> 2) & 3, (spsr & 0x80) ? " I" : "", pc, name);
        printf(" fp 0x%lx pmc0 %lu MHz\n", cpu_last[c].fp, cpu_last[c].pmc0_hz / 1000000);
        guest_fp_backtrace(cpu_last[c].fp, cpu_last[c].lr, 24);
        /* Anything on the stack pointing outside the kernel: the faulting driver's return
         * addresses and EXCEPTION_RECORD.ExceptionAddress survive there even when the frame
         * chain stops at the trap frame. */
        if (pc >> 63)
            guest_stack_images_filtered((spsr & 1) ? cpu_last[c].sp1 : cpu_last[c].sp0, 1024, 8, "ntkrnlmp");
    }
    /*
     * The guest's view of the cycle counter on this CPU: the Apple PMU controls (PMCR1_EL12 is
     * the copy that applies while the guest runs at EL1/EL0) and what the kernel measured --
     * KPCR is in TPIDR_EL1, its KPRCB at +0x980, Prcb->MHz / CycleCounterFrequency at
     * +0x774 / +0x778 (ntkrnlmp 26300.9457; KiIntSteerSnapPerf divides by the latter).
     */
    printf("HV pmu(cpu%d): PMCR0 0x%lx PMCR1 0x%lx PMCR1_EL12 0x%lx PMC0 0x%lx\n", smp_id(), mrs(SYS_IMP_APL_PMCR0),
           mrs(SYS_IMP_APL_PMCR1), mrs(SYS_IMP_APL_PMCR1_EL12), mrs(SYS_IMP_APL_PMC0));
    u64 kpcr = mrs(TPIDR_EL1);
    if (kpcr >> 63) {
        u32 mhz;
        u64 ccf;
        if (guest_read32(kpcr + 0x980 + 0x774, &mhz) && guest_read64(kpcr + 0x980 + 0x778, &ccf))
            printf("HV prcb(cpu%d): KPCR 0x%lx MHz %u CycleCounterFrequency %lu\n", smp_id(), kpcr, mhz, ccf);
    }
    if (kbase)
        guest_dump_bugcheck(kbase);
}

static void die_dump(struct exc_info *ctx, const char *why)
{
    /* Take the screen back from the guest, keeping what it drew outside the console column */
    shadow_stop();
    if (!hv_verbose)
        fb_init(false);
    fb_set_active(true);
    for (int i = 0; i < MAX_CPUS; i++) {
        if (echo_len[i]) {
            echo_buf[i][echo_len[i]] = 0;
            printf("[cpu%d] %s\n", i, echo_buf[i]);
            echo_len[i] = 0;
        }
    }
    hv_input_stats();

    u64 *r = ctx->regs;
    int from_el = FIELD_GET(SPSR_M, ctx->spsr) >> 2;
    u64 elr_phys = hv_translate(ctx->elr, false, false, NULL);
    u64 far_phys = hv_translate(ctx->far, false, false, NULL);

    printf("\nHV: %s (cpu%lu)\n", why, ctx->cpu_id);
    printf("Guest EL%d  ELR 0x%lx (phys 0x%lx)  SPSR 0x%lx\n", from_el, ctx->elr, elr_phys,
           ctx->spsr);
    printf("ESR 0x%lx (EC 0x%lx ISS 0x%lx)  FAR 0x%lx (phys 0x%lx)  AFSR1 0x%lx\n", ctx->esr,
           FIELD_GET(ESR_EC, ctx->esr), FIELD_GET(ESR_ISS, ctx->esr), ctx->far, far_phys,
           ctx->afsr1);
    printf("SP_EL0 0x%lx  SP_EL1 0x%lx\n", ctx->sp[0], ctx->sp[1]);
    if (elr_phys && (elr_phys & 3) == 0 && hv_pt_is_ram(elr_phys & ~0x3fffULL))
        printf("Insn at ELR: %08x %08x %08x %08x (last is ELR)\n", read32(elr_phys - 12),
               read32(elr_phys - 8), read32(elr_phys - 4), read32(elr_phys));
    {
        char name[128];
        if (guest_image_name(ctx->elr, name, sizeof(name)) == 0)
            printf("ELR is in %s\n", name);
        guest_stack_images(ctx->sp[(ctx->spsr & 1) ? 1 : 0], 64);
    }
    for (int i = 0; i < 32; i += 4)
        printf("x%-2d %016lx %016lx %016lx %016lx\n", i, r[i], r[i + 1], r[i + 2], r[i + 3]);
    printf("HV: halted. Photograph this screen.\n");
    iodev_console_flush();

    while (1)
        sysop("wfe");
}

/*
 * Guest liveness sampler (hv=2). The host timer FIQ reaches EL2 whatever the guest's DAIF says,
 * so once a second we can show where the guest is -- including when it sits in its own EL1
 * exception handler with interrupts masked, which is exactly what bootmgfw does when it dies.
 */
static bool guest_in_kernel;

void hv_standalone_tick(struct exc_info *ctx)
{
    static u64 last_pc, same_count;
    static int ticks;

    /* shadow_tick() runs from hv_standalone_shadow_tick() before the big lock is taken */
    hv_input_tick();
    hv_standalone_note_cpu(smp_id(), ctx->elr, ctx->spsr, ctx->sp[1], ctx->sp[0], ctx->regs[29],
                           ctx->regs[30]);

    if (!hv_verbose || overlay_active)
        return;
    if (++ticks % hv_tick_hz)
        return;

    u64 pc = ctx->elr;

    /*
     * hv=2: the host console keeps the screen through firmware boot (its log is what we read).
     * Once the guest runs at kernel addresses the OS owns the screen: hand it over to the
     * shadow converter; the sampler keeps talking on the UART and comes back on screen only
     * when the guest looks stuck or dies.
     */
    if (!guest_in_kernel && (pc >> 63)) {
        guest_in_kernel = true;
        printf("HV: guest now runs at kernel addresses (pc 0x%lx)\n", pc);
        shadow_start();
    }
    /* A guest spinning in a small loop alternates between a few PCs: same 256-byte window = stuck */
    if ((pc & ~0xffULL) == (last_pc & ~0xffULL)) {
        same_count++;
        if (same_count != 3 && same_count != 30 && same_count % 300)
            return; /* only report a stuck PC a few times */
    } else {
        same_count = 0;
    }
    last_pc = pc;

    u64 vbar = mrs(VBAR_EL1), elr1 = mrs(ELR_EL1), esr1 = mrs(ESR_EL1), far1 = mrs(FAR_EL1);
    u64 spsr1 = mrs(SPSR_EL1);
    u64 pa = hv_translate(pc, false, false, NULL);
    u32 insn = 0;
    if (pa && hv_pt_is_ram(pa & ~0x3fffULL))
        insn = read32(pa);

    bool idle = pa && hv_pt_is_ram(pa & ~0x3fffULL) && read32(pa - 4) == 0xd503207f; /* after wfi */
    char image[128] = "?";
    guest_image_name(pc, image, sizeof(image));
    printf("HV sample: pc 0x%lx = %s (insn %08x) spsr 0x%lx%s%s | EL1 vbar 0x%lx elr 0x%lx esr 0x%lx "
           "far 0x%lx spsr 0x%lx\n",
           pc, image, insn, ctx->spsr, same_count ? " [unchanged]" : "",
           idle ? " [idle, after wfi]" : "", vbar, elr1, esr1, far1, spsr1);
    if (vbar && pc >= vbar && pc < vbar + 0x800)
        printf("HV sample: guest is inside its own exception vector (+0x%lx): it took an EL1 "
               "exception it cannot handle\n",
               pc - vbar);

    /* Stuck for a few seconds: dump the registers once (a 'b .' handler usually keeps
     * ELR/SPSR/ESR in low registers) and the code leading up to the PC. */
    /*
     * An OS idling in 'wfi' always gets sampled right after the wfi (our tick is what wakes
     * it), so an unchanged PC there means healthy and idle, not stuck (run 28: WinPE's command
     * prompt was on screen when this heuristic took it away). Never take the screen back from
     * a running kernel here; die_dump() does that when something is really wrong.
     */
    if (same_count == 3 && !guest_in_kernel) {
        u64 *r = ctx->regs;
        printf("HV sample: SP_EL0 0x%lx SP_EL1 0x%lx\n", ctx->sp[0], ctx->sp[1]);
        for (int i = 0; i < 32; i += 4)
            printf("HV sample: x%-2d %016lx %016lx %016lx %016lx\n", i, r[i], r[i + 1], r[i + 2],
                   r[i + 3]);
        if (insn) {
            printf("HV sample: code before pc:");
            for (int i = 8; i >= 1; i--)
                printf(" %08x", read32(pa - 4 * i));
            printf(" [%08x]\n", insn);
        }
        printf("HV sample: return addresses on the stack:\n");
        guest_stack_images(ctx->sp[(ctx->spsr & 1) ? 1 : 0], 96);
    }
}

void hv_standalone_exc(struct exc_info *ctx, uartproxy_boot_reason_t reason, u32 type, void *extra)
{
    UNUSED(extra);

    if (reason == START_EXCEPTION_LOWER) {
        if (type == EXC_SYNC && handle_sync(ctx))
            return;
        if (type == EXC_FIQ) {
            /* Host-side virtual timer / PMU FIQ leaking through: silence it */
            msr(CNTV_CTL_EL0, 0);
            printf("HV: stray FIQ in EL2, ignored\n");
            return;
        }
    } else if (reason == START_HV) {
        if (type == HV_VTIMER) {
            msr(CNTV_CTL_EL0, 0);
            return;
        }
        if (type == HV_USER_INTERRUPT)
            return;
    }

    char why[80];
    snprintf(why, sizeof(why), "unhandled guest exception %s/%s (no host to ask)",
             reason == START_HV ? "HV" : "EXC", reason_name(reason, type));
    die_dump(ctx, why);
}
