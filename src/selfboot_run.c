/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * See selfboot.h. Each step here says where it has its counterpart in
 * siliconnt/firmware/run_firmware.py or proxyclient/m1n1/hv/__init__.py (HV), and where it
 * differs from it. Only what the M4 (T8132) needs is here.
 *
 * Everything that can fail for a reason outside m1n1 (the SSD, the two files, the layout)
 * comes before the hypervisor is initialized, and m1n1 keeps the screen and its consoles
 * until nothing can fail any more.
 *
 * Tested on the build machine by siliconnt/selfboot-model (run_test.c).
 */

#include "../build/build_cfg.h"

#include "selfboot.h"
#include "adt.h"
#include "cpu_regs.h"
#include "fb.h"
#include "heapblock.h"
#include "hv.h"
#include "iodev.h"
#include "mcc.h"
#include "memory.h"
#include "nvme.h"
#include "string.h"
#include "types.h"
#include "utils.h"
#include "xnuboot.h"

#define ARM_IO "/arm-io"
#define UART   "/arm-io/uart0"

/*
 * What HV.init() puts into HACR_EL2, by the names of class HACR of
 * proxyclient/m1n1/sysreg.py: TRAP_CTRR, TRAP_IPI, TRAP_SERROR_INFO, TRAP_EHID, TRAP_HID,
 * TRAP_ACC, TRAP_PM
 */
#define HACR_TRAPS (BIT(14) | BIT(16) | BIT(48) | BIT(49) | BIT(50) | BIT(52) | BIT(57))
/* class ACTLR of the same file */
#define ACTLR_EnMDSB BIT(12)

/* No host: not a port that usb_iodev_vuart_setup() of src/usb.c takes */
#define NO_HOST IODEV_NUM

/* rust/src/readfile.rs, in a build with CHAINLOADING */
/* What follows hv_init() in HV.init(), as far as the M4 has it */
static void selfboot_regs(void)
{
    hv_write_hcr((mrs(HCR_EL2) & ~(HCR_TIDCP | HCR_TVM)) | HCR_TACR | HCR_FMO | HCR_IMO |
                 HCR_TTLBOS);
    hv_write_hacr(HACR_TRAPS);

    /*
     * The guest's accesses to the debug registers come to EL2, where hv_dbg.c and hv_exc.c
     * answer them. The host also sets TDE, which sends the guest's debug exceptions to EL2,
     * BRK among them (Arm DDI 0487 M.d, D2.2), and hands each one back to the guest itself
     * (HV.handle_brk(), HV._lower()). Without TDE they go to the guest directly.
     */
    msr(MDCR_EL2, MDCR_TDRA | MDCR_TDOSA | MDCR_TDA);
    /*
     * The host sets MDE for breakpoints of its own. Here nothing is to raise a breakpoint or
     * watchpoint exception, whatever the registers of the CPU hold (D24.3.20, MDE).
     */
    msr(SYS_MDSCR_EL1, 0);

    /*
     * The host does this on every machine; the guest's own accesses go through hv_exc.c.
     * The AMX and key registers HV.init() also sets are not touched: the M4 does not have
     * Apple's registers unlocked (features_m4, src/chickens.c).
     */
    msr(SYS_ACTLR_EL12, mrs(SYS_ACTLR_EL12) | ACTLR_EnMDSB);
    sysop("isb");
}

static u64 cells(const u8 *p, u32 count)
{
    u64 value = 0;

    memcpy(&value, p, 4 * count);
    return value;
}

static int map_hw(const char *what, u64 base, u64 size)
{
    if (hv_map_hw(base, base, size) < 0) {
        printf("selfboot: %s, 0x%lx .. 0x%lx, cannot be mapped for the guest\n", what, base,
               base + size);
        return -1;
    }
    return 0;
}

/*
 * The stage 2 mappings HV.pt_update() makes from what HV.init(), HV._prepare_load() and
 * HV.map_vuart() ask for. The registers the host keeps to itself in HV.map_essential()
 * (power states of the port it talks over, the start of the other cores, their reset
 * vectors) are the devices they are here: there is no host to answer for them.
 */
static int selfboot_map(const struct selfboot_layout *l, iodev_id_t host)
{
    u64 ram_base = l->phys_base & ~0xffffffffUL;
    u32 ac, sc, pac, len, entry, irq;
    const u8 *ranges;
    int path[8];
    u64 uart;
    int node;

    /* Every range of /arm-io: bus address, address of the CPU, size */
    node = adt_path_offset(adt, ARM_IO);
    ranges = node < 0 ? NULL : adt_getprop(adt, node, "ranges", &len);
    if (!ranges || ADT_GETPROP(adt, node, "#address-cells", &ac) < 0 ||
        ADT_GETPROP(adt, node, "#size-cells", &sc) < 0 ||
        ADT_GETPROP(adt, 0, "#address-cells", &pac) < 0) {
        printf("selfboot: " ARM_IO " of the device tree has no ranges to map\n");
        return -1;
    }
    if (!ac || ac > 2 || !sc || sc > 2 || !pac || pac > 2) {
        printf("selfboot: the ranges of " ARM_IO " have addresses of %d and %d cells and sizes "
               "of %d\n",
               ac, pac, sc);
        return -1;
    }
    entry = 4 * (ac + pac + sc);
    for (u32 off = 0; off + entry <= len; off += entry)
        if (map_hw("a range of " ARM_IO, cells(ranges + off + 4 * ac, pac),
                   cells(ranges + off + 4 * (ac + pac), sc)) < 0)
            return -1;

    /*
     * Memory: what lies below iBoot's, and from the start of guest RAM to the end of all
     * there is. In between are m1n1 and its heap.
     */
    if (map_hw("the memory below iBoot's", ram_base, cur_boot_args.phys_base - ram_base) < 0 ||
        map_hw("guest RAM", l->phys_base, ram_base + mem_size_actual - l->phys_base) < 0)
        return -1;
    for (size_t i = 0; i < mcc_carveout_count; i++)
        if (hv_unmap(mcc_carveouts[i].base, mcc_carveouts[i].size) < 0) {
            printf("selfboot: the carveout at 0x%lx cannot be taken from the guest\n",
                   mcc_carveouts[i].base);
            return -1;
        }

    /* The virtual UART in place of the real one, after the range it lies in */
    if (adt_path_offset_trace(adt, UART, path) < 0 ||
        adt_get_reg(adt, path, "reg", 0, &uart, NULL) < 0 ||
        ADT_GETPROP(adt, adt_path_offset(adt, UART), "interrupts", &irq) < 0) {
        printf("selfboot: no " UART " with an address and an interrupt in the device tree\n");
        return -1;
    }
    hv_map_vuart(uart, irq, host);
    return 0;
}

static void print_layout(const struct selfboot_layout *l)
{
    printf("selfboot: guest RAM 0x%lx .. 0x%lx\n", l->phys_base, l->mem_top);
    printf("  device tree        0x%lx .. 0x%lx\n", l->adt_base, l->adt_base + l->adt_size);
    printf("  firmware image     0x%lx .. 0x%lx\n", l->image_base, l->image_base + l->image_size);
    printf("  boot arguments     0x%lx .. 0x%lx\n", l->bootargs,
           l->bootargs + SELFBOOT_BOOTARGS_SIZE);
    printf("  boot disk          0x%lx .. 0x%lx\n", l->disk_base, l->disk_base + l->disk_size);
    printf("  firmware work area 0x%lx .. 0x%lx\n", l->uefi_base, l->mem_top);
}

/*
 * The two files are in guest memory and the layout is complete. host: the port a host talks
 * to m1n1 over, or NO_HOST.
 */
static int selfboot_enter(const struct selfboot_layout *l, iodev_id_t host)
{
    u64 regs[4] = {l->bootargs, 0, 0, 0};

    print_layout(l);

    /* run_firmware.py: the entry code runs with the MMU of EL1 off until it has its tables */
    if (mrs(SYS_SCTLR_EL12) & SCTLR_M) {
        printf("selfboot: the MMU of EL1 is on, the firmware expects it off\n");
        return -1;
    }

    /* The device tree and the boot arguments: load_firmware() and HV.start() */
    if (selfboot_adt((void *)l->adt_base, cur_boot_args.devtree_size, l) < 0)
        return -1;
    memset((void *)l->bootargs, 0, SELFBOOT_BOOTARGS_SIZE);
    selfboot_boot_args((struct boot_args *)l->bootargs, &cur_boot_args, l);

    /* HV._load_executable_raw() */
    dc_cvau_range((void *)l->image_base, l->image_size);
    ic_ivau_range((void *)l->image_base, l->image_size);

    /*
     * HV.init(). The USB ports nobody talks over stop being m1n1's first: hv_init() gives the
     * interrupts of their port controllers back, for the guest (usb_hpm_restore_irqs() of
     * src/usb.c). The other cores are left alone: hv_init() does not start them.
     */
    for (int i = IODEV_USB0; i < IODEV_MAX; i++)
        if (i != (int)host)
            iodev_set_usage(i, 0);
    hv_init();
    selfboot_regs();
    if (selfboot_map(l, host) < 0)
        return -1;

    /*
     * Execution tracking, when all the memory is mapped: this is where it is taken apart
     * into pages (hv_pt_xtrack() of src/hv/hv_vm.c). run_firmware.py turns it on before
     * HV.start() makes the mappings.
     */
    if (hv_pt_xtrack() < 0) {
        printf("selfboot: the hypervisor refused to track execution\n");
        return -1;
    }

    /*
     * HV.start(): from here on m1n1 prints to the host only, if there is one, and the screen
     * is the guest's.
     */
    printf("selfboot: entering the firmware at 0x%lx, x0 = 0x%lx\n", l->image_base, l->bootargs);
    for (int i = 0; i < IODEV_USB0; i++)
        if (i != (int)host)
            iodev_set_usage(i, 0);
    fb_shutdown(true);

    hv_start((void *)l->image_base, regs);
    return 0;
}

#ifdef CHAINLOADING

int selfboot_run(const char *part_uuid)
{
    /* Where ProxyUtils of the host puts its own heap: here guest RAM starts there */
    u64 phys_base = ALIGN_UP((u64)heapblock_alloc(0) + SELFBOOT_M1N1_HEAP, SELFBOOT_PAGE);
    struct selfboot_layout l;
    int ret;

    printf("selfboot: the firmware and the boot disk are on partition %s\n", part_uuid);
    if (selfboot_layout_init(&l, phys_base, &cur_boot_args) < 0)
        return -1;
    heapblock_set_limit((void *)phys_base);

    if (!nvme_init()) {
        printf("selfboot: the SSD controller did not come up\n");
        return -1;
    }
    ret = selfboot_read_files(&l, part_uuid, selfboot_check_disk);
    nvme_shutdown();
    if (ret < 0)
        return -1;

    return selfboot_enter(&l, NO_HOST);
}

#endif

int selfboot_run_loaded(u64 phys_base, u64 image_size, u64 disk_size, int host_iodev)
{
    struct selfboot_layout l;

    if (phys_base % SELFBOOT_PAGE || phys_base < (u64)heapblock_alloc(0)) {
        printf("selfboot: guest RAM cannot start at 0x%lx: not a page above m1n1's heap\n",
               phys_base);
        return -1;
    }
    if (selfboot_layout_init(&l, phys_base, &cur_boot_args) < 0 ||
        selfboot_layout_image(&l, image_size) < 0 ||
        selfboot_check_image((void *)l.image_base, image_size) < 0 ||
        selfboot_layout_disk(&l, disk_size) < 0 ||
        selfboot_check_disk((void *)l.disk_base, disk_size) < 0)
        return -1;

    return selfboot_enter(&l, host_iodev);
}
