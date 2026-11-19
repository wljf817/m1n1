/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * See selfboot.h. Each function here says where it has its counterpart in
 * siliconnt/firmware/run_firmware.py or proxyclient/m1n1/hv/__init__.py.
 *
 * Tested on the build machine by siliconnt/selfboot-model.
 */

#include "selfboot.h"
#include "adt.h"
#include "string.h"
#include "types.h"
#include "utils.h"

#define MEMORY_MAP "/chosen/memory-map"

/* The primary GPT header is the second sector of a disk with 512-byte sectors */
#define GPT_OFFSET    512
#define GPT_SIGNATURE "EFI PART"

/* The spare entries iBoot leaves in /chosen/memory-map, base and size 0 */
#define SPARE_ENTRY "MemoryMapReserved-"
#define RAMDISK     "RAMDisk"

/* Without base + size, which need not fit in 64 bits */
static bool overlaps(u64 base, u64 size, u64 start, u64 end)
{
    if (!size || base >= end)
        return false;
    return base >= start || start - base < size;
}

/* Layout.__init__() of run_firmware.py, and the checks after it that do not need the files */
int selfboot_layout_init(struct selfboot_layout *l, u64 phys_base, const struct boot_args *ba)
{
    u64 fb_size = ALIGN_UP(ba->video.stride * ba->video.height, SELFBOOT_PAGE);
    int node;

    memset(l, 0, sizeof(*l));
    l->phys_base = phys_base;
    l->mem_top = ba->phys_base + ba->mem_size;
    l->adt_base = l->phys_base + SELFBOOT_GUEST_GAP;
    l->adt_size = ALIGN_UP((u64)ba->devtree_size, SELFBOOT_PAGE);
    l->image_base = ALIGN_UP(l->adt_base + l->adt_size, SELFBOOT_IMAGE_ALIGN);

    if (l->mem_top < SELFBOOT_UEFI_REGION || l->phys_base > l->mem_top - SELFBOOT_UEFI_REGION ||
        l->image_base + SELFBOOT_BOOTARGS_SIZE > l->mem_top - SELFBOOT_UEFI_REGION) {
        printf("selfboot: guest RAM 0x%lx .. 0x%lx has no room for the device tree, the "
               "boot arguments and the firmware's work area\n",
               l->phys_base, l->mem_top);
        return -1;
    }
    l->uefi_base = l->mem_top - SELFBOOT_UEFI_REGION;

    /* The firmware declares all of guest RAM as memory and maps the framebuffer separately */
    if (overlaps(ba->video.base, fb_size, l->phys_base, l->mem_top)) {
        printf("selfboot: the framebuffer, 0x%lx .. 0x%lx, lies inside guest RAM\n", ba->video.base,
               ba->video.base + fb_size);
        return -1;
    }

    /*
     * What iBoot left in memory. The files are not read yet, so this looks at all the room
     * they may take, where the host looks at what they do take.
     */
    node = adt_path_offset(adt, MEMORY_MAP);
    if (node < 0) {
        printf("selfboot: no " MEMORY_MAP " in the device tree\n");
        return -1;
    }
    ADT_FOREACH_PROPERTY(adt, node, prop)
    {
        u64 range[2];

        if (prop->size != sizeof(range))
            continue;
        memcpy(range, prop->value, sizeof(range));
        if (overlaps(range[0], range[1], l->adt_base, l->adt_base + l->adt_size) ||
            overlaps(range[0], range[1], l->image_base, l->mem_top)) {
            printf("selfboot: " MEMORY_MAP " entry %s, 0x%lx .. 0x%lx, is where the device "
                   "tree, the files or the firmware's work area go\n",
                   prop->name, range[0], range[0] + range[1]);
            return -1;
        }
    }

    return 0;
}

u64 selfboot_image_room(const struct selfboot_layout *l)
{
    return l->uefi_base - l->image_base - SELFBOOT_BOOTARGS_SIZE;
}

int selfboot_layout_image(struct selfboot_layout *l, u64 size)
{
    if (size > selfboot_image_room(l)) {
        printf("selfboot: the firmware image, 0x%lx bytes, reaches into the firmware's work "
               "area: there is room for 0x%lx\n",
               size, selfboot_image_room(l));
        return -1;
    }
    l->image_size = size;
    l->bootargs = l->image_base + ALIGN_UP(size, SELFBOOT_PAGE);
    l->disk_base = ALIGN_UP(l->bootargs + SELFBOOT_BOOTARGS_SIZE, SELFBOOT_DISK_ALIGN);
    return 0;
}

u64 selfboot_disk_room(const struct selfboot_layout *l)
{
    return l->disk_base < l->uefi_base ? l->uefi_base - l->disk_base : 0;
}

int selfboot_layout_disk(struct selfboot_layout *l, u64 size)
{
    if (size > selfboot_disk_room(l)) {
        printf("selfboot: the boot disk, 0x%lx bytes, reaches into the firmware's work area: "
               "there is room for 0x%lx\n",
               size, selfboot_disk_room(l));
        return -1;
    }
    l->disk_size = size;
    l->top = ALIGN_UP(l->disk_base + size, SELFBOOT_PAGE);
    return 0;
}

/* check_image() of run_firmware.py: the Linux arm64 image header at offset 0 */
int selfboot_check_image(const void *image, u64 size)
{
    const u8 *p = image;
    u32 code0, code1;
    u64 image_size, entry_code;

    if (size < 0x40) {
        printf("selfboot: firmware image: too short for an image header\n");
        return -1;
    }
    memcpy(&code0, p, 4);
    memcpy(&code1, p + 4, 4);
    memcpy(&image_size, p + 0x10, 8);
    if (memcmp(p + 0x38, "ARM\x64", 4)) {
        printf("selfboot: firmware image: no arm64 image magic at offset 0x38\n");
        return -1;
    }
    if (code0 != 0x10000001) {
        printf("selfboot: firmware image: the first instruction is not 'adr x1, .'; the entry "
               "code takes the load address from x1\n");
        return -1;
    }
    if (code1 >> 26 != 0x5) {
        printf("selfboot: firmware image: the second instruction is not a branch\n");
        return -1;
    }
    entry_code = 4 + ((u64)(code1 & 0x3ffffff) << 2);
    if (image_size != size) {
        printf("selfboot: firmware image: header says 0x%lx bytes, file has 0x%lx\n", image_size,
               size);
        return -1;
    }
    if (size % SELFBOOT_PAGE) {
        printf("selfboot: firmware image: size 0x%lx is not a multiple of 0x%lx\n", size,
               SELFBOOT_PAGE);
        return -1;
    }
    if (entry_code >= size) {
        printf("selfboot: firmware image: the header branches outside the image\n");
        return -1;
    }
    return 0;
}

/* check_disk() of run_firmware.py */
int selfboot_check_disk(const void *disk, u64 size)
{
    if (!size) {
        printf("selfboot: boot disk: empty\n");
        return -1;
    }
    if (size % SELFBOOT_DISK_PAGE) {
        printf("selfboot: boot disk: size 0x%lx is not a multiple of 0x%lx\n", size,
               SELFBOOT_DISK_PAGE);
        return -1;
    }
    if (memcmp((const u8 *)disk + GPT_OFFSET, GPT_SIGNATURE, strlen(GPT_SIGNATURE))) {
        printf("selfboot: boot disk: no GPT header: no '" GPT_SIGNATURE "' at offset %d\n",
               GPT_OFFSET);
        return -1;
    }
    return 0;
}

/* HV._write_boot_args() */
void selfboot_boot_args(struct boot_args *out, const struct boot_args *cur,
                        const struct selfboot_layout *l)
{
    memcpy(out, cur, sizeof(*out));
    out->mem_size = l->mem_top - l->phys_base;
    out->phys_base = l->phys_base;
    out->virt_base = 0xfffffe0010000000UL + (l->phys_base & ((32UL << 20) - 1));
    out->devtree = (void *)(l->adt_base - l->phys_base + out->virt_base);
    out->top_of_kernel_data = l->top;
}

/* A property of /chosen/memory-map in the copy at dest, by where it is in m1n1's tree */
static struct adt_property *copy_of(void *dest, const struct adt_property *prop)
{
    return (struct adt_property *)((u8 *)dest + ((const u8 *)prop - (const u8 *)adt));
}

static int set_range(void *dest, int node, const char *name, u64 base, u64 size)
{
    const struct adt_property *prop = adt_get_property(adt, node, name);
    u64 range[2] = {base, size};

    if (!prop || prop->size != sizeof(range)) {
        printf("selfboot: no entry %s of 16 bytes in " MEMORY_MAP "\n", name);
        return -1;
    }
    memcpy(copy_of(dest, prop)->value, range, sizeof(range));
    return 0;
}

/*
 * The entries load_firmware() of run_firmware.py sets. There the tree is taken apart and
 * built again, with RAMDisk as a new entry. Here it stays as it is: one of the spare entries
 * iBoot leaves, base and size 0, gets the name and the range. The functions of adt.h look at
 * m1n1's tree only, so that is where the entries are looked up; the copy is what is written.
 */
int selfboot_adt(void *dest, u32 size, const struct selfboot_layout *l)
{
    int node = adt_path_offset(adt, MEMORY_MAP);
    if (node < 0) {
        printf("selfboot: no " MEMORY_MAP " in the device tree\n");
        return -1;
    }

    memcpy(dest, adt, size);

    if (set_range(dest, node, "DeviceTree", l->adt_base, l->adt_size) < 0 ||
        set_range(dest, node, "BootArgs", l->bootargs, SELFBOOT_BOOTARGS_SIZE) < 0)
        return -1;

    return selfboot_adt_add_range(dest, RAMDISK, l->disk_base, l->disk_size);
}

int selfboot_adt_add_range(void *dest, const char *name, u64 base, u64 bytes)
{
    int node = adt_path_offset(adt, MEMORY_MAP);
    struct adt_property *ramdisk = NULL;
    u64 range[2];
    if (node < 0 || strlen(name) >= sizeof(ramdisk->name))
        return -1;
    ADT_FOREACH_PROPERTY(adt, node, prop)
    {
        struct adt_property *candidate = copy_of(dest, prop);
        if (strncmp(candidate->name, SPARE_ENTRY, strlen(SPARE_ENTRY)) || prop->size != sizeof(range))
            continue;
        memcpy(range, candidate->value, sizeof(range));
        if (range[0] || range[1])
            continue;
        ramdisk = copy_of(dest, prop);
        break;
    }
    if (!ramdisk) {
        printf("selfboot: no spare entry in " MEMORY_MAP " for the boot disk\n");
        return -1;
    }

    memset(ramdisk->name, 0, sizeof(ramdisk->name));
    strcpy(ramdisk->name, name);
    /* The size in bytes, not rounded: it is the size of the disk */
    range[0] = base;
    range[1] = bytes;
    memcpy(ramdisk->value, range, sizeof(range));
    return 0;
}
