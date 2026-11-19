/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "../build/build_cfg.h"
#include "selfboot.h"
#include "utils.h"

#ifdef CHAINLOADING
int rust_read_file(const char *part_uuid, const char *path, void *dest, size_t max_size,
                   size_t *size);

int selfboot_read_files(struct selfboot_layout *l, const char *part_uuid,
                        int (*check_disk)(const void *, u64))
{
    size_t size;

    if (rust_read_file(part_uuid, SELFBOOT_IMAGE_FILE, (void *)l->image_base,
                       selfboot_image_room(l), &size) < 0 ||
        selfboot_layout_image(l, size) < 0 || selfboot_check_image((void *)l->image_base, size) < 0)
        return -1;
    printf("selfboot: " SELFBOOT_IMAGE_FILE ": 0x%lx bytes\n", size);

    if (rust_read_file(part_uuid, SELFBOOT_DISK_FILE, (void *)l->disk_base, selfboot_disk_room(l),
                       &size) < 0 ||
        selfboot_layout_disk(l, size) < 0 || check_disk((void *)l->disk_base, size) < 0)
        return -1;
    printf("selfboot: " SELFBOOT_DISK_FILE ": 0x%lx bytes\n", size);
    return 0;
}

#endif
