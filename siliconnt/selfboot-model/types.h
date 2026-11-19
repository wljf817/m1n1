/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Stand-in for the m1n1 header of the same name, see test.c */

#ifndef TYPES_H
#define TYPES_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
/* Match the AArch64 target's LP64 types on Linux and Darwin hosts. */
typedef unsigned long u64;
typedef long s64;
_Static_assert(sizeof(u64) == 8, "the host model requires LP64");
#define PACKED __attribute__((packed))
/* As in src/types.h */
#define BIT(x)            (1UL << (x))
#define GENMASK(msb, lsb) ((BIT((msb + 1) - (lsb)) - 1) << (lsb))
/* A system register is just a number here */
#define sys_reg(op0, op1, CRn, CRm, op2)                                                           \
    (((op0) << 14) | ((op1) << 11) | ((CRn) << 7) | ((CRm) << 3) | (op2))
#endif
