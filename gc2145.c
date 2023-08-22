// SPDX-License-Identifier: GPL-2.0
/*
 * GC2145 CMOS Image Sensor driver
 *
 * Inspired from the imx219.c driver
 *
 * Copyright (C) 2023 YH System Design Full Solution
 *
 * YH Chiu <chiuyungho@gmail.com>
 *
 * This program is free software; you may redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#ifndef _MEDIA_MIPI_CSI2_H
#define _MEDIA_MIPI_CSI2_H

/* Short packet data types */
#define MIPI_CSI2_DT_FS   0x00
#define MIPI_CSI2_DT_FE   0x01
#define MIPI_CSI2_DT_LS   0x02
#define MIPI_CSI2_DT_LE   0x03
#define MIPI_CSI2_DT_GENERIC_SHORT(n) (0x08 + (n)) /* 0..7 */

/* Long packet data types */
#define MIPI_CSI2_DT_NULL               0x10
#define MIPI_CSI2_DT_BLANKING           0x11
#define MIPI_CSI2_DT_EMBEDDED_8B        0x12
#define MIPI_CSI2_DT_YUV420_8B          0x18
#define MIPI_CSI2_DT_YUV420_10B         0x19
#define MIPI_CSI2_DT_YUV420_8B_LEGACY   0x1a
#define MIPI_CSI2_DT_YUV420_8B_CS       0x1c
#define MIPI_CSI2_DT_YUV420_10B_CS      0x1d
#define MIPI_CSI2_DT_YUV422_8B          0x1e
#define MIPI_CSI2_DT_YUV422_10B         0x1f
#define MIPI_CSI2_DT_RGB444             0x20
#define MIPI_CSI2_DT_RGB555             0x21
#define MIPI_CSI2_DT_RGB565             0x22
#define MIPI_CSI2_DT_RGB666             0x23
#define MIPI_CSI2_DT_RGB888             0x24
#define MIPI_CSI2_DT_RAW24              0x27
#define MIPI_CSI2_DT_RAW6               0x28
#define MIPI_CSI2_DT_RAW7               0x29
#define MIPI_CSI2_DT_RAW8               0x2a
#define MIPI_CSI2_DT_RAW10              0x2b
#define MIPI_CSI2_DT_RAW12              0x2c
#define MIPI_CSI2_DT_RAW14              0x2d
#define MIPI_CSI2_DT_RAW16              0x2e
#define MIPI_CSI2_DT_RAW20              0x2f
#define MIPI_CSI2_DT_USER_DEFINED(n)    (0x30 + (n)) /* 0..7 */

#endif /* _MEDIA_MIPI_CSI2_H */

#define GC2145_DEBUG_MSG

#define GC2145_CHIP_ID      0x2145
#define GC2145_ADDR_WRITE   0x78
#define GC2145_ADDR_READ    0x79

#define GC2145_XCLK_MIN     6000000
#define GC2145_XCLK_MAX     48000000
#define GC2145_PIXEL_RATE   (120 * 1000 * 1000)

/* Page 0 */
enum {
    GC2145_REG_OUTPUT_FORMAT = 0x84,
    GC2145_REG_CHIP_ID_H = 0xF0,
    GC2145_REG_CHIP_ID_L = 0xF1,
    GC2145_REG_PAD_MODE = 0xF2,
    GC2145_REG_PAGE_SELECT = 0xFE,
    GC2145_REG_NULL = 0xFF, /* Array end token */
};

enum {
    GC2145_OUTPUT_FMT_UYVY = 0x00,
    GC2145_OUTPUT_FMT_VYUY = 0x01,
    GC2145_OUTPUT_FMT_YUYV = 0x02,
    GC2145_OUTPUT_FMT_YVYU = 0x03,
    GC2145_OUTPUT_FMT_RGB = 0x06,
    GC2145_OUTPUT_FMT_DNDD = 0x18,
    GC2145_OUTPUT_FMT_LSC = 0x19,
};

struct gc2145_reg {
    unsigned char addr;
    unsigned char val;
};

static struct gc2145_reg gc2145_init_regs[] = {
    {0xfe, 0xf0}, // Reset
    {0xfe, 0xf0},
    {0xfe, 0xf0},
    {0xfc, 0x06},
    {0xf6, 0x00},
    {0xf7, 0x1d},
    {0xf8, 0x83}, // {0xf8, 0x84},
    {0xfa, 0x00},
    {0xf9, 0xfe},
    {0xf2, 0x00},

    {0xfe, 0x00}, // Select bank0
    {0x03, 0x04}, // CHECK: Read only?
    {0x04, 0xe2}, // CHECK: Read only?
    {0x09, 0x00},
    {0x0a, 0x00},
    {0x0b, 0x00},
    {0x0c, 0x00},
    {0x0d, 0x04},
    {0x0e, 0xc0},
    {0x0f, 0x06},
    {0x10, 0x52},
    {0x12, 0x2e},
    {0x17, 0x17}, // {0x17, 0x14},
    {0x18, 0x22},
    {0x19, 0x0e},
    {0x1a, 0x01},
    {0x1b, 0x4b},
    {0x1c, 0x07},
    {0x1d, 0x10},
    {0x1e, 0x88},
    {0x1f, 0x78},
    {0x20, 0x03},
    {0x21, 0x40},
    {0x22, 0xa0},
    {0x24, 0x16},
    {0x25, 0x01},
    {0x26, 0x10},
    {0x2d, 0x60},
    {0x30, 0x01},
    {0x31, 0x90},
    {0x33, 0x06},
    {0x34, 0x01},

    /* ISP */
    {0xfe, 0x00}, // Select bank0
    {0x80, 0x7f},
    {0x81, 0x26},
    {0x82, 0xfa},
    {0x83, 0x00},
    {0x84, 0x02}, // {0x84, 0x00},
    {0x86, 0x03}, // {0x86, 0x02},
    {0x88, 0x03},
    {0x89, 0x03},
    {0x85, 0x08},
    {0x8a, 0x00},
    {0x8b, 0x00},
    {0xb0, 0x55},
    {0xc3, 0x00},
    {0xc4, 0x80},
    {0xc5, 0x90},
    {0xc6, 0x3b},
    {0xc7, 0x46},
    {0xec, 0x06},
    {0xed, 0x04},
    {0xee, 0x60},
    {0xef, 0x90},
    {0xb6, 0x01},
    {0x90, 0x01},
    {0x91, 0x00},
    {0x92, 0x00},
    {0x93, 0x00},
    {0x94, 0x00},
    {0x95, 0x04},
    {0x96, 0xb0},
    {0x97, 0x06},
    {0x98, 0x40},
    /* BLK */
    {0xfe, 0x00}, // Select bank0
    {0x40, 0x42},
    {0x41, 0x00},
    {0x43, 0x5b},
    {0x5e, 0x00},
    {0x5f, 0x00},
    {0x60, 0x00},
    {0x61, 0x00},
    {0x62, 0x00},
    {0x63, 0x00},
    {0x64, 0x00},
    {0x65, 0x00},
    {0x66, 0x20},
    {0x67, 0x20},
    {0x68, 0x20},
    {0x69, 0x20},
    {0x76, 0x00},
    {0x6a, 0x08},
    {0x6b, 0x08},
    {0x6c, 0x08},
    {0x6d, 0x08},
    {0x6e, 0x08},
    {0x6f, 0x08},
    {0x70, 0x08},
    {0x71, 0x08},
    {0x76, 0x00},
    {0x72, 0xf0},
    {0x7e, 0x3c},
    {0x7f, 0x00},
    {0xfe, 0x02}, // Select bank2
    {0x48, 0x15},
    {0x49, 0x00},
    {0x4b, 0x0b},
    {0xfe, 0x00}, // Select bank0
    /* AEC */
    {0xfe, 0x01}, // Select bank1
    {0x01, 0x04},
    {0x02, 0xc0},
    {0x03, 0x04},
    {0x04, 0x90},
    {0x05, 0x30},
    {0x06, 0x90},
    {0x07, 0x30},
    {0x08, 0x80},
    {0x09, 0x00},
    {0x0a, 0x82},
    {0x0b, 0x11},
    {0x0c, 0x10},
    {0x11, 0x10},
    {0x13, 0x7b},
    {0x17, 0x00},
    {0x1c, 0x11},
    {0x1e, 0x61},
    {0x1f, 0x35},
    {0x20, 0x40},
    {0x22, 0x40},
    {0x23, 0x20},
    {0xfe, 0x02}, // Select bank2
    {0x0f, 0x04},
    {0xfe, 0x01}, // Select bank1
    {0x12, 0x35},
    {0x15, 0xb0},
    {0x10, 0x31},
    {0x3e, 0x28},
    {0x3f, 0xb0},
    {0x40, 0x90},
    {0x41, 0x0f},
    /* INTPEE */
    {0xfe, 0x02}, // Select bank2
    {0x90, 0x6c},
    {0x91, 0x03},
    {0x92, 0xcb},
    {0x94, 0x33},
    {0x95, 0x84},
    {0x97, 0x65}, // {0x97, 0x45},
    {0xa2, 0x11},
    {0xfe, 0x00}, // Select bank0
    /* DNDD */
    {0xfe, 0x02}, // Select bank2
    {0x80, 0xc1},
    {0x81, 0x08},
    {0x82, 0x05}, // {0x82, 0x1f},
    {0x83, 0x08}, // {0x83, 0x10},
    {0x84, 0x0a},
    {0x86, 0xf0},
    {0x87, 0x50},
    {0x88, 0x15},
    {0x89, 0xb0},
    {0x8a, 0x30},
    {0x8b, 0x10},
    /* ASDE */
    {0xfe, 0x01}, // Select bank1
    {0x21, 0x04},
    {0xfe, 0x02}, // Select bank2
    {0xa3, 0x50},
    {0xa4, 0x20},
    {0xa5, 0x40},
    {0xa6, 0x80},
    {0xab, 0x40},
    {0xae, 0x0c},
    {0xb3, 0x46},
    {0xb4, 0x64},
    {0xb6, 0x38},
    {0xb7, 0x01},
    {0xb9, 0x2b},
    {0x3c, 0x04},
    {0x3d, 0x15},
    {0x4b, 0x06},
    {0x4c, 0x20},
    {0xfe, 0x00}, // Select bank0
    /* GAMMA */
    /* gamma1 */
#if 1
    {0xfe, 0x02}, // Select bank2
    {0x10, 0x09},
    {0x11, 0x0d},
    {0x12, 0x13},
    {0x13, 0x19},
    {0x14, 0x27},
    {0x15, 0x37},
    {0x16, 0x45},
    {0x17, 0x53},
    {0x18, 0x69},
    {0x19, 0x7d},
    {0x1a, 0x8f},
    {0x1b, 0x9d},
    {0x1c, 0xa9},
    {0x1d, 0xbd},
    {0x1e, 0xcd},
    {0x1f, 0xd9},
    {0x20, 0xe3},
    {0x21, 0xea},
    {0x22, 0xef},
    {0x23, 0xf5},
    {0x24, 0xf9},
    {0x25, 0xff},
#else
    {0xfe, 0x02}, // Select bank2
    {0x10, 0x0a},
    {0x11, 0x12},
    {0x12, 0x19},
    {0x13, 0x1f},
    {0x14, 0x2c},
    {0x15, 0x38},
    {0x16, 0x42},
    {0x17, 0x4e},
    {0x18, 0x63},
    {0x19, 0x76},
    {0x1a, 0x87},
    {0x1b, 0x96},
    {0x1c, 0xa2},
    {0x1d, 0xb8},
    {0x1e, 0xcb},
    {0x1f, 0xd8},
    {0x20, 0xe2},
    {0x21, 0xe9},
    {0x22, 0xf0},
    {0x23, 0xf8},
    {0x24, 0xfd},
    {0x25, 0xff},
    {0xfe, 0x00}, // Select bank0
#endif 
    {0xfe, 0x00}, // Select bank0
    {0xc6, 0x20},
    {0xc7, 0x2b},
    /* gamma2 */
#if 1
    {0xfe, 0x02}, // Select bank2
    {0x26, 0x0f},
    {0x27, 0x14},
    {0x28, 0x19},
    {0x29, 0x1e},
    {0x2a, 0x27},
    {0x2b, 0x33},
    {0x2c, 0x3b},
    {0x2d, 0x45},
    {0x2e, 0x59},
    {0x2f, 0x69},
    {0x30, 0x7c},
    {0x31, 0x89},
    {0x32, 0x98},
    {0x33, 0xae},
    {0x34, 0xc0},
    {0x35, 0xcf},
    {0x36, 0xda},
    {0x37, 0xe2},
    {0x38, 0xe9},
    {0x39, 0xf3},
    {0x3a, 0xf9},
    {0x3b, 0xff},
#else
    /* Gamma outdoor */
    {0xfe, 0x02}, // Select bank2
    {0x26, 0x17},
    {0x27, 0x18},
    {0x28, 0x1c},
    {0x29, 0x20},
    {0x2a, 0x28},
    {0x2b, 0x34},
    {0x2c, 0x40},
    {0x2d, 0x49},
    {0x2e, 0x5b},
    {0x2f, 0x6d},
    {0x30, 0x7d},
    {0x31, 0x89},
    {0x32, 0x97},
    {0x33, 0xac},
    {0x34, 0xc0},
    {0x35, 0xcf},
    {0x36, 0xda},
    {0x37, 0xe5},
    {0x38, 0xec},
    {0x39, 0xf8},
    {0x3a, 0xfd},
    {0x3b, 0xff},
#endif
    /* YCP */
    {0xfe, 0x02}, // Select bank2
    {0xd1, 0x32},
    {0xd2, 0x32},
    {0xd3, 0x40},
    {0xd6, 0xf0},
    {0xd7, 0x10},
    {0xd8, 0xda},
    {0xdd, 0x14},
    {0xde, 0x86},
    {0xed, 0x80},
    {0xee, 0x00},
    {0xef, 0x3f},
    {0xd8, 0xd8},
    /* ABS */
    {0xfe, 0x01}, // Select bank1
    {0x9f, 0x40},
    /* Lens Shading Correction */
    {0xfe, 0x01}, // Select bank1
    {0xc2, 0x14},
    {0xc3, 0x0d},
    {0xc4, 0x0c},
    {0xc8, 0x15},
    {0xc9, 0x0d},
    {0xca, 0x0a},
    {0xbc, 0x24},
    {0xbd, 0x10},
    {0xbe, 0x0b},
    {0xb6, 0x25},
    {0xb7, 0x16},
    {0xb8, 0x15},
    {0xc5, 0x00},
    {0xc6, 0x00},
    {0xc7, 0x00},
    {0xcb, 0x00},
    {0xcc, 0x00},
    {0xcd, 0x00},
    {0xbf, 0x07},
    {0xc0, 0x00},
    {0xc1, 0x00},
    {0xb9, 0x00},
    {0xba, 0x00},
    {0xbb, 0x00},
    {0xaa, 0x01},
    {0xab, 0x01},
    {0xac, 0x00},
    {0xad, 0x05},
    {0xae, 0x06},
    {0xaf, 0x0e},
    {0xb0, 0x0b},
    {0xb1, 0x07},
    {0xb2, 0x06},
    {0xb3, 0x17},
    {0xb4, 0x0e},
    {0xb5, 0x0e},
    {0xd0, 0x09},
    {0xd1, 0x00},
    {0xd2, 0x00},
    {0xd6, 0x08},
    {0xd7, 0x00},
    {0xd8, 0x00},
    {0xd9, 0x00},
    {0xda, 0x00},
    {0xdb, 0x00},
    {0xd3, 0x0a},
    {0xd4, 0x00},
    {0xd5, 0x00},
    {0xa4, 0x00},
    {0xa5, 0x00},
    {0xa6, 0x77},
    {0xa7, 0x77},
    {0xa8, 0x77},
    {0xa9, 0x77},
    {0xa1, 0x80},
    {0xa2, 0x80},

    {0xfe, 0x01}, // Select bank1
    {0xdf, 0x0d},
    {0xdc, 0x25},
    {0xdd, 0x30},
    {0xe0, 0x77},
    {0xe1, 0x80},
    {0xe2, 0x77},
    {0xe3, 0x90},
    {0xe6, 0x90},
    {0xe7, 0xa0},
    {0xe8, 0x90},
    {0xe9, 0xa0},
    {0xfe, 0x00}, // Select bank0
    /* Auto White Balance */
    {0xfe, 0x01},  // Select bank1
    {0x4f, 0x00},
    {0x4f, 0x00},
    {0x4b, 0x01},
    {0x4f, 0x00},
                                        
    {0x4c, 0x01}, // D75
    {0x4d, 0x71},
    {0x4e, 0x01},
    {0x4c, 0x01},
    {0x4d, 0x91},
    {0x4e, 0x01},
    {0x4c, 0x01},
    {0x4d, 0x70},
    {0x4e, 0x01},
    {0x4c, 0x01}, // D65
    {0x4d, 0x90},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0xb0},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0x8f},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0x6f},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0xaf},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0xd0},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0xf0},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0xcf},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0xef},
    {0x4e, 0x02},
    {0x4c, 0x01}, // D50
    {0x4d, 0x6e},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x8e},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xae},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xce},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x4d},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x6d},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x8d},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xad},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xcd},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x4c},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x6c},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x8c},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xac},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xcc},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xcb},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x4b},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x6b},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x8b},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xab},
    {0x4e, 0x03},
    {0x4c, 0x01}, // CWF
    {0x4d, 0x8a},
    {0x4e, 0x04},
    {0x4c, 0x01},
    {0x4d, 0xaa},
    {0x4e, 0x04},
    {0x4c, 0x01},
    {0x4d, 0xca},
    {0x4e, 0x04},
    {0x4c, 0x01},
    {0x4d, 0xca},
    {0x4e, 0x04},
    {0x4c, 0x01},
    {0x4d, 0xc9},
    {0x4e, 0x04},
    {0x4c, 0x01},
    {0x4d, 0x8a},
    {0x4e, 0x04},
    {0x4c, 0x01},
    {0x4d, 0x89},
    {0x4e, 0x04},
    {0x4c, 0x01},
    {0x4d, 0xa9},
    {0x4e, 0x04},
    {0x4c, 0x02}, // tl84
    {0x4d, 0x0b},
    {0x4e, 0x05},
    {0x4c, 0x02},
    {0x4d, 0x0a},
    {0x4e, 0x05},
    {0x4c, 0x01},
    {0x4d, 0xeb},
    {0x4e, 0x05},
    {0x4c, 0x01},
    {0x4d, 0xea},
    {0x4e, 0x05},
    {0x4c, 0x02},
    {0x4d, 0x09},
    {0x4e, 0x05},
    {0x4c, 0x02},
    {0x4d, 0x29},
    {0x4e, 0x05},
    {0x4c, 0x02},
    {0x4d, 0x2a},
    {0x4e, 0x05},
    {0x4c, 0x02},
    {0x4d, 0x4a},
    {0x4e, 0x05},
    // {0x4c, 0x02}, //A
    // {0x4d, 0x6a},
    // {0x4e, 0x06},
    {0x4c, 0x02},
    {0x4d, 0x8a},
    {0x4e, 0x06},
    {0x4c, 0x02},
    {0x4d, 0x49},
    {0x4e, 0x06},
    {0x4c, 0x02},
    {0x4d, 0x69},
    {0x4e, 0x06},
    {0x4c, 0x02},
    {0x4d, 0x89},
    {0x4e, 0x06},
    {0x4c, 0x02},
    {0x4d, 0xa9},
    {0x4e, 0x06},
    {0x4c, 0x02},
    {0x4d, 0x48},
    {0x4e, 0x06},
    {0x4c, 0x02},
    {0x4d, 0x68},
    {0x4e, 0x06},
    {0x4c, 0x02},
    {0x4d, 0x69},
    {0x4e, 0x06},
    {0x4c, 0x02}, // H
    {0x4d, 0xca},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xc9},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xe9},
    {0x4e, 0x07},
    {0x4c, 0x03},
    {0x4d, 0x09},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xc8},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xe8},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xa7},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xc7},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xe7},
    {0x4e, 0x07},
    {0x4c, 0x03},
    {0x4d, 0x07},
    {0x4e, 0x07},

    {0x4f, 0x01},
    {0x50, 0x80},
    {0x51, 0xa8},
    {0x52, 0x47},
    {0x53, 0x38},
    {0x54, 0xc7},
    {0x56, 0x0e},
    {0x58, 0x08},
    {0x5b, 0x00},
    {0x5c, 0x74},
    {0x5d, 0x8b},
    {0x61, 0xdb},
    {0x62, 0xb8},
    {0x63, 0x86},
    {0x64, 0xc0},
    {0x65, 0x04},
    {0x67, 0xa8},
    {0x68, 0xb0},
    {0x69, 0x00},
    {0x6a, 0xa8},
    {0x6b, 0xb0},
    {0x6c, 0xaf},
    {0x6d, 0x8b},
    {0x6e, 0x50},
    {0x6f, 0x18},
    {0x73, 0xf0},
    {0x70, 0x0d},
    {0x71, 0x60},
    {0x72, 0x80},
    {0x74, 0x01},
    {0x75, 0x01},
    {0x7f, 0x0c},
    {0x76, 0x70},
    {0x77, 0x58},
    {0x78, 0xa0},
    {0x79, 0x5e},
    {0x7a, 0x54},
    {0x7b, 0x58},
    {0xfe, 0x00}, // Select bank0
    /* CC */
    {0xfe, 0x02}, // Select bank2
    {0xc0, 0x01},
    {0xc1, 0x44},
    {0xc2, 0xfd},
    {0xc3, 0x04},
    {0xc4, 0xF0},
    {0xc5, 0x48},
    {0xc6, 0xfd},
    {0xc7, 0x46},
    {0xc8, 0xfd},
    {0xc9, 0x02},
    {0xca, 0xe0},
    {0xcb, 0x45},
    {0xcc, 0xec},
    {0xcd, 0x48},
    {0xce, 0xf0},
    {0xcf, 0xf0},
    {0xe3, 0x0c},
    {0xe4, 0x4b},
    {0xe5, 0xe0},
    /* ABS */
    {0xfe, 0x01}, // Select bank1
    {0x9f, 0x40},
    {0xfe, 0x00}, // Select bank0
    /* OUTPUT */
    {0xfe, 0x00}, // Select bank0
    {0xf2, 0x0f},
    /* dark sun */
    {0xfe, 0x02}, // Select bank2
    {0x40, 0xbf},
    {0x46, 0xcf},
    {0xfe, 0x00}, // Select bank0

    /* frame rate 50Hz */
    {0xfe, 0x00}, // Select bank0
    {0x05, 0x01},
    {0x06, 0x56},
    {0x07, 0x00},
    {0x08, 0x32},
    {0xfe, 0x01}, // Select bank1
    {0x25, 0x00},
    {0x26, 0xfa},
    {0x27, 0x04},
    {0x28, 0xe2}, //20fps 
    {0x29, 0x06},
    {0x2a, 0xd6}, //14fps 
    {0x2b, 0x07},
    {0x2c, 0xd0}, //12fps
    {0x2d, 0x0b},
    {0x2e, 0xb8}, //8fps
    {0xfe, 0x00}, // Select bank0
    
    {0xfe, 0x00}, // Select bank0
    {0xfd, 0x01},
    {0xfa, 0x00}, // {0xfa, 0x11},
    /* crop window */
    {0xfe, 0x00}, // Select bank0
    {0x90, 0x01},
    {0x91, 0x00},
    {0x92, 0x00},
    {0x93, 0x00},
    {0x94, 0x00},
    {0x95, 0x02},
    {0x96, 0x58},
    {0x97, 0x03},
    {0x98, 0x20},
    {0x99, 0x11},
    {0x9a, 0x06},
    /*AWB*/
    {0xfe, 0x00}, // Select bank0
    {0xec, 0x02},
    {0xed, 0x02},
    {0xee, 0x30},
    {0xef, 0x48},
    {0xfe, 0x02}, // Select bank2
    {0x9d, 0x08},
    {0xfe, 0x01}, // Select bank1
    {0x74, 0x00},
    /* Automatic Exposure Control */
    {0xfe, 0x01}, // Select bank1
    {0x01, 0x04},
    {0x02, 0x60},
    {0x03, 0x02},
    {0x04, 0x48},
    {0x05, 0x18},
    {0x06, 0x50},
    {0x07, 0x10},
    {0x08, 0x38},
    {0x0a, 0x80},
    {0x21, 0x04},
    {0xfe, 0x00}, // Select bank0
    {0x20, 0x03},
    {0xfe, 0x00}, // Select bank0
    {GC2145_REG_NULL, 0x00},
};

/* 320X240 QVGA,30fps*/
static struct gc2145_reg gc2145_setting_qvga[] ={
    {0xfe, 0x00},
    {0xb6, 0x01},
    {0xfd, 0x01},
    {0xfa, 0x00},
    /* crop window */ 
    {0xfe, 0x00},
    {0x90, 0x01},
    {0x91, 0x00},
    {0x92, 0x00},
    {0x93, 0x00},
    {0x94, 0x00},
    {0x95, 0x00},
    {0x96, 0xf0},
    {0x97, 0x01},
    {0x98, 0x40},
    {0x99, 0x55}, // subsample
    {0x9a, 0x06},
    {0x9b, 0x01},
    {0x9c, 0x00},
    {0x9d, 0x00},
    {0x9e, 0x00},
    {0x9f, 0x01},
    {0xa0, 0x00},
    {0xa1, 0x00},
    {0xa2, 0x00},
    /* Auto White Balance */
    {0xfe, 0x00},
    {0xec, 0x02}, // measure window 
    {0xed, 0x02},
    {0xee, 0x30},
    {0xef, 0x48},
    {0xfe, 0x02},
    {0x9d, 0x08},
    {0xfe, 0x01},
    {0x74, 0x00}, // [2:0]awb skip:2x2
    /* Automatic Exposure Control */
    {0xfe, 0x01},
    {0x01, 0x04},
    {0x02, 0x60},
    {0x03, 0x02},
    {0x04, 0x48},
    {0x05, 0x18},
    {0x06, 0x50},
    {0x07, 0x10},
    {0x08, 0x38},
    {0x0a, 0x80}, // {0x0a, 0xc0},//[1:0]AEC skip
    {0x21, 0x04},
    {0xfe, 0x00},
    {0x20, 0x03},
    {0xfe, 0x00},
    {GC2145_REG_NULL, 0x00},
};

/* 640X480 VGA,30fps*/
static struct gc2145_reg gc2145_setting_vga[]={
//SENSORDB("GC2145_Sensor_VGA"},
    {0xfe, 0x00},
    {0xb6, 0x01},
    {0xfd, 0x01},
    {0xfa, 0x00},
    /* crop window */ 
    {0xfe, 0x00},
    {0x90, 0x01},
    {0x91, 0x00},
    {0x92, 0x00},
    {0x93, 0x00},
    {0x94, 0x00},
    {0x95, 0x01},
    {0x96, 0xe0},
    {0x97, 0x02},
    {0x98, 0x80},
    {0x99, 0x55},
    {0x9a, 0x06},
    {0x9b, 0x01},
    {0x9c, 0x23},
    {0x9d, 0x00},
    {0x9e, 0x00},
    {0x9f, 0x01},
    {0xa0, 0x23},
    {0xa1, 0x00},
    {0xa2, 0x00},
    /* Auto White Balance */
    {0xfe, 0x00},
    {0xec, 0x02},
    {0xed, 0x02},
    {0xee, 0x30},
    {0xef, 0x48},
    {0xfe, 0x02},
    {0x9d, 0x08},
    {0xfe, 0x01},
    {0x74, 0x00},
    /* Automatic Exposure Control */
    {0xfe, 0x01},
    {0x01, 0x04},
    {0x02, 0x60},
    {0x03, 0x02},
    {0x04, 0x48},
    {0x05, 0x18},
    {0x06, 0x50},
    {0x07, 0x10},
    {0x08, 0x38},
    // {0x0a, 0xc0},//[1:0]AEC Skip
    {0x0a, 0x80},//[1:0]AEC Skip
    {0x21, 0x04},
    {0xfe, 0x00},
    {0x20, 0x03},
    {0xfe, 0x00},
    {GC2145_REG_NULL, 0x00},
};

/* 800X600 SVGA,30fps*/
static struct gc2145_reg gc2145_setting_svga[] ={
#if 1
    /* gc2145_dvp_svga_20fps */
	{0xfe, 0x00},
	{0x05, 0x02},
	{0x06, 0x20},
	{0x07, 0x03},
	{0x08, 0x80},
	{0xb6, 0x01},
	{0xfd, 0x03},
	{0xfa, 0x00},
	{0x18, 0x42},
	/*crop window*/
	{0xfe, 0x00},
	{0x90, 0x01},
	{0x91, 0x00},
	{0x92, 0x00},
	{0x93, 0x00},
	{0x94, 0x00},
	{0x95, 0x02},
	{0x96, 0x58},
	{0x97, 0x03},
	{0x98, 0x20},
	{0x99, 0x11},
	{0x9a, 0x06},
	/*AWB*/
	{0xfe, 0x00},
	{0xec, 0x02},
	{0xed, 0x02},
	{0xee, 0x30},
	{0xef, 0x48},
	{0xfe, 0x02},
	{0x9d, 0x08},
	{0xfe, 0x01},
	{0x74, 0x00},
	/*AEC*/
	{0xfe, 0x01},
	{0x01, 0x04},
	{0x02, 0x60},
	{0x03, 0x02},
	{0x04, 0x48},
	{0x05, 0x18},
	{0x06, 0x50},
	{0x07, 0x10},
	{0x08, 0x38},
	{0x0a, 0x80},
	{0x21, 0x04},
	{0xfe, 0x00},
	{0x20, 0x03},
	{0xfe, 0x00},
	{GC2145_REG_NULL, 0x00},
#else
    /* From JV2 */
    {0xfe, 0x00},
    {0xb6, 0x01},
    {0xfd, 0x01}, // {0xfd, 0x00},
    {0xfa, 0x00}, // {0xfa, 0x11},
    /* crop window */ 
    {0xfe, 0x00},
    {0x90, 0x01},
    {0x91, 0x00},
    {0x92, 0x00},
    {0x93, 0x00},
    {0x94, 0x00},
    {0x95, 0x02},
    {0x96, 0x58},
    {0x97, 0x03},
    {0x98, 0x20},
    {0x99, 0x11},
    {0x9a, 0x06},
    /* Auto White Balance */
    {0xfe, 0x00},
    {0xec, 0x02},
    {0xed, 0x02},
    {0xee, 0x30},
    {0xef, 0x48},
    {0xfe, 0x02},
    {0x9d, 0x08},
    {0xfe, 0x01},
    {0x74, 0x00},
    /* Automatic Exposure Control */
    {0xfe, 0x01},
    {0x01, 0x04},
    {0x02, 0x60},
    {0x03, 0x02},
    {0x04, 0x48},
    {0x05, 0x18},
    {0x06, 0x50},
    {0x07, 0x10},
    {0x08, 0x38},
    {0x0a, 0x80},
    {0x21, 0x04},
    {0xfe, 0x00},
    {0x20, 0x03},
    {0xfe, 0x00},
    {GC2145_REG_NULL, 0x00},
#endif
};

/* 1600X1200 UXGA capture */
static struct gc2145_reg gc2145_setting_uxga[] ={
    {0xfe, 0x00},
    {0xfd, 0x00}, 
    {0xfa, 0x11}, 
    /* crop window */
    {0xfe, 0x00},
    {0x90, 0x01}, 
    {0x91, 0x00},
    {0x92, 0x00},
    {0x93, 0x00},
    {0x94, 0x00},
    {0x95, 0x04},
    {0x96, 0xb0},
    {0x97, 0x06},
    {0x98, 0x40},
    {0x99, 0x11}, 
    {0x9a, 0x06},
    /* Auto White Balance */
    {0xfe, 0x00},
    {0xec, 0x06}, 
    {0xed, 0x04},
    {0xee, 0x60},
    {0xef, 0x90},
    {0xfe, 0x01},
    {0x74, 0x01}, 
    /* Automatic Exposure Control */
    {0xfe, 0x01},
    {0x01, 0x04},
    {0x02, 0xc0},
    {0x03, 0x04},
    {0x04, 0x90},
    {0x05, 0x30},
    {0x06, 0x90},
    {0x07, 0x30},
    {0x08, 0x80},
    {0x0a, 0x82},
    {0xfe, 0x01},
    {0x21, 0x15}, 
    {0xfe, 0x00},
    {0x20, 0x15}, // if 0xfa=11, then 0x21=15;else if 0xfa=00, then 0x21=04
    {0xfe, 0x00},
    {GC2145_REG_NULL, 0x00},
};

static struct gc2145_reg gc2145_fmt_yuv422_yuyv[] = {
    {0x84, 0x02},
};

static struct gc2145_reg gc2145_fmt_yuv422_yvyu[] = {
    {0x84, 0x03},
};

static struct gc2145_reg gc2145_fmt_yuv422_vyuy[] = {
    {0x84, 0x01},
};

static struct gc2145_reg gc2145_fmt_yuv422_uyvy[] = {
    {0x84, 0x00},
};

static struct gc2145_reg gc2145_fmt_raw[] = {
    {0x84, 0x18},
};

struct gc2145_pixfmt {
    unsigned int code;
    unsigned int colorspace;
    unsigned char output_fmt;
    struct gc2145_reg *fmt_reg;
};

static const struct gc2145_pixfmt gc2145_format_list[] = {
    // First one is the default format
    {
        .code = MEDIA_BUS_FMT_UYVY8_2X8,
        .colorspace = V4L2_COLORSPACE_SRGB,
        .output_fmt = GC2145_OUTPUT_FMT_UYVY,
        .fmt_reg = gc2145_fmt_yuv422_uyvy,
    },
    {
        .code = MEDIA_BUS_FMT_VYUY8_2X8,
        .colorspace = V4L2_COLORSPACE_JPEG,
        .output_fmt = GC2145_OUTPUT_FMT_VYUY,
        .fmt_reg = gc2145_fmt_yuv422_vyuy,
    },
    {
        .code = MEDIA_BUS_FMT_YUYV8_2X8,
        .colorspace = V4L2_COLORSPACE_SRGB,
        .output_fmt = GC2145_OUTPUT_FMT_YUYV,
        .fmt_reg = gc2145_fmt_yuv422_yuyv,
    },
    {
        .code = MEDIA_BUS_FMT_YVYU8_2X8,
        .colorspace = V4L2_COLORSPACE_JPEG,
        .output_fmt = GC2145_OUTPUT_FMT_YVYU,
        .fmt_reg = gc2145_fmt_yuv422_yvyu,
    },
    {
        .code = MEDIA_BUS_FMT_RGB565_2X8_BE,
        .colorspace = V4L2_COLORSPACE_SRGB,
        .output_fmt = GC2145_OUTPUT_FMT_RGB,
        .fmt_reg = gc2145_fmt_raw,
    },
    {
        .code = MEDIA_BUS_FMT_SBGGR8_1X8,
        .colorspace = V4L2_COLORSPACE_RAW,
        .output_fmt = GC2145_OUTPUT_FMT_LSC,
        .fmt_reg = gc2145_fmt_raw,
    },
};

#define GC2145_QVGA_WIDTH 320
#define GC2145_QVGA_HEIGHT 240
#define GC2145_VGA_WIDTH 640
#define GC2145_VGA_HEIGHT 480
#define GC2145_SVGA_WIDTH 800
#define GC2145_SVGA_HEIGHT 600
#define GC2145_UXGA_WIDTH 1600
#define GC2145_UXGA_HEIGHT 1200

enum gc2145_mode_id {
    GC2145_MODE_QVGA_320_240 = 0,
    GC2145_MODE_VGA_640_480 = 1,
    GC2145_MODE_SVGA_800_600 = 2,
    GC2145_MODE_UXGA_1600_1200 = 3,
    GC2145_MODE_NUM,
};

struct gc2145_mode {
    enum gc2145_mode_id id;
    unsigned int hact; // Width
    unsigned int htot;
    unsigned int vact; // Height
    unsigned int vtot;
    const struct gc2145_reg *reg_list;
    unsigned int reg_list_size;
};

static const struct gc2145_mode gc2145_mode_list[GC2145_MODE_NUM] = {
    {
        .id = GC2145_MODE_QVGA_320_240,
        .hact = 320,
        .htot = 320,
        .vact = 240,
        .vtot = 240,
        .reg_list = gc2145_setting_qvga,
        .reg_list_size = ARRAY_SIZE(gc2145_setting_qvga),
    },
    {
        .id = GC2145_MODE_VGA_640_480,
        .hact = 640,
        .htot = 640,
        .vact = 480,
        .vtot = 480,
        .reg_list = gc2145_setting_vga,
        .reg_list_size = ARRAY_SIZE(gc2145_setting_vga),
    },
    {
        .id = GC2145_MODE_SVGA_800_600,
        .hact = 800,
        .htot = 800,
        .vact = 600,
        .vtot = 600,
        .reg_list = gc2145_setting_svga,
        .reg_list_size = ARRAY_SIZE(gc2145_setting_svga),
    },
    {
        .id = GC2145_MODE_UXGA_1600_1200,
        .hact = 1600,
        .htot = 1600,
        .vact = 1200,
        .vtot = 1200,
        .reg_list = gc2145_setting_uxga,
        .reg_list_size = ARRAY_SIZE(gc2145_setting_uxga),
    }
};

struct gc2145_ctrls {
    struct v4l2_ctrl_handler handler;
    struct {
        struct v4l2_ctrl *auto_exp;
        struct v4l2_ctrl *exposure;
    };
    struct {
        struct v4l2_ctrl *auto_wb;
        struct v4l2_ctrl *blue_balance;
        struct v4l2_ctrl *red_balance;
    };
    struct {
        struct v4l2_ctrl *auto_gain;
        struct v4l2_ctrl *gain;
    };
    struct v4l2_ctrl *brightness;
    struct v4l2_ctrl *light_freq;
    struct v4l2_ctrl *saturation;
    struct v4l2_ctrl *contrast;
    struct v4l2_ctrl *hue;
    struct v4l2_ctrl *test_pattern;
    struct v4l2_ctrl *hflip;
    struct v4l2_ctrl *vflip;
};

struct gc2145_dev {
    struct v4l2_subdev sd;
    struct v4l2_mbus_framefmt fmt;
    struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
    struct i2c_client *i2c_client;
    struct media_pad pad;
    struct clk *xclk; /* system clock to GC2145 */
    struct gpio_desc *reset_gpio;
    struct gpio_desc *pwdn_gpio;
    /* lock to protect all members below */
    struct mutex lock;
    const struct gc2145_mode *current_mode;
    const struct gc2145_mode *last_mode;
    struct gc2145_ctrls ctrls;
    unsigned int xclk_freq;
    int power_count;
};

/* General functions */
static inline struct gc2145_dev *to_gc2145_dev(struct v4l2_subdev *sd);
static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl);
static int gc2145_write_reg(struct i2c_client *client, u8 reg, u8 val);
static int gc2145_read_reg(struct i2c_client *client, u8 reg, u8 *val);
static int gc2145_write_array(
    struct i2c_client *client,
    const struct gc2145_reg *regs,
    unsigned int size);

static int gc2145_enum_mbus_code(
    struct v4l2_subdev *sd,
    struct v4l2_subdev_pad_config *cfg,
    struct v4l2_subdev_mbus_code_enum *code);
static int gc2145_enum_frame_size(
    struct v4l2_subdev *sd,
    struct v4l2_subdev_pad_config *cfg,
    struct v4l2_subdev_frame_size_enum *fse);
static const struct gc2145_mode *gc2145_find_mode(
    struct gc2145_dev *sensor,
    int width, int height, bool nearest);
static int gc2145_try_fmt_internal(
    struct v4l2_subdev *sd,
    struct v4l2_mbus_framefmt *fmt,
    const struct gc2145_mode **new_mode);
static int gc2145_get_fmt(
    struct v4l2_subdev *sd,
    struct v4l2_subdev_pad_config *cfg,
    struct v4l2_subdev_format *format);
static int gc2145_set_fmt(
    struct v4l2_subdev *sd,
    struct v4l2_subdev_pad_config *cfg,
    struct v4l2_subdev_format *format);

static int gc2145_power(struct gc2145_dev *sensor, bool enable);
static int gc2145_reset(struct gc2145_dev *sensor);
/* soc_camera_ops functions */
static int gc2145_set_power_on(struct gc2145_dev *sensor);
/* OF probe functions */
static int gc2145_remove(struct i2c_client *client);

static inline struct gc2145_dev *to_gc2145_dev(struct v4l2_subdev *sd)
{
    return container_of(sd, struct gc2145_dev, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
    return &container_of(ctrl->handler, struct gc2145_dev, ctrls.handler)->sd;
}

static int gc2145_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
    struct i2c_msg msg;
    u8 buf[2];
    int ret;
#ifdef GC2145_DEBUG_MSG
    printk("%s: reg:0x%02X val:0x%02X\n", __func__, reg, val);
#endif
    buf[0] = reg;
    buf[1] = val;

    msg.addr = client->addr;
    msg.flags = client->flags;
    msg.buf = buf;
    msg.len = sizeof(buf);

    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret < 0) {
        dev_err(&client->dev, "%s: error: reg=%x, val=%x\n", __func__, reg, val);
        return ret;
    }

    return 0;
}

static int gc2145_read_reg(struct i2c_client *client, u8 reg, u8 *val)
{
    struct i2c_msg msg[2];
    u8 buf[1];
    int ret;
    buf[0] = reg;

    msg[0].addr = client->addr;
    msg[0].flags = client->flags;
    msg[0].buf = buf;
    msg[0].len = sizeof(buf);

    msg[1].addr = client->addr;
    msg[1].flags = client->flags | I2C_M_RD;
    msg[1].buf = buf;
    msg[1].len = 1;

    ret = i2c_transfer(client->adapter, msg, 2);
    if (ret < 0) {
        dev_err(&client->dev, "%s: error: reg=%x i2c addr %x\n", __func__, reg, client->addr);
        return ret;
    }

    *val = buf[0];
#ifdef GC2145_DEBUG_MSG
    printk("%s: reg:0x%02X val:0x%02X\n", __func__, reg, *val);
#endif
    return 0;
}

static int gc2145_write_array(
    struct i2c_client *client,
    const struct gc2145_reg *regs,
    unsigned int size)
{
    int ret = 0;
    unsigned int i = 0;
    if (client == NULL) {
    #ifdef GC2145_DEBUG_MSG
        printk("%s: error(1)\n", __func__);
    #endif
        return -EINVAL;
    }
    if (regs == NULL) {
    #ifdef GC2145_DEBUG_MSG
        printk("%s: error(2)\n", __func__);
    #endif
        return -EINVAL;
    }
    if (size == 0) {
    #ifdef GC2145_DEBUG_MSG
        printk("%s: error(3)\n", __func__);
    #endif
        return -EINVAL;
    }
    for (i = 0; i < size ; i++) {
        if(regs[i].addr == GC2145_REG_NULL) {
            mdelay(regs[i].val);
        } else {
            ret = gc2145_write_reg(client, regs[i].addr, regs[i].val);
            if (ret < 0) {
                dev_err(&client->dev, "%s failed !\n", __func__);
                break;
            }
        }
    }
#ifdef GC2145_DEBUG_MSG
    printk("%s: end(%d)\n", __func__, ret);
#endif
    return ret;
}

static int gc2145_enum_mbus_code(
    struct v4l2_subdev *sd,
    struct v4l2_subdev_pad_config *cfg,
    struct v4l2_subdev_mbus_code_enum *code)
{
    struct gc2145_dev *dev = to_gc2145_dev(sd);
    if (code->pad != 0) {
    #ifdef GC2145_DEBUG_MSG
        printk("%s: error(1)\n", __func__);
    #endif
        return -EINVAL;
    }
    if (code->index >= ARRAY_SIZE(gc2145_format_list)) {
    #ifdef GC2145_DEBUG_MSG
        printk("%s: error(2)\n", __func__);
    #endif
        return -EINVAL;
    }
    mutex_lock(&dev->lock);
    code->code = gc2145_format_list[code->index].code;
    mutex_unlock(&dev->lock);
#ifdef GC2145_DEBUG_MSG
    printk("%s: index:%d code:%u\n", __func__, code->index, code->code);
#endif
    return 0;
}

static const struct gc2145_pixfmt *gc2145_find_pixfmt(unsigned int code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(gc2145_format_list); i++)
		if (gc2145_format_list[i].code == code) {
        #ifdef GC2145_DEBUG_MSG
            printk("%s: code found:%u\n", __func__, code);
        #endif
            break;
        }

	if (i >= ARRAY_SIZE(gc2145_format_list)) {
		i = 0;
    #ifdef GC2145_DEBUG_MSG
        printk("%s: code not found, use default.\n", __func__);
    #endif
    }

	return &gc2145_format_list[i];
}

static int gc2145_enum_frame_size(
    struct v4l2_subdev *sd,
    struct v4l2_subdev_pad_config *cfg,
    struct v4l2_subdev_frame_size_enum *fse)
{
    struct gc2145_dev *dev = to_gc2145_dev(sd);
    unsigned int code;
    const struct gc2145_pixfmt *gc2145_pixfmt;
    if (fse->index >= ARRAY_SIZE(gc2145_format_list)) {
    #ifdef GC2145_DEBUG_MSG
        printk("%s: error(1)\n", __func__);
    #endif
        return -EINVAL;
    }
    mutex_lock(&dev->lock);
    gc2145_pixfmt = gc2145_find_pixfmt(fse->code);
    code = gc2145_pixfmt->code;
    mutex_unlock(&dev->lock);
    if (fse->code != code) {
    #ifdef GC2145_DEBUG_MSG
        printk("%s: error(2)\n", __func__);
    #endif
        return -EINVAL;
    }
    fse->min_width = gc2145_mode_list[fse->index].hact;
    fse->max_width = fse->min_width;
    fse->min_height = gc2145_mode_list[fse->index].vact;
    fse->max_height = fse->min_height;
#ifdef GC2145_DEBUG_MSG
    printk("%s: min:%dx%d %dx%d\n", __func__, fse->min_width, fse->min_height, fse->max_width, fse->max_height);
#endif
    return 0;
}

static const struct gc2145_mode *gc2145_find_mode(
    struct gc2145_dev *sensor,
    int width, int height, bool nearest)
{
    const struct gc2145_mode *mode;
#ifdef GC2145_DEBUG_MSG
    printk("%s: finding. width:%u height:%u\n"
        , __func__
        , width
        , height);
#endif
    mode = v4l2_find_nearest_size(
                gc2145_mode_list,
                ARRAY_SIZE(gc2145_mode_list),
                hact, vact,
                width, height);
    if (!mode || (!nearest && ((mode->hact != width) || (mode->vact != height)))) {
    #ifdef GC2145_DEBUG_MSG
        printk("%s: not found\n", __func__);
    #endif
        return NULL;
    }
#ifdef GC2145_DEBUG_MSG
    printk("%s: found. mode:%u hact:%u htot:%u vact:%u vtot:%u regs_size:%u\n"
        , __func__
        , mode->id
        , mode->hact
        , mode->htot
        , mode->vact
        , mode->vtot
        , mode->reg_list_size);
#endif
    return mode;
}

static int gc2145_try_fmt_internal(
    struct v4l2_subdev *sd,
    struct v4l2_mbus_framefmt *mbus_fmt,
    const struct gc2145_mode **new_mode)
{
    struct gc2145_dev *sensor = to_gc2145_dev(sd);
    const struct gc2145_mode *mode = NULL;
    const struct gc2145_pixfmt *pix_fmt = NULL;
    if (new_mode == NULL) {
    #ifdef GC2145_DEBUG_MSG
        printk("%s: error(1)\n", __func__);
    #endif
        return -EINVAL;
    }
    /* Find mode */
    mode = gc2145_find_mode(sensor, mbus_fmt->width, mbus_fmt->height, true);
    if (mode == NULL) {
    #ifdef GC2145_DEBUG_MSG
        printk("%s: mode not found\n", __func__);
    #endif
        return -EINVAL;
    }
    *new_mode = mode;
#ifdef GC2145_DEBUG_MSG
    printk("%s: new mode found %dx%d\n", __func__, mode->hact, mode->vact);
#endif
    mbus_fmt->width = mode->hact;
    mbus_fmt->height = mode->vact;
    mbus_fmt->field = V4L2_FIELD_NONE;
    /* Find pix_fmt */
    pix_fmt = gc2145_find_pixfmt(mbus_fmt->code);
    mbus_fmt->code = pix_fmt->code;
    mbus_fmt->colorspace = pix_fmt->colorspace;
    mbus_fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(mbus_fmt->colorspace);
    mbus_fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true, mbus_fmt->colorspace, mbus_fmt->ycbcr_enc);
    mbus_fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(mbus_fmt->colorspace);
    return 0;
}

static int gc2145_get_fmt(
    struct v4l2_subdev *sd,
    struct v4l2_subdev_pad_config *cfg,
    struct v4l2_subdev_format *format)
{
    struct gc2145_dev *sensor = to_gc2145_dev(sd);
    // struct v4l2_mbus_framefmt *mbus_fmt = &format->format;
    struct v4l2_mbus_framefmt *fmt;
#ifdef GC2145_DEBUG_MSG
    printk("%s: called\n", __func__);
#endif
    if (format->pad != 0)
        return -EINVAL;
#if 1
    mutex_lock(&sensor->lock);
    if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
    #ifdef GC2145_DEBUG_MSG
        printk("%s: v4l2_subdev_get_try_format\n", __func__);
    #endif
        fmt = v4l2_subdev_get_try_format(
                &sensor->sd,
                cfg,
                format->pad);
    } else {
    #ifdef GC2145_DEBUG_MSG
        printk("%s: got format\n", __func__);
    #endif
        fmt = &sensor->fmt;
    }
    format->format = *fmt;
    mutex_unlock(&sensor->lock);
    return 0;
#else
    // mutex_lock(&sensor->lock);
    if (!sensor->current_mode) {
        unsigned int width = GC2145_SVGA_WIDTH;
        unsigned int height = GC2145_SVGA_HEIGHT;
        sensor->current_mode = gc2145_find_mode(sensor, width, height, true);
        width = sensor->current_mode->hact;
        height = sensor->current_mode->vact;
    }

    mbus_fmt->width = sensor->current_mode->hact;
    mbus_fmt->height = sensor->current_mode->vact;

    switch (mbus_fmt->code) {
    case MEDIA_BUS_FMT_RGB565_2X8_BE:
    case MEDIA_BUS_FMT_RGB565_2X8_LE:
        mbus_fmt->colorspace = V4L2_COLORSPACE_SRGB;
        break;
    default:
    case MEDIA_BUS_FMT_YUYV8_2X8:
    case MEDIA_BUS_FMT_UYVY8_2X8:
        mbus_fmt->colorspace = V4L2_COLORSPACE_JPEG;
    }
    mbus_fmt->field = V4L2_FIELD_NONE;
    // mutex_unlock(&sensor->lock);
    return 0;
#endif
}

static int gc2145_params_set(
    struct gc2145_dev *sensor,
    struct v4l2_mbus_framefmt *fmt)
{
    const struct gc2145_pixfmt *pixfmt = NULL;
    // struct gc2145_reg *cfmt_regs_selected;
    // unsigned int cfmt_regs_size;
    int ret;
    pixfmt = gc2145_find_pixfmt(sensor->fmt.code);
    // sensor->current_mode = gc2145_find_mode(sensor, fmt->width, fmt->height, true);
#ifdef GC2145_DEBUG_MSG
    printk("%s: sensor:%dx%d, fmt:%dx%d\n",
        __func__,
        sensor->current_mode->hact,
        sensor->current_mode->vact,
        fmt->width,
        fmt->height);
#endif
    // Init
    ret = gc2145_write_array(sensor->i2c_client, gc2145_init_regs, ARRAY_SIZE(gc2145_init_regs));
    if (ret < 0)
        return ret;
    /* Set the output format */
    ret = gc2145_write_reg(sensor->i2c_client, GC2145_REG_PAGE_SELECT, 0x00);
    if (ret < 0)
        return ret;
    ret = gc2145_write_reg(sensor->i2c_client, pixfmt->fmt_reg->addr, pixfmt->fmt_reg->val);
    if (ret < 0)
        return ret;

// #ifdef GC2145_DEBUG_MSG
//     printk("%s: width:%u height:%u\r\n", __func__, fmt->width, fmt->height);
// #endif
    // if((sensor->current_mode->hact == 640) && (sensor->current_mode->vact == 480)) {
    //     msleep(300);
    // }
    // if((sensor->current_mode->hact == 800) && (sensor->current_mode->vact == 600)) {
    //     msleep(300);
    // }
    // if((sensor->current_mode->hact == 320) && (sensor->current_mode->vact == 240)) {
    //     msleep(300);
    // }
    // ret = gc2145_write_array(sensor->i2c_client, sensor->current_mode->reg_list, sensor->current_mode->reg_list_size);
    // if (ret < 0)
    //     return ret;
    // fmt->width = sensor->current_mode->hact;
    // fmt->height = sensor->current_mode->vact;
    // if((sensor->current_mode->hact >= 1024) && (sensor->current_mode->vact >= 768))
    // {
    //     msleep(550);
    // }
    return 0;
}

static int gc2145_set_fmt(
    struct v4l2_subdev *sd,
    struct v4l2_subdev_pad_config *cfg,
    struct v4l2_subdev_format *format)
{
    struct gc2145_dev *sensor = to_gc2145_dev(sd);
    const struct gc2145_mode *new_mode = NULL;
    struct v4l2_mbus_framefmt *mbus_fmt_in = &format->format;
    struct v4l2_mbus_framefmt *mbus_fmt_out;
    int ret;
#ifdef GC2145_DEBUG_MSG
    printk("%s: called\n", __func__);
#endif
    if (format->pad != 0) {
        printk("%s: error(1)\n", __func__);
        return -EINVAL;
    }
#if 1
    mutex_lock(&sensor->lock);
    mbus_fmt_in->width = 800;
    mbus_fmt_in->height = 600;
    ret = gc2145_try_fmt_internal(sd, mbus_fmt_in, &new_mode);
    if (ret) {
        printk("%s: error(2)\n", __func__);
        goto out;
    }
    if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
    #ifdef GC2145_DEBUG_MSG
        printk("%s: V4L2_SUBDEV_FORMAT_TRY\n", __func__);
    #endif
        mbus_fmt_out = v4l2_subdev_get_try_format(sd, cfg, 0);
    } else {
        mbus_fmt_out = &sensor->fmt;
    }
    *mbus_fmt_out = *mbus_fmt_in;
    
    if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
    // if (new_mode != sensor->current_mode) {
        sensor->current_mode = new_mode;
    #ifdef GC2145_DEBUG_MSG
        printk("%s: new_mode found, %dx%d\n", __func__, mbus_fmt_out->width, mbus_fmt_out->height);
    #endif
        ret = gc2145_params_set(sensor, mbus_fmt_out);
        if (ret != 0) {
            printk("%s: error(3)\n", __func__);
        }
    }
out:
    mutex_unlock(&sensor->lock);
    return ret;
#else
    // mutex_lock(&sensor->lock);
    /*
     * select suitable win, but don't store it
     */
    new_mode = gc2145_find_mode(sensor, mbus_fmt->width, mbus_fmt->height, true);
    mbus_fmt->width = new_mode->hact;
    mbus_fmt->height = new_mode->vact;
    mbus_fmt->field = V4L2_FIELD_NONE;
    switch (mbus_fmt->code) {
    case MEDIA_BUS_FMT_SBGGR8_1X8:
        mbus_fmt->colorspace = V4L2_COLORSPACE_SRGB;
        break;
    default:
        mbus_fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;
    case MEDIA_BUS_FMT_VYUY8_2X8:
    case MEDIA_BUS_FMT_YUYV8_2X8:
    case MEDIA_BUS_FMT_UYVY8_2X8:
        mbus_fmt->colorspace = V4L2_COLORSPACE_JPEG;
        break;
    }
    mbus_fmt->field = V4L2_FIELD_NONE;
    if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
        return gc2145_params_set(sensor, mbus_fmt);
    cfg->try_fmt = *mbus_fmt;
out:
    // mutex_unlock(&sensor->lock);
    return ret;
#endif
}

static int gc2145_power(struct gc2145_dev *sensor, bool enable)
{
    if (!sensor->pwdn_gpio) {
        printk("%s: error(1)\r\n", __func__);
        return -1;
    }
    if (enable) {
        gpiod_set_value_cansleep(sensor->pwdn_gpio, 1);
        udelay(100);
        gpiod_set_value_cansleep(sensor->pwdn_gpio, 0);
    } else {
        gpiod_set_value_cansleep(sensor->pwdn_gpio, 1);
    }
    udelay(100);
#ifdef GC2145_DEBUG_MSG
    printk("%s: success\r\n", __func__);
#endif
    return 0;
}

static int gc2145_reset(struct gc2145_dev *sensor)
{
    if (!sensor->reset_gpio) {
        printk("%s: error(1)\r\n", __func__);
        return -1;
    }
    /* camera power cycle */
    gpiod_set_value_cansleep(sensor->reset_gpio, 1);
    udelay(100);
    gpiod_set_value_cansleep(sensor->reset_gpio, 0);
    udelay(100);
#ifdef GC2145_DEBUG_MSG
    printk("%s: success\r\n", __func__);
#endif
    return 0;
}

static int gc2145_set_power_on(struct gc2145_dev *sensor)
{
    if (!sensor) {
        printk("%s: error(1)\r\n", __func__);
        return -1;
    }
    if (gc2145_power(sensor, true) != 0) {
        printk("%s: error(2)\r\n", __func__);
        return -1;
    }
    if (gc2145_reset(sensor) != 0) {
        printk("%s: error(3)\r\n", __func__);
        return -1;
    }
#ifdef GC2145_DEBUG_MSG
    printk("%s: success\r\n", __func__);
#endif
    return 0;
}

static int gc2145_set_power_off(struct gc2145_dev *sensor)
{
    if (gc2145_power(sensor, false) != 0) {
        printk("%s: error(1)\r\n", __func__);
        return -1;
    }
    clk_disable_unprepare(sensor->xclk);
#ifdef GC2145_DEBUG_MSG
    printk("%s: success\r\n", __func__);
#endif
    return 0;
}

static int gc2145_s_std(struct v4l2_subdev *sd, v4l2_std_id norm)
{
#ifdef GC2145_DEBUG_MSG
    printk("%s: called\r\n", __func__);
#endif
    return 0;
}

static int gc2145_s_stream(struct v4l2_subdev *sd, int enable)
{
#ifdef GC2145_DEBUG_MSG
    printk("%s: called\r\n", __func__);
#endif
    return 0;
}

static int gc2145_s_power(struct v4l2_subdev *sd, int on)
{
    struct gc2145_dev *sensor = to_gc2145_dev(sd);
    int ret = 0;
#ifdef GC2145_DEBUG_MSG
    printk("%s: called\r\n", __func__);
#endif
    // mutex_lock(&sensor->lock);
    if (sensor->power_count == !on) {
        ret = gc2145_set_power_on(sensor);
        if (ret)
            goto out;
        // gc2145_write_array(sensor->i2c_client, sensor->current_mode->reg_list, sensor->current_mode->reg_list_size);
    }

    /* Update the power count. */
    sensor->power_count += on ? 1 : -1;
    WARN_ON(sensor->power_count < 0);
out:
    // mutex_unlock(&sensor->lock);
    return ret;
}

static int gc2145_log_status(struct v4l2_subdev *sd)
{
#ifdef GC2145_DEBUG_MSG
    printk("%s: called\r\n", __func__);
#endif
    return 0;
}

static int gc2145_s_vflip(struct i2c_client *client, int value)
{
    int ret;
    struct gc2145_reg regs;
    regs.addr = 0xfe;
    regs.val = 0x00; //page 0
    ret = gc2145_write_reg(client, regs.addr, regs.val);
    if (ret < 0) {
    #ifdef GC2145_DEBUG_MSG
        printk("%s: error(1)\r\n", __func__);
    #endif
        return ret;
    }
    regs.addr = 0x17;
    ret = gc2145_read_reg(client, regs.addr, &regs.val);
    if (ret < 0) {
    #ifdef GC2145_DEBUG_MSG
        printk("%s: error(2)\r\n", __func__);
    #endif
        return ret;
    }
    switch (value) {
        case 0:
          regs.val &= 0xfd;
            break;
        case 1:
            regs.val |= 0x02;
            break;
        default:
            return -EINVAL;
    }
    ret = gc2145_write_reg(client, regs.addr, regs.val);
    if (ret < 0) {
    #ifdef GC2145_DEBUG_MSG
        printk("%s: error(3)\r\n", __func__);
    #endif
        return ret;
    }
    mdelay(20);
    // info->vflip = value;
    return 0;
}

static int gc2145_s_hflip(struct i2c_client *client, int value)
{
    int ret;
    //struct gc2145_info *info = to_state(client);
    struct gc2145_reg regs;
    regs.addr = 0xfe;
    regs.val = 0x00; //page 0
    ret = gc2145_write_reg(client, regs.addr, regs.val);
    if (ret < 0) {
    #ifdef GC2145_DEBUG_MSG
        printk("%s: error(1)\r\n", __func__);
    #endif
        return ret;
    }
    regs.addr = 0x17;
    ret = gc2145_read_reg(client, regs.addr, &regs.val);
    if (ret < 0) {
    #ifdef GC2145_DEBUG_MSG
        printk("%s: error(2)\r\n", __func__);
    #endif
        return ret;
    }
    switch (value) {
        case 0:
            regs.val &= 0xfe;
            break;
        case 1:
            regs.val |= 0x01;
            break;
        default:
            return -EINVAL;
    }
    ret = gc2145_write_reg(client, regs.addr, regs.val);
    if (ret < 0) {
    #ifdef GC2145_DEBUG_MSG
        printk("%s: error(3)\r\n", __func__);
    #endif
        return ret;
    }
    mdelay(20);
    return 0;
}

static int gc2145_s_ctrl(struct v4l2_ctrl *ctrl)
{
    struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
    struct i2c_client  *client = v4l2_get_subdevdata(sd);
#ifdef GC2145_DEBUG_MSG
    printk("%s: called\r\n", __func__);
#endif
    switch (ctrl->id) {
    case V4L2_CID_VFLIP:
        return gc2145_s_vflip(client,ctrl->val);
    case V4L2_CID_HFLIP:
        return gc2145_s_hflip(client,ctrl->val);
    }
    return -EINVAL;
}

static const struct v4l2_ctrl_ops gc2145_ctrl_ops = {
    .s_ctrl = gc2145_s_ctrl,
};

static const struct v4l2_subdev_core_ops gc2145_core_ops = {
    .s_power = gc2145_s_power,
    .log_status = gc2145_log_status,
};

static const struct v4l2_subdev_video_ops gc2145_video_ops = {
    .s_std = gc2145_s_std,
    .s_stream = gc2145_s_stream,
};

static const struct v4l2_subdev_pad_ops gc2145_pad_ops = {
    .enum_mbus_code = gc2145_enum_mbus_code,
    .enum_frame_size = gc2145_enum_frame_size,
    .get_fmt = gc2145_get_fmt,
    .set_fmt = gc2145_set_fmt,
};

static const struct v4l2_subdev_ops gc2145_subdev_ops = {
    .core = &gc2145_core_ops,
    .video = &gc2145_video_ops,
    .pad = &gc2145_pad_ops,
};

static int gc2145_check_chip_id(struct gc2145_dev *sensor)
{
    struct i2c_client *client = sensor->i2c_client;
    int ret = 0;
    u8 chip_id[2];

    ret = gc2145_set_power_on(sensor);
    if (ret) {
        printk("%s: failed\n", __func__);
        return ret;
    }

    gc2145_read_reg(sensor->i2c_client, GC2145_REG_CHIP_ID_H, &chip_id[0]);
    gc2145_read_reg(sensor->i2c_client, GC2145_REG_CHIP_ID_L, &chip_id[1]);
    dev_info(&client->dev, "chip id 0x%x%x\n", chip_id[0], chip_id[1]);
    if ((chip_id[0] != ((GC2145_CHIP_ID >> 8) & 0xFF)) && (chip_id[1] != (GC2145_REG_CHIP_ID_L & 0xFF))) {
        dev_err(&client->dev, "%s: wrong chip identifier, expected 0x%03X, got 0x%02X%02X\n",
            __func__, GC2145_CHIP_ID, chip_id[0], chip_id[1]);
        ret = -ENXIO;
        gc2145_set_power_off(sensor);
        return ret;
    }
    return 0;
}

static void gc2145_mode_set_default(struct gc2145_dev *sensor) {
    /*
     * default init sequence initialize sensor to
     * YUV422 UYVY VGA@30fps
     */
    struct v4l2_mbus_framefmt *fmt = &sensor->fmt;
    fmt->code = gc2145_format_list[0].code;
    fmt->colorspace = gc2145_format_list[0].colorspace;
    fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
    fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true, fmt->colorspace, fmt->ycbcr_enc);
    fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
    fmt->width = gc2145_mode_list[GC2145_MODE_SVGA_800_600].hact;
    fmt->height = gc2145_mode_list[GC2145_MODE_SVGA_800_600].vact;
    fmt->field = V4L2_FIELD_NONE;
    sensor->current_mode = &gc2145_mode_list[GC2145_MODE_SVGA_800_600];
    sensor->last_mode = sensor->current_mode;
    return;
}

static int gc2145_probe(
    struct i2c_client *client,
    const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct fwnode_handle *endpoint;
    struct gc2145_dev *sensor;
    int ret;
#ifdef GC2145_DEBUG_MSG
    printk("%s: called\n", __func__);
#endif
    sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
    if (!sensor)
        return -ENOMEM;

    sensor->i2c_client = client;
    gc2145_mode_set_default(sensor);

    endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
    if (!endpoint) {
        dev_err(dev, "endpoint node not found\n");
        return -EINVAL;
    }

    ret = v4l2_fwnode_endpoint_parse(endpoint, &sensor->ep);
    fwnode_handle_put(endpoint);
    if (ret) {
        dev_err(dev, "Could not parse endpoint\n");
        return ret;
    }

    /* get system clock (xclk) */
    sensor->xclk = devm_clk_get(dev, "xclk");
    if (IS_ERR(sensor->xclk)) {
        dev_err(dev, "failed to get xclk\n");
        return PTR_ERR(sensor->xclk);
    }

    sensor->xclk_freq = clk_get_rate(sensor->xclk);
    if ((sensor->xclk_freq < GC2145_XCLK_MIN) || (sensor->xclk_freq > GC2145_XCLK_MAX)) {
        dev_err(dev, "xclk frequency out of range: %d Hz\n", sensor->xclk_freq);
        return -EINVAL;
    }

    ret = clk_prepare_enable(sensor->xclk);
    if (ret) {
        dev_err(dev, "%s: failed to enable clock\n", __func__);
        return ret;
    }

    /* request optional power down pin */
    sensor->pwdn_gpio = devm_gpiod_get_optional(dev, "powerdown", GPIOD_OUT_HIGH);
    if (IS_ERR(sensor->pwdn_gpio)) {
        dev_err(dev, "%s: failed to init powerdown pin\n", __func__);
        return PTR_ERR(sensor->pwdn_gpio);
    }

    /* request optional reset pin */
    sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
    if (IS_ERR(sensor->reset_gpio)) {
        dev_err(dev, "%s: failed to init reset pin\n", __func__);
        return PTR_ERR(sensor->reset_gpio);
    }

    v4l2_i2c_subdev_init(&sensor->sd, client, &gc2145_subdev_ops);

    /* ctrl */
    v4l2_ctrl_handler_init(&sensor->ctrls.handler, 3);
    v4l2_ctrl_new_std(
        &sensor->ctrls.handler, &gc2145_ctrl_ops,
        V4L2_CID_PIXEL_RATE, 0, GC2145_PIXEL_RATE, 1, GC2145_PIXEL_RATE);
    v4l2_ctrl_new_std(
        &sensor->ctrls.handler, &gc2145_ctrl_ops,
        V4L2_CID_VFLIP, 0, 1, 1, 0);
    v4l2_ctrl_new_std(
        &sensor->ctrls.handler, &gc2145_ctrl_ops,
        V4L2_CID_HFLIP, 0, 1, 1, 0);
    sensor->sd.ctrl_handler = &sensor->ctrls.handler;
    if (sensor->ctrls.handler.error) {
        dev_err(dev, "%s: control initialization error %d\n", __func__, sensor->ctrls.handler.error);
        ret = sensor->ctrls.handler.error;
        goto LABEL_FREE;
    }

    /* Initialize subdev */
    sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
    sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
    sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
    ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
    if (ret) {
        dev_err(dev, "%s: media_entity_pads_init() failed\n", __func__);
        return ret;
    }

    // ret = gc2145_get_regulators(sensor);
    // if (ret)
    // return ret;

    mutex_init(&sensor->lock);

    ret = gc2145_check_chip_id(sensor);
    if (ret) {
        dev_err(dev, "%s: gc2145 chip id check failed\n", __func__);
        goto LABEL_CLEANUP;
    }

    // ret = gc2145_init_controls(sensor);
    ret = v4l2_ctrl_handler_setup(&sensor->ctrls.handler);
    if (ret)
        goto LABEL_CLEANUP;

    ret = v4l2_async_register_subdev_sensor_common(&sensor->sd);
    if (ret) {
        dev_err(dev, "%s: v4l2 register subdev failed\n", __func__);
        goto LABEL_FREE;
    }
    return 0;

LABEL_FREE:
    v4l2_ctrl_handler_free(&sensor->ctrls.handler);
LABEL_CLEANUP:
    media_entity_cleanup(&sensor->sd.entity);
    mutex_destroy(&sensor->lock);
    return ret;
}

static int gc2145_remove(struct i2c_client *client)
{
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct gc2145_dev *sensor = to_gc2145_dev(sd);
#ifdef GC2145_DEBUG_MSG
    printk("%s: called\n", __func__);
#endif
    v4l2_async_unregister_subdev(&sensor->sd);
    media_entity_cleanup(&sensor->sd.entity);
    v4l2_ctrl_handler_free(&sensor->ctrls.handler);
    mutex_destroy(&sensor->lock);
    return 0;
}

static const struct i2c_device_id gc2145_id[] = {
    {"gc2145", 0},
    {}
};

MODULE_DEVICE_TABLE(i2c, gc2145_id);

static const struct of_device_id gc2145_dt_ids[] = {
    {.compatible = "galaxycore,gc2145"},
    {}
};

static struct i2c_driver gc2145_i2c_driver = {
    .driver = {
        .name = "gc2145",
        .of_match_table = gc2145_dt_ids,
    },
    .id_table = gc2145_id,
    .probe    = gc2145_probe,
    .remove   = gc2145_remove,
};

module_i2c_driver(gc2145_i2c_driver);

MODULE_DESCRIPTION("GC2145 Camera Driver");
MODULE_AUTHOR("YH Chiu <chiuyungho@gmail.com>");
MODULE_LICENSE("GPL v2");
