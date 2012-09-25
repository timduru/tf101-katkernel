/*
 * arch/arm/mach-tegra/include/board-ventana-misc.h
 *
 * Copyright (C) 2010-2011 ASUSTek Computer Incorporation
 * Author: Paris Yeh <paris_yeh@asus.com>, Victor Chen <victor_chen@asus.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

//The ventana_hw is hexadecimal representation as follows.
//
//    HW[9:0] =  9  8  7  6  5  4  3  2  1  0
//              +----------------+-----------+
//              |   PCB_ID[5:0]  |GMI_AD[7:4]|
//              +----------------+-----------+
//    where PCB_ID[5:0] is KB_ROW[5:0]
//
//    PCB_ID[5:0] =   5    4    3    2    1    0
//                  +----------+-----------------+
//                  |WIMAX[1:0]|PRJ[0]| SKU[2:0] |
//                  +----------+-----------------+
//    PRJ[0]                    SKU[3:0]
//    0b -> TF101(SINTEK)       000b -> W/O 3G, Murata BT/WLAN
//    0b -> TF101(SINTEK)       001b -> 3G, Murata BT/WLAN
//    0b -> TF101(SINTEK)       010b -> W/O 3G, AZW BT/WLAN
//    0b -> TF101(SINTEK)       011b -> 3G, AZW BT/WLAN
//
//    WIMAX[1:0]	PRJ[0]                    SKU[3:0]
//    01b		0b -> TF101(SINTEK)       000b -> WiMAX, Murata BT/WLAN
//
//    0b -> TF101(WINTEK)       100b -> W/O 3G, Murata BT/WLAN
//    0b -> TF101(WINTEK)       101b -> 3G, Murata BT/WLAN
//    0b -> TF101(WINTEK)       110b -> W/O 3G, AZW BT/WLAN
//    0b -> TF101(WINTEK)       111b -> 3G, AZW BT/WLAN
//
//    1b -> SL101(SINTEK)       000b -> W/O 3G, Murata BT/WLAN
//    1b -> SL101(SINTEK)       001b -> 3G, Murata BT/WLAN
//    1b -> SL101(SINTEK)       010b -> W/O 3G, AZW BT/WLAN
//    1b -> SL101(SINTEK)       011b -> 3G, AZW BT/WLAN
//    1b -> SL101(WINTEK)       100b -> W/O 3G, Murata BT/WLAN
//    1b -> SL101(WINTEK)       101b -> 3G, Murata BT/WLAN
//    1b -> SL101(WINTEK)       110b -> W/O 3G, AZW BT/WLAN
//    1b -> SL101(WINTEK)       111b -> 3G, AZW BT/WLAN
//
#ifndef ASUS_TEGRA_DEVKIT_MISC_HW_H
#define ASUS_TEGRA_DEVKIT_MISC_HW_H

#if defined(__cplusplus)
extern "C"
{
#endif
/*
 * The HW_FIELD_* macros are helper macros for the public HW_DRF_* macros.
 */
#define HW_FIELD_LOWBIT(x)      (0?x)
#define HW_FIELD_HIGHBIT(x)     (1?x)
#define HW_FIELD_SIZE(x)        (HW_FIELD_HIGHBIT(x)-HW_FIELD_LOWBIT(x)+1)
#define HW_FIELD_SHIFT(x)       ((0?x)%32)
#define HW_FIELD_MASK(x)        (0xFFFFFFFFUL>>(31-((1?x)%32)+((0?x)%32)))
#define HW_FIELD_BITS(val, x)   (((val) & HW_FIELD_MASK(x))<<HW_FIELD_SHIFT(x))
#define HW_FIELD_SHIFTMASK(x)   (HW_FIELD_MASK(x)<< (HW_FIELD_SHIFT(x)))

/** HW_DRF_VAL - read a field from a register.

    @param d register domain (hardware block)
    @param r register name
    @param f register field
    @param v register value
 */
#define HW_DRF_VAL(d,r,f,v) \
    (((v)>> HW_FIELD_SHIFT(d##_##r##_0_##f##_RANGE)) & \
        HW_FIELD_MASK(d##_##r##_0_##f##_RANGE))

/** HW_DRF_SIZE - read a occupied length from a register.

    @param d register domain (hardware block)
    @param r register name
    @param f register field
 */
#define HW_DRF_SIZE(d,r,f) \
    (HW_FIELD_SIZE(d##_##r##_0_##f##_RANGE))

//Memory Type
#define TEGRA_DEVKIT_MISC_HW_0_MEMTYPE_RANGE	1:0
#define TEGRA_DEVKIT_MISC_HW_0_MEMTYPE_DEFAULT	0x0UL //EPD 8Gb
#define TEGRA_DEVKIT_MISC_HW_0_MEMTYPE_1	0x1UL //EPD 4Gb
#define TEGRA_DEVKIT_MISC_HW_0_MEMTYPE_2	0x2UL //HYN 8Gb
#define TEGRA_DEVKIT_MISC_HW_0_MEMTYPE_3	0x3UL //HYN 4Gb

//MMC Type
#define TEGRA_DEVKIT_MISC_HW_0_MMCTYPE_RANGE	3:2
#define TEGRA_DEVKIT_MISC_HW_0_MMCTYPE_DEFAULT	0x0UL //TOS/KIN 16G
#define TEGRA_DEVKIT_MISC_HW_0_MMCTYPE_1	0x1UL //TBD
#define TEGRA_DEVKIT_MISC_HW_0_MMCTYPE_2	0x2UL //TOS/KIN 32G
#define TEGRA_DEVKIT_MISC_HW_0_MMCTYPE_3     	0x3UL //HYN 64G

//SKU Identification
#define TEGRA_DEVKIT_MISC_HW_0_SKU_RANGE        4:4
#define TEGRA_DEVKIT_MISC_HW_0_SKU_DEFAULT      0x0UL //W/O 3G
#define TEGRA_DEVKIT_MISC_HW_0_SKU_1            0x1UL //3G

//BT/WLAN Module Vendor
#define TEGRA_DEVKIT_MISC_HW_0_VENDOR_RANGE     5:5
#define TEGRA_DEVKIT_MISC_HW_0_VENDOR_DEFAULT   0x0UL //Murata BT/WLAN
#define TEGRA_DEVKIT_MISC_HW_0_VENDOR_1         0x1UL //AZW BT/WLAN

//Touch Panel Vendor
#define TEGRA_DEVKIT_MISC_HW_0_TOUCH_RANGE      6:6
#define TEGRA_DEVKIT_MISC_HW_0_TOUCH_DEFAULT    0x0UL //SINTEK
#define TEGRA_DEVKIT_MISC_HW_0_TOUCH_1          0x1UL //WINTEK

//Project Identification
#define TEGRA_DEVKIT_MISC_HW_0_PROJECT_RANGE    7:7
#define TEGRA_DEVKIT_MISC_HW_0_PROJECT_DEFAULT  0x0UL //TF101(EP101)
#define TEGRA_DEVKIT_MISC_HW_0_PROJECT_1        0x1UL //SL101(EP102)

//WiMAX Identification
#define TEGRA_DEVKIT_MISC_HW_0_WIMAX_RANGE      8:8
#define TEGRA_DEVKIT_MISC_HW_0_WIMAX_DEFAULT    0x0UL //W/O WiMAX
#define TEGRA_DEVKIT_MISC_HW_0_WIMAX_1          0x1UL //WiMAX


//The byte-field definition of Tegra2 ReservedOdm fuse is defined as follows
//31    26       20                        0
//+------+--------+------------------------+
//|  BT  |  WiFi  |         Reserved       |
//+------+--------+------------------------+

//Tegra2 ReservedOdm fuse
#define TEGRA_DEVKIT_MISC_HW_0_RSV_RANGE 31:0

//BT Mac address
#define TEGRA_DEVKIT_MISC_HW_0_BT_RANGE 31:26

//WiFi Mac address
#define TEGRA_DEVKIT_MISC_HW_0_WIFI_RANGE 25:20

#define VENTANA_PCBID_OUTPUT_LENGTH 7

extern unsigned char ventana_chipid[17];

int __init ventana_setup_misc(void);

/* Acquire project identification
 *   @ret unsigned int
 *      Project identification will be returned. (eg, TF101(EP101) <-> 101)
 *      Otherwise, 0 will be returned instead.
 */
unsigned int ASUSGetProjectID(void);

/* Detect if 3G is equipped
 *   @ret unsigned int
 *      If 3G is equipped, 1 will be returned; Otherwise, 0 will be instead.
 */
unsigned int ASUS3GAvailable(void);

/* Detect if WiMAX is equipped
 *   @ret unsigned int
 *      If WiMAX is equipped, 1 will be returned; Otherwise, 0 will be instead.
 */
unsigned int ASUSWiMAXAvailable(void);


#define BT_WLAN_VENDOR_AZW          TEGRA_DEVKIT_MISC_HW_0_VENDOR_DEFAULT
#define BT_WLAN_VENDOR_MURATA       TEGRA_DEVKIT_MISC_HW_0_VENDOR_1

/* Check if BT/WLAN module vendor is equipped
 *   @param v module name
 *   @ret unsigned int
 *      If specified wireless module is equipped, 1 will be returned;
 *      Otherwise, 0 will be instead.
 */
unsigned int ASUSCheckWLANVendor(unsigned int vendor);


#define TOUCH_VENDOR_SINTEK		TEGRA_DEVKIT_MISC_HW_0_TOUCH_DEFAULT
#define TOUCH_VENDOR_WINTEK		TEGRA_DEVKIT_MISC_HW_0_TOUCH_1
/* Check if Touch module vendor is equipped
 *   @param v module name
 *   @ret unsigned int
 *      If specified touch panel vendor is equipped, 1 will be returned;
 *      Otherwise, 0 will be instead.
 */
unsigned int ASUSCheckTouchVendor(unsigned int vendor);

/* Acquire project name
 *      No return parameter
 *      Project name will be assigned according to different kinds devices. (TF101, TF101G, TF101-WIMAX, and SL101)
 */
static void ASUSGetProjectName(void);

/* Acquire pcb id
 *      No return parameter
 *      This function will perform the assignment for ventana_pcbid in string format
 *      Format of ventana_pcbid:
 *      =>TF101:x00xx0b
 *      =>TF101G:x00xx1b
 *      =>TF101-WiMax:x10xx0b
 *      =>SL101:x01xx0b
 */
static void ASUSGetPcbid(void);

/* ASUS uses the following to store the lcd pixel clock freq. Then decide the vaule by project name.*/
extern u32 lcd_pclk_khz;

#if defined(__cplusplus)
}
#endif

#endif

