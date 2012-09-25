/*
 * kernel/drivers/media/video/tegra
 *
 * Aptina MT9D115 sensor driver
 *
 * Copyright (C) 2010 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/yuv_sensor.h>

#define SENSOR_WIDTH_REG 0x2703
#define SENSOR_640_WIDTH_VAL 0x0280

struct sensor_reg {
	u16 addr;
	u16 val;
};

struct sensor_info {
	int mode;
	struct i2c_client *i2c_client;
	struct yuv_sensor_platform_data *pdata;
};

static struct sensor_info *info;


static struct sensor_reg mode_1600x1200[] = {
{0x098C, 0xA115},     // MCU_ADDRESS [SEQ_CAP_MODE]
{0x0990, 0x0000},      // MCU_DATA_0
{0x098C, 0xA116},     // MCU_ADDRESS [SEQ_CAP_NUMFRAMES]
{0x0990, 0x000A},      // MCU_DATA_0
{0x098C, 0xA103},     // MCU_ADDRESS [SEQ_CMD]
{0x0990, 0x0002},      // MCU_DATA_0
{SENSOR_WAIT_MS, 100}, //delay=100
{SENSOR_TABLE_END, 0x0000}
};


static struct sensor_reg mode_640x480[] = {
    {0x3103, 0x11},
    {0x3008, 0x82},
    {0x3008, 0x42},
    {0x3103, 0x03},
    {0x3017, 0x00},
    {0x3018, 0x00},
    {0x3034, 0x18},
    {0x3035, 0x18}, //;15fps- 0x28
    {0x3036, 0x38},
    {0x3037, 0x13},
    {0x3108, 0x01},
    {0x3630, 0x2e},
    {0x3632, 0xe2},
    {0x3633, 0x23},
    {0x3621, 0xe0},
    {0x3704, 0xa0},
    {0x3703, 0x5a},
    {0x3715, 0x78},
    {0x3717, 0x01},
    {0x370b, 0x60},
    {0x3705, 0x1a},
    {0x3905, 0x02},
    {0x3906, 0x10},
    {0x3901, 0x0a},
    {0x3731, 0x12},
    {0x3600, 0x08},
    {0x3601, 0x33},
    {0x302d, 0x60},
    {0x3620, 0x52},
    {0x371b, 0x20},
    {0x471c, 0x50},
    {0x3a13, 0x43},
    {0x3a18, 0x00},
    {0x3a19, 0xf8},
    {0x3635, 0x1c},
    {0x3634, 0x40},
    {0x3622, 0x01},
    {0x3c01, 0x34},
    {0x3c04, 0x28},
    {0x3c05, 0x98},
    {0x3c06, 0x00},
    {0x3c07, 0x08},
    {0x3c08, 0x00},
    {0x3c09, 0x1c},
    {0x3c0a, 0x9c},
    {0x3c0b, 0x40},
    {0x3820, 0x41},
    {0x3821, 0x07},
    {0x3814, 0x31},
    {0x3815, 0x31},
    {0x3800, 0x00},
    {0x3801, 0x00},
    {0x3802, 0x00},
    {0x3803, 0x04},
    {0x3804, 0x0a},
    {0x3805, 0x3f},
    {0x3806, 0x07},
    {0x3807, 0x9b},
    {0x3808, 0x01},
    {0x3809, 0x40},
    {0x380a, 0x00},
    {0x380b, 0xf0},
    {0x380c, 0x07},
    {0x380d, 0x68},
    {0x380e, 0x03},
    {0x380f, 0xd8},
    {0x3810, 0x00},
    {0x3811, 0x10},
    {0x3812, 0x00},
    {0x3813, 0x06},
    {0x3618, 0x00},
    {0x3612, 0x29},
    {0x3708, 0x62},
    {0x3709, 0x52},
    {0x370c, 0x03},
    {0x3a02, 0x03},
    {0x3a03, 0xd8},
    {0x3a08, 0x01},
    {0x3a09, 0x27},
    {0x3a0a, 0x00},
    {0x3a0b, 0xf6},
    {0x3a0e, 0x03},
    {0x3a0d, 0x04},
    {0x3a14, 0x03},
    {0x3a15, 0xd8},
    {0x4001, 0x02},
    {0x4004, 0x02},
    {0x3000, 0x00},
    {0x3002, 0x1c},
    {0x3004, 0xff},
    {0x3006, 0xc3},
    {0x300e, 0x45},
    {0x302e, 0x08},
    {0x4300, 0x32},
    {0x501f, 0x00},
    {0x4713, 0x03},
    {0x4407, 0x04},
    {0x460b, 0x35},
    {0x460c, 0x22},
    {0x3824, 0x02},
    {0x5000, 0xa7},
    {0x5001, 0xa3},
    {0x5180, 0xff},
    {0x5181, 0xf2},
    {0x5182, 0x00},
    {0x5183, 0x14},
    {0x5184, 0x25},
    {0x5185, 0x24},
    {0x5186, 0x09},
    {0x5187, 0x09},
    {0x5188, 0x09},
    {0x5189, 0x75},
    {0x518a, 0x54},
    {0x518b, 0xe0},
    {0x518c, 0xb2},
    {0x518d, 0x42},
    {0x518e, 0x3d},
    {0x518f, 0x56},
    {0x5190, 0x46},
    {0x5191, 0xf8},
    {0x5192, 0x04},
    {0x5193, 0x70},
    {0x5194, 0xf0},
    {0x5195, 0xf0},
    {0x5196, 0x03},
    {0x5197, 0x01},
    {0x5198, 0x04},
    {0x5199, 0x12},
    {0x519a, 0x04},
    {0x519b, 0x00},
    {0x519c, 0x06},
    {0x519d, 0x82},
    {0x519e, 0x38},
    {0x5381, 0x1c},
    {0x5382, 0x5a},
    {0x5383, 0x06},
    {0x5384, 0x0a},
    {0x5385, 0x7e},
    {0x5386, 0x88},
    {0x5387, 0x7c},
    {0x5388, 0x6c},
    {0x5389, 0x10},
    {0x538a, 0x01},
    {0x538b, 0x98},
    {0x5300, 0x08},
    {0x5301, 0x30},
    {0x5302, 0x10},
    {0x5303, 0x00},
    {0x5304, 0x08},
    {0x5305, 0x30},
    {0x5306, 0x08},
    {0x5307, 0x16},
    {0x5309, 0x08},
    {0x530a, 0x30},
    {0x530b, 0x04},
    {0x530c, 0x06},
    {0x5480, 0x01},
    {0x5481, 0x08},
    {0x5482, 0x14},
    {0x5483, 0x28},
    {0x5484, 0x51},
    {0x5485, 0x65},
    {0x5486, 0x71},
    {0x5487, 0x7d},
    {0x5488, 0x87},
    {0x5489, 0x91},
    {0x548a, 0x9a},
    {0x548b, 0xaa},
    {0x548c, 0xb8},
    {0x548d, 0xcd},
    {0x548e, 0xdd},
    {0x548f, 0xea},
    {0x5490, 0x1d},
    {0x5580, 0x02},
    {0x5583, 0x40},
    {0x5584, 0x10},
    {0x5589, 0x10},
    {0x558a, 0x00},
    {0x558b, 0xf8},
    {0x5800, 0x23},
    {0x5801, 0x14},
    {0x5802, 0x0f},
    {0x5803, 0x0f},
    {0x5804, 0x12},
    {0x5805, 0x26},
    {0x5806, 0x0c},
    {0x5807, 0x08},
    {0x5808, 0x05},
    {0x5809, 0x05},
    {0x580a, 0x08},
    {0x580b, 0x0d},
    {0x580c, 0x08},
    {0x580d, 0x03},
    {0x580e, 0x00},
    {0x580f, 0x00},
    {0x5810, 0x03},
    {0x5811, 0x09},
    {0x5812, 0x07},
    {0x5813, 0x03},
    {0x5814, 0x00},
    {0x5815, 0x01},
    {0x5816, 0x03},
    {0x5817, 0x08},
    {0x5818, 0x0d},
    {0x5819, 0x08},
    {0x581a, 0x05},
    {0x581b, 0x06},
    {0x581c, 0x08},
    {0x581d, 0x0e},
    {0x581e, 0x29},
    {0x581f, 0x17},
    {0x5820, 0x11},
    {0x5821, 0x11},
    {0x5822, 0x15},
    {0x5823, 0x28},
    {0x5824, 0x46},
    {0x5825, 0x26},
    {0x5826, 0x08},
    {0x5827, 0x26},
    {0x5828, 0x64},
    {0x5829, 0x26},
    {0x582a, 0x24},
    {0x582b, 0x22},
    {0x582c, 0x24},
    {0x582d, 0x24},
    {0x582e, 0x06},
    {0x582f, 0x22},
    {0x5830, 0x40},
    {0x5831, 0x42},
    {0x5832, 0x24},
    {0x5833, 0x26},
    {0x5834, 0x24},
    {0x5835, 0x22},
    {0x5836, 0x22},
    {0x5837, 0x26},
    {0x5838, 0x44},
    {0x5839, 0x24},
    {0x583a, 0x26},
    {0x583b, 0x28},
    {0x583c, 0x42},
    {0x583d, 0xce},
    {0x5025, 0x00},
    {0x3a0f, 0x30},
    {0x3a10, 0x28},
    {0x3a1b, 0x30},
    {0x3a1e, 0x26},
    {0x3a11, 0x60},
    {0x3a1f, 0x14},
    {0x3008, 0x02},
    {0x3808, 0x02},
    {0x3809, 0x80},
    {0x380a, 0x01},
    {0x380b, 0xe0},
    {0x3035, 0x14},
    {0x4800, 0x34},
    {SENSOR_WAIT_MS, 500},
    {SENSOR_TABLE_END, 0x0000}

};

static struct sensor_reg mode_2592x1944[] = {
    {0x3008, 0x82},
    {SENSOR_WAIT_MS, 0x05}, //;sw reset should wait 5 ms
    {0x3008, 0x42},
    {0x3103, 0x03},
    {0x3017, 0x00},
    {0x3018, 0x00},
    {0x3630, 0x2e},
    {0x3632, 0xe2},
    {0x3633, 0x23},
    {0x3634, 0x44},
    {0x3621, 0xe0},
    {0x3704, 0xa0},
    {0x3703, 0x5a},
    {0x3715, 0x78},
    {0x3717, 0x01},
    {0x370b, 0x60},
    {0x3705, 0x1a},
    {0x3905, 0x02},
    {0x3906, 0x10},
    {0x3901, 0x0a},
    {0x3731, 0x12},
    {0x3600, 0x04},
    {0x3601, 0x22},
    {0x471c, 0x50},
    {0x3a18, 0x00},
    {0x3a19, 0xf8},
    {0x3503, 0x07},
    {0x3500, 0x00},
    {0x3501, 0x01},
    {0x3502, 0x00},
    {0x350a, 0x00},
    {0x350b, 0x3f},
    {0x3002, 0x1c},
    {0x3006, 0xc3},
    {0x300e, 0x45},
    {0x302e, 0x08},
    {0x3612, 0x4b},
    {0x3618, 0x04},
    {0x3034, 0x18},
    {0x3035, 0x11},
    {0x3036, 0x54},
    {0x3708, 0x21},
    {0x3709, 0x12},
    {0x370c, 0x00},
    {0x3800, 0x00},
    {0x3801, 0x00},
    {0x3802, 0x00},
    {0x3803, 0x00},
    {0x3804, 0x0a},
    {0x3805, 0x3f},
    {0x3806, 0x07},
    {0x3807, 0x9f},
    {0x3808, 0x0a},
    {0x3809, 0x20},
    {0x380a, 0x07},
    {0x380b, 0x98},
    {0x380c, 0x0b},
    {0x380d, 0x1c},
    {0x380e, 0x07},
    {0x380f, 0xb0},
    {0x3810, 0x00},
    {0x3811, 0x10},
    {0x3812, 0x00},
    {0x3813, 0x06},
    {0x3814, 0x11},
    {0x3815, 0x11},
    {0x3820, 0x40},
    {0x3821, 0x06},
    {0x3824, 0x01},
    {0x3a02, 0x07},
    {0x3a03, 0xb0},
    {0x3a08, 0x01},
    {0x3a09, 0x27},
    {0x3a0a, 0x00},
    {0x3a0b, 0xf6},
    {0x3a0e, 0x06},
    {0x3a0d, 0x08},
    {0x3a14, 0x07},
    {0x3a15, 0xb0},
    {0x4001, 0x02},
    {0x4004, 0x06},
    {0x4300, 0x32},
    {0x460b, 0x37},
    {0x460c, 0x20},
    {0x4713, 0x02},
    {0x4750, 0x00},
    {0x4751, 0x00},
    {0x5000, 0x07},
    {0x5001, 0x03},
    {0x501d, 0x00},
    {0x501f, 0x00},
    {0x5684, 0x10},
    {0x5685, 0xa0},
    {0x5686, 0x0c},
    {0x5687, 0x78},
    {0x5a00, 0x08},
    {0x5a21, 0x00},
    {0x5a24, 0x00},
    {0x5000, 0x27},
    {0x5001, 0x83},
    {0x3821, 0x06},
    {0x5481, 0x08},
    {0x5482, 0x14},
    {0x5483, 0x28},
    {0x5484, 0x51},
    {0x5485, 0x65},
    {0x5486, 0x71},
    {0x5487, 0x7d},
    {0x5488, 0x87},
    {0x5489, 0x91},
    {0x548a, 0x9a},
    {0x548b, 0xaa},
    {0x548c, 0xb8},
    {0x548d, 0xcd},
    {0x548e, 0xdd},
    {0x548f, 0xea},
    {0x5490, 0x1d},
    {0x5381, 0x20},
    {0x5382, 0x64},
    {0x5383, 0x08},
    {0x5384, 0x20},
    {0x5385, 0x80},
    {0x5386, 0xa0},
    {0x5387, 0xa2},
    {0x5388, 0xa0},
    {0x5389, 0x02},
    {0x538a, 0x01},
    {0x538b, 0x98},
    {0x5300, 0x08},
    {0x5301, 0x30},
    {0x5302, 0x10},
    {0x5303, 0x00},
    {0x5304, 0x08},
    {0x5305, 0x30},
    {0x5306, 0x08},
    {0x5307, 0x16},
    {0x5580, 0x02},
    {0x5583, 0x40},
    {0x5584, 0x10},
    {0x5589, 0x10},
    {0x558a, 0x00},
    {0x558b, 0xf8},
    {0x3a0f, 0x36},
    {0x3a10, 0x2e},
    {0x3a1b, 0x38},
    {0x3a1e, 0x2c},
    {0x3a11, 0x70},
    {0x4800, 0x34},
    {0x3a1f, 0x18},
    {0x3a18, 0x00},
    {0x3a19, 0xf8},
    {0x3003, 0x03},
    {0x3003, 0x01},
    {0x3008, 0x02},
    {0x3503, 0x00},
    {SENSOR_WAIT_MS, 100}, //;wait at least 3 frames  // julian remove
    {0x4837, 0x09},
    {SENSOR_TABLE_END, 0x0000}

};

static struct sensor_reg ColorEffect_None[] = {
{0x098C, 0x2759},        //content A
{0x0990, 0x0000},
{0x098C, 0x275B},        //content B
{0x0990, 0x0000},
{0x098C, 0xA103},        //MCU_ADDRESS [SEQ_CMD]
{0x0990, 0x0006},        //MCU_DATA_0
{SENSOR_TABLE_END, 0x0000}
};

static struct sensor_reg ColorEffect_Mono[] = {
{0x098C, 0x2759},        //content A
{0x0990, 0x0001},
{0x098C, 0x275B},        //content B
{0x0990, 0x0001},        
{0x098C, 0xA103}, 	 //MCU_ADDRESS [SEQ_CMD]
{0x0990, 0x0006}, 	 //MCU_DATA_0
{SENSOR_TABLE_END, 0x0000}
};

static struct sensor_reg ColorEffect_Sepia[] = {
{0x098C, 0x2759},        //content A
{0x0990, 0x0002},
{0x098C, 0x275B},        //content B
{0x0990, 0x0002},
{0x098C, 0xA103},        //MCU_ADDRESS [SEQ_CMD]
{0x0990, 0x0006},        //MCU_DATA_0
{SENSOR_TABLE_END, 0x0000}
};

static struct sensor_reg ColorEffect_Negative[] = {
{0x098C, 0x2759},        //content A
{0x0990, 0x0003},
{0x098C, 0x275B},        //content B
{0x0990, 0x0003},
{0x098C, 0xA103},        //MCU_ADDRESS [SEQ_CMD]
{0x0990, 0x0006},        //MCU_DATA_0
{SENSOR_TABLE_END, 0x0000}
};

static struct sensor_reg ColorEffect_Solarize[] = {
{0x098C, 0x2759},        //content A
{0x0990, 0x0004},
{0x098C, 0x275B},        //content B
{0x0990, 0x0004},
{0x098C, 0xA103},        //MCU_ADDRESS [SEQ_CMD]
{0x0990, 0x0006},        //MCU_DATA_0
{SENSOR_TABLE_END, 0x0000}
};

//Sensor ISP Not Support this function
static struct sensor_reg ColorEffect_Posterize[] = {
{SENSOR_TABLE_END, 0x0000}
};

static struct sensor_reg Whitebalance_Auto[] = {
{0x098C, 0xA115},       // MCU_ADDRESS [SEQ_CAP_MODE]
{0x0990, 0x0000},       // MCU_DATA_0
{0x098C, 0xA11F},       // MCU_ADDRESS [SEQ_PREVIEW_1_AWB]
{0x0990, 0x0001},       // MCU_DATA_0
{0x098C, 0xA103},       // MCU_ADDRESS [SEQ_CMD]
{0x0990, 0x0005},       // MCU_DATA_0
{SENSOR_TABLE_END, 0x0000}
};

static struct sensor_reg Whitebalance_Incandescent[] = {
{0x098C, 0xA115 },      // MCU_ADDRESS [SEQ_CAP_MODE]
{0x0990, 0x0000 },      // MCU_DATA_0
{0x098C, 0xA11F },      // MCU_ADDRESS [SEQ_PREVIEW_1_AWB]
{0x0990, 0x0000 },      // MCU_DATA_0
{0x098C, 0xA103 },      // MCU_ADDRESS [SEQ_CMD]
{0x0990, 0x0005 },      // MCU_DATA_0
{0x098C, 0xA353 },      // MCU_ADDRESS [AWB_CCM_POSITION]
{0x0990, 0x002B },      // MCU_DATA_0
{0x098C, 0xA34E },      // MCU_ADDRESS [AWB_GAIN_R]
{0x0990, 0x007B },      // MCU_DATA_0
{0x098C, 0xA34F },      // MCU_ADDRESS [AWB_GAIN_G]
{0x0990, 0x0080 },      // MCU_DATA_0
{0x098C, 0xA350 },      // MCU_ADDRESS [AWB_GAIN_B]
{0x0990, 0x007E },      // MCU_DATA_0
{SENSOR_TABLE_END, 0x0000}
};

static struct sensor_reg Whitebalance_Daylight[] = {
{0x098C, 0xA115 },       // MCU_ADDRESS [SEQ_CAP_MODE]
{0x0990, 0x0000 },       // MCU_DATA_0
{0x098C, 0xA11F },       // MCU_ADDRESS [SEQ_PREVIEW_1_AWB]
{0x0990, 0x0000 },       // MCU_DATA_0
{0x098C, 0xA103 },       // MCU_ADDRESS [SEQ_CMD]
{0x0990, 0x0005 },       // MCU_DATA_0
{0x098C, 0xA353 },       // MCU_ADDRESS [AWB_CCM_POSITION]
{0x0990, 0x007F },       // MCU_DATA_0
{0x098C, 0xA34E },       // MCU_ADDRESS [AWB_GAIN_R]
{0x0990, 0x008E },       // MCU_DATA_0
{0x098C, 0xA34F },       // MCU_ADDRESS [AWB_GAIN_G]
{0x0990, 0x0080 },       // MCU_DATA_0
{0x098C, 0xA350 },       // MCU_ADDRESS [AWB_GAIN_B]
{0x0990, 0x0074 },       // MCU_DATA_0
{SENSOR_TABLE_END, 0x0000}
};

static struct sensor_reg Whitebalance_Fluorescent[] = {
{0x098C, 0xA115  },     // MCU_ADDRESS [SEQ_CAP_MODE]
{0x0990, 0x0000  },     // MCU_DATA_0
{0x098C, 0xA11F  },     // MCU_ADDRESS [SEQ_PREVIEW_1_AWB]
{0x0990, 0x0000  },     // MCU_DATA_0
{0x098C, 0xA103  },     // MCU_ADDRESS [SEQ_CMD]
{0x0990, 0x0005  },     // MCU_DATA_0
{0x098C, 0xA353  },     // MCU_ADDRESS [AWB_CCM_POSITION]
{0x0990, 0x0036  },     // MCU_DATA_0
{0x098C, 0xA34E  },     // MCU_ADDRESS [AWB_GAIN_R]
{0x0990, 0x0099  },     // MCU_DATA_0
{0x098C, 0xA34F  },     // MCU_ADDRESS [AWB_GAIN_G]
{0x0990, 0x0080  },     // MCU_DATA_0
{0x098C, 0xA350  },     // MCU_ADDRESS [AWB_GAIN_B]
{0x0990, 0x007D  },     // MCU_DATA_0
{SENSOR_TABLE_END, 0x0000}
};


enum {
//Kenji-	SENSOR_MODE_1600x1200,
//Kenji-	SENSOR_MODE_1280x720,
	SENSOR_MODE_2592x1944,		//Kenji+
	SENSOR_MODE_640x480,
};

static struct sensor_reg *mode_table[] = {
//Kenji-	[SENSOR_MODE_1600x1200] = mode_1600x1200,
//Kenji-	[SENSOR_MODE_1280x720]  = mode_1280x720,
	[SENSOR_MODE_2592x1944]  = mode_2592x1944,	//Kenji+
	[SENSOR_MODE_640x480]   = mode_640x480,
};

static int sensor_read_reg(struct i2c_client *client, u16 addr, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[4];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	//Kenji- msg[1].len = 2;
	msg[1].len = 1;		//Kenji+
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

        //Kenji- swap(*(data+2),*(data+3)); //swap high and low byte to match table format
	//Kenji- memcpy(val, data+2, 2);
	memcpy(val, data+2, 1);		//Kenji+
	return 0;
}

static int sensor_write_reg(struct i2c_client *client, u16 addr, u16 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[4];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
        //Kenji- data[2] = (u8) (val >> 8);
	//Kenji- data[3] = (u8) (val & 0xff);
	data[2] = (u8) (val & 0xff);	//Kenji+
	
	msg.addr = client->addr;
	msg.flags = 0;
	//Kenji-msg.len = 4;
	msg.len = 3;	//Kenji+
	msg.buf = data;
//msg.addr = 0;	//Kenji+
	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("yuv_sensor : i2c transfer failed, retrying %x %x\n",
		       addr, val);
		pr_err("yuv_sensor : i2c transfer failed, count %x \n",
		       msg.addr);
//msg.addr++;	//Kenji+
		msleep(3);
	} while (retry <= SENSOR_MAX_RETRIES);
//(msg.addr!=0x80);//Kenji- (retry <= SENSOR_MAX_RETRIES);

	return err;
}

static int sensor_write_table(struct i2c_client *client,
			      const struct sensor_reg table[])
{
	int err;
	const struct sensor_reg *next;
	u16 val;

	pr_info("yuv %s\n",__func__);
	for (next = table; next->addr != SENSOR_TABLE_END; next++) {
		if (next->addr == SENSOR_WAIT_MS) {
			msleep(next->val);
			continue;
		}

		val = next->val;

		err = sensor_write_reg(client, next->addr, val);
		if (err)
			return err;
	}
	return 0;
}

static int get_sensor_current_width(struct i2c_client *client, u16 *val)
{
        int err;
         
        err = sensor_write_reg(client, 0x098c, 0x2703);
        if (err)
          return err;

        err = sensor_read_reg(client, 0x0990, val);

        if (err)
          return err;

        return 0;
}

static int sensor_set_mode(struct sensor_info *info, struct sensor_mode *mode)
{
	int sensor_table;
	int err;
        u16 val;

	pr_info("%s: xres %u yres %u\n",__func__, mode->xres, mode->yres);

//Kenji-	if (mode->xres == 1600 && mode->yres == 1200)
//Kenji-		sensor_table = SENSOR_MODE_1600x1200;
	if (mode->xres == 2592 && mode->yres == 1944)	//Kenji+
		sensor_table = SENSOR_MODE_2592x1944;		//Kenji+
//Kenji-	else if (mode->xres == 1280 && mode->yres == 720)
//Kenji-		sensor_table = SENSOR_MODE_1280x720;
	else if (mode->xres == 640 && mode->yres == 480)
		sensor_table = SENSOR_MODE_640x480;
	else {
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
		       __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

        //Kenji- err = get_sensor_current_width(info->i2c_client, &val);

        //check already program the sensor mode, Aptina support Context B fast switching capture mode back to preview mode
        //we don't need to re-program the sensor mode for 640x480 table
        if(!(val == SENSOR_640_WIDTH_VAL && sensor_table == SENSOR_MODE_640x480))
        {
	       err = sensor_write_table(info->i2c_client, mode_table[sensor_table]);
	       if (err)
		 return err;
        }

	info->mode = sensor_table;
	return 0;
}

static long sensor_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	struct sensor_info *info = file->private_data;
        int err=0;

	pr_info("yuv %s\n",__func__);
	switch (cmd) {
	case SENSOR_IOCTL_SET_MODE:
	{
		struct sensor_mode mode;
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct sensor_mode))) {
			return -EFAULT;
		}

		return sensor_set_mode(info, &mode);
	}
	case SENSOR_IOCTL_GET_STATUS:
	{

		return 0;
	}
        case SENSOR_IOCTL_SET_COLOR_EFFECT:
        {
                u8 coloreffect;
                return 0;	//Kenji+

		if (copy_from_user(&coloreffect,
				   (const void __user *)arg,
				   sizeof(coloreffect))) {
			return -EFAULT;
		}

                switch(coloreffect)
                {
                    case YUV_ColorEffect_None:
	                 err = sensor_write_table(info->i2c_client, ColorEffect_None);
                         break;
                    case YUV_ColorEffect_Mono:
	                 err = sensor_write_table(info->i2c_client, ColorEffect_Mono);
                         break;
                    case YUV_ColorEffect_Sepia:
	                 err = sensor_write_table(info->i2c_client, ColorEffect_Sepia);
                         break;
                    case YUV_ColorEffect_Negative:
	                 err = sensor_write_table(info->i2c_client, ColorEffect_Negative);
                         break;
                    case YUV_ColorEffect_Solarize:
	                 err = sensor_write_table(info->i2c_client, ColorEffect_Solarize);
                         break;
                    case YUV_ColorEffect_Posterize:
	                 err = sensor_write_table(info->i2c_client, ColorEffect_Posterize);
                         break;
                    default:
                         break;
                }

	        if (err)
		   return err;

                return 0;
        }
        case SENSOR_IOCTL_SET_WHITE_BALANCE:
        {
                u8 whitebalance;
                return 0;	//Kenji+

		if (copy_from_user(&whitebalance,
				   (const void __user *)arg,
				   sizeof(whitebalance))) {
			return -EFAULT;
		}

                switch(whitebalance)
                {
                    case YUV_Whitebalance_Auto:
	                 err = sensor_write_table(info->i2c_client, Whitebalance_Auto);
                         break;
                    case YUV_Whitebalance_Incandescent:
	                 err = sensor_write_table(info->i2c_client, Whitebalance_Incandescent);
                         break;
                    case YUV_Whitebalance_Daylight:
	                 err = sensor_write_table(info->i2c_client, Whitebalance_Daylight);
                         break;
                    case YUV_Whitebalance_Fluorescent:
	                 err = sensor_write_table(info->i2c_client, Whitebalance_Fluorescent);
                         break;
                    default:
                         break;
                }

	        if (err)
		   return err;

                return 0;

                return 0;
        }
        case SENSOR_IOCTL_SET_SCENE_MODE:
        {
                return 0;
        }
	default:
		return -EINVAL;
	}
	return 0;
}

static int sensor_open(struct inode *inode, struct file *file)
{
	pr_info("yuv %s\n",__func__);
	file->private_data = info;
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();
	return 0;
}

int sensor_release(struct inode *inode, struct file *file)
{
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();
	file->private_data = NULL;
	return 0;
}


static const struct file_operations sensor_fileops = {
	.owner = THIS_MODULE,
	.open = sensor_open,
	.unlocked_ioctl = sensor_ioctl,
	.release = sensor_release,
};

static struct miscdevice sensor_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = SENSOR_NAME,
	.fops = &sensor_fileops,
};

static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	pr_info("yuv %s\n",__func__);

	info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);

	if (!info) {
		pr_err("yuv_sensor : Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&sensor_device);
	if (err) {
		pr_err("yuv_sensor : Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;

	i2c_set_clientdata(client, info);
	return 0;
}

static int sensor_remove(struct i2c_client *client)
{
	struct sensor_info *info;

	pr_info("yuv %s\n",__func__);
	info = i2c_get_clientdata(client);
	misc_deregister(&sensor_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ SENSOR_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.name = SENSOR_NAME,
		.owner = THIS_MODULE,
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};

static int __init sensor_init(void)
{
	pr_info("yuv %s\n",__func__);
	return i2c_add_driver(&sensor_i2c_driver);
}

static void __exit sensor_exit(void)
{
	pr_info("yuv %s\n",__func__);
	i2c_del_driver(&sensor_i2c_driver);
}

module_init(sensor_init);
module_exit(sensor_exit);

