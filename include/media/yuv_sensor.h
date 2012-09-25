#ifndef ___YUV_SENSOR_H__
#define ___YUV_SENSOR_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

/*-------------------------------------------Important---------------------------------------
 * for changing the SENSOR_NAME, you must need to change the owner of the device. For example
 * Please add /dev/mt9d115 0600 media camera in below file
 * ./device/nvidia/ventana/ueventd.ventana.rc
 * Otherwise, ioctl will get permission deny
 * -------------------------------------------Important--------------------------------------
*/

#define SENSOR_NAME	"mt9d115"
#define DEV(x)          "/dev/"x
#define SENSOR_PATH     DEV(SENSOR_NAME)
#define LOG_NAME(x)     "ImagerODM-"x
#define LOG_TAG         LOG_NAME(SENSOR_NAME)

#define SENSOR_WAIT_MS       0 /* special number to indicate this is wait time require */
#define SENSOR_TABLE_END     1 /* special number to indicate this is end of table */
#define SENSOR_BYTE_WRITE    2
#define SENSOR_WORD_WRITE    3
#define SENSOR_MASK_BYTE_WRITE  4
#define SENSOR_MASK_WORD_WRITE  5
#define SEQ_WRITE_START 6
#define SEQ_WRITE_END 7

#define SENSOR_MAX_RETRIES   3 /* max counter for retry I2C access */

#define SENSOR_IOCTL_SET_MODE		_IOW('o', 1, struct sensor_mode)
#define SENSOR_IOCTL_GET_STATUS		_IOR('o', 2, __u8)
#define SENSOR_IOCTL_SET_COLOR_EFFECT   _IOW('o', 3, __u8)
#define SENSOR_IOCTL_SET_WHITE_BALANCE  _IOW('o', 4, __u8)
#define SENSOR_IOCTL_SET_SCENE_MODE     _IOW('o', 5, __u8)
#define SENSOR_IOCTL_SET_AF_MODE        _IOW('o', 6, __u8)
#define SENSOR_IOCTL_GET_AF_STATUS      _IOW('o', 7, __u8)

enum {
  ASUS_CUSTOM_IOCTL_NUMBASE = 40,
  ASUS_CUSTOM_IOCTL_AF_SET,
  ASUS_CUSTOM_IOCTL_GET_ID,
  ASUS_CUSTOM_IOCTL_SET_EV,
  ASUS_CUSTOM_IOCTL_GET_EV,
  ASUS_CUSTOM_IOCTL_AF_GET,
  ASUS_CUSTOM_IOCTL_GET_ET,
  ASUS_CUSTOM_IOCTL_SET_AE_LOCK,
  ASUS_CUSTOM_IOCTL_SET_AWB_LOCK,
  ASUS_CUSTOM_IOCTL_GET_AE_LOCK,
  ASUS_CUSTOM_IOCTL_GET_AWB_LOCK
};

#define  AF_CMD_START 0
#define  AF_CMD_ABORT 1
#define  AF_CMD_SET_POSITION  2
#define  AF_CMD_SET_WINDOW_POSITION 3
#define  AF_CMD_SET_WINDOW_SIZE 4
#define  AF_CMD_SET_AFMODE  5
#define  AF_CMD_SET_CAF 6
#define  AF_CMD_GET_AF_STATUS 7

typedef struct
{
	int 		cmd;
	int 		data;
} custom_af_cmd_package;

typedef struct
{
	unsigned int 		exposure;
	unsigned int 		vts;
} custom_et_value_package;

#define SENSOR_CUSTOM_IOCTL_SET_AF_MODE _IOW('o', ASUS_CUSTOM_IOCTL_AF_SET, custom_af_cmd_package)
#define SENSOR_CUSTOM_IOCTL_GET_AF_MODE _IOWR('o', ASUS_CUSTOM_IOCTL_AF_GET, custom_af_cmd_package)
#define SENSOR_CUSTOM_IOCTL_GET_ID _IOW('o', ASUS_CUSTOM_IOCTL_GET_ID, __u16)
#define SENSOR_CUSTOM_IOCTL_SET_EV _IOW('o', ASUS_CUSTOM_IOCTL_SET_EV, __s16)
#define SENSOR_CUSTOM_IOCTL_GET_EV _IOR('o', ASUS_CUSTOM_IOCTL_GET_EV, __s16)
#define SENSOR_CUSTOM_IOCTL_GET_ET _IOR('o', ASUS_CUSTOM_IOCTL_GET_ET, __s16)
#define SENSOR_CUSTOM_IOCTL_SET_AE_LOCK       _IOW('o', ASUS_CUSTOM_IOCTL_SET_AE_LOCK, __u32)
#define SENSOR_CUSTOM_IOCTL_SET_AWB_LOCK      _IOW('o', ASUS_CUSTOM_IOCTL_SET_AWB_LOCK, __u32)
#define SENSOR_CUSTOM_IOCTL_GET_AE_LOCK       _IOWR('o', ASUS_CUSTOM_IOCTL_GET_AE_LOCK, __u32)
#define SENSOR_CUSTOM_IOCTL_GET_AWB_LOCK      _IOWR('o', ASUS_CUSTOM_IOCTL_GET_AWB_LOCK, __u32)

enum {
      YUV_ColorEffect = 0,
      YUV_Whitebalance,
      YUV_SceneMode
};

enum {
      YUV_ColorEffect_Invalid = 0,
      YUV_ColorEffect_Aqua,
      YUV_ColorEffect_Blackboard,
      YUV_ColorEffect_Mono,
      YUV_ColorEffect_Negative,
      YUV_ColorEffect_None,
      YUV_ColorEffect_Posterize,
      YUV_ColorEffect_Sepia,
      YUV_ColorEffect_Solarize,
      YUV_ColorEffect_Whiteboard
};

enum {
      YUV_Whitebalance_Invalid = 0,
      YUV_Whitebalance_Auto,
      YUV_Whitebalance_Incandescent,
      YUV_Whitebalance_Fluorescent,
      YUV_Whitebalance_WarmFluorescent,
      YUV_Whitebalance_Daylight,
      YUV_Whitebalance_CloudyDaylight,
      YUV_Whitebalance_Shade,
      YUV_Whitebalance_Twilight,
      YUV_Whitebalance_Custom
};

enum {
      YUV_SceneMode_Invalid = 0,
      YUV_SceneMode_Auto,
      YUV_SceneMode_Action,
      YUV_SceneMode_Portrait,
      YUV_SceneMode_Landscape,
      YUV_SceneMode_Beach,
      YUV_SceneMode_Candlelight,
      YUV_SceneMode_Fireworks,
      YUV_SceneMode_Night,
      YUV_SceneMode_NightPortrait,
      YUV_SceneMode_Party,
      YUV_SceneMode_Snow,
      YUV_SceneMode_Sports,
      YUV_SceneMode_SteadyPhoto,
      YUV_SceneMode_Sunset,
      YUV_SceneMode_Theatre,
      YUV_SceneMode_Barcode
};

struct sensor_mode {
	int xres;
	int yres;
};

#ifdef __KERNEL__
struct yuv_sensor_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */

#endif  /* __YUV_SENSOR_H__ */

