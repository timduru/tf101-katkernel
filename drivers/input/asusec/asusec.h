#ifndef _ASUSEC_H
#define _ASUSEC_H

#include <linux/switch.h>
#include <linux/wakelock.h>

/*
 * compiler option
 */
#define ASUSEC_DEBUG			0
#define TOUCHPAD_MODE			1	// 0: relative mode, 1: absolute mode
#define ASUSEC_INTERRUPT_DRIVEN		1	// 0: polling, 1: interrupt
#define TOUCHPAD_ELAN			1	// 0: not elan, 1:elantech
#define CALLBACK_READY			1	// 0: not ready, 1: ready
/*
 * Debug Utility
 */
#if ASUSEC_DEBUG
#define ASUSEC_INFO(format, arg...)	\
	printk(KERN_INFO "asusec: [%s] " format , __FUNCTION__ , ## arg)
#define ASUSEC_I2C_DATA(array, i)	\
					do {		\
						for (i = 0; i < array[0]+1; i++) \
							ASUSEC_INFO("ec_data[%d] = 0x%x\n", i, array[i]);	\
					} while(0)
#else
#define ASUSEC_INFO(format, arg...)	 
#define ASUSEC_I2C_DATA(array, i)
#endif

#define ASUSEC_NOTICE(format, arg...)	\
	printk(KERN_NOTICE "asusec: [%s] " format , __FUNCTION__ , ## arg)

#define ASUSEC_ERR(format, arg...)	\
	printk(KERN_ERR "asusec: [%s] " format , __FUNCTION__ , ## arg)

//-----------------------------------------	       

#define DRIVER_DESC     		"ASUS EC driver"
#define DOCK_SDEV_NAME			"dock"
#define CONVERSION_TIME_MS		50

#define ASUSEC_RETRY_COUNT		3
#define ASUSEC_POLLING_RATE		80

#define ASUSEC_OBF_MASK			0x1
#define ASUSEC_KEY_MASK			0x4
#define ASUSEC_KBC_MASK			0x8
#define ASUSEC_AUX_MASK			0x20
#define ASUSEC_SCI_MASK			0x40
#define ASUSEC_SMI_MASK			0x80


#define ASUSEC_RELATIVE_MODE		0
#define ASUSEC_ABSOLUTE_MODE		1

#define NUM_RELATIVE_MODE		3	// The number of bytes per packet in relative mode
#define NUM_ABSOLUTE_MODE		6	// The number of bytes per packet in absolute mode

/* relative mode packet formate */
#define Y_OVERFLOW_MASK			0x80
#define X_OVERFLOW_MASK			0x40
#define Y_SIGN_MASK			0x20
#define X_SIGN_MASK			0x10
#define RIGHT_BTN_MASK			0x2
#define LEFT_BTN_MASK			0x1

/* absolute mode packet formate */
#define TOUCHPAD_SAMPLE_RATE		20
#define ABSOLUTE_BIT_MASK		0x80
#define Z_SNESITIVITY			30
#define X_LIMIT				5600
#define Y_LIMIT				1300
#define X_MAX				5855
#define X_MIN				1198
#define Y_MAX				4942
#define Y_MIN				946

#define ASUSEC_PS2_ACK			0xFA

//-----------------------------------------
#define TEGRA_GPIO_PS2			146 // AP_WAKE
#define TEGRA_GPIO_PS3			147 // EC_REQUEST
#define TEGRA_GPIO_PX5			189 // DOCK_IN	LOW: dock-in, HIGH: dock disconnected
#define TEGRA_GPIO_PS4			148 // HALL_SENSOR, SW_LID
//-----------------------------------------
#define ASUSEC_KEY_TOUCHPAD		KEY_F2
#define ASUSEC_KEY_AUTOBRIGHT	KEY_F3
#define ASUSEC_KEY_SETTING		KEY_F4

/*************scan 2 make mapping***************/
#define ASUSEC_KEYPAD_ESC		0x76
#define ASUSEC_KEYPAD_KEY_WAVE		0x0E
#define ASUSEC_KEYPAD_KEY_1		0x16
#define ASUSEC_KEYPAD_KEY_2		0X1E
#define ASUSEC_KEYPAD_KEY_3		0x26	
#define ASUSEC_KEYPAD_KEY_4		0x25
#define ASUSEC_KEYPAD_KEY_5		0x2E
#define ASUSEC_KEYPAD_KEY_6        	0x36
#define ASUSEC_KEYPAD_KEY_7        	0x3D
#define ASUSEC_KEYPAD_KEY_8        	0x3E
#define ASUSEC_KEYPAD_KEY_9        	0x46
#define ASUSEC_KEYPAD_KEY_0        	0x45
#define ASUSEC_KEYPAD_KEY_MINUS    	0x4E
#define ASUSEC_KEYPAD_KEY_EQUAL		0x55
#define ASUSEC_KEYPAD_KEY_BACKSPACE	0x66
#define ASUSEC_KEYPAD_KEY_TAB      	0x0D
#define ASUSEC_KEYPAD_KEY_Q        	0x15
#define ASUSEC_KEYPAD_KEY_W        	0x1D
#define ASUSEC_KEYPAD_KEY_E        	0x24
#define ASUSEC_KEYPAD_KEY_R        	0x2D
#define ASUSEC_KEYPAD_KEY_T        	0x2C
#define ASUSEC_KEYPAD_KEY_Y        	0x35
#define ASUSEC_KEYPAD_KEY_U        	0x3C
#define ASUSEC_KEYPAD_KEY_I        	0x43
#define ASUSEC_KEYPAD_KEY_O        	0x44
#define ASUSEC_KEYPAD_KEY_P        	0x4D
#define ASUSEC_KEYPAD_KEY_LEFTBRACE	0x54
#define ASUSEC_KEYPAD_KEY_RIGHTBRACE 	0x5B
#define ASUSEC_KEYPAD_KEY_BACKSLASH	0x5D
#define ASUSEC_KEYPAD_KEY_CAPSLOCK 	0x58
#define ASUSEC_KEYPAD_KEY_A        	0x1C
#define ASUSEC_KEYPAD_KEY_S        	0x1B
#define ASUSEC_KEYPAD_KEY_D        	0x23
#define ASUSEC_KEYPAD_KEY_F        	0x2B
#define ASUSEC_KEYPAD_KEY_G        	0x34
#define ASUSEC_KEYPAD_KEY_H        	0x33
#define ASUSEC_KEYPAD_KEY_J        	0x3B
#define ASUSEC_KEYPAD_KEY_K        	0x42
#define ASUSEC_KEYPAD_KEY_L        	0x4B
#define ASUSEC_KEYPAD_KEY_SEMICOLON	0x4C
#define ASUSEC_KEYPAD_KEY_APOSTROPHE	0x52
#define ASUSEC_KEYPAD_KEY_ENTER    	0x5A
#define ASUSEC_KEYPAD_KEY_LEFTSHIFT 	0x12
#define ASUSEC_KEYPAD_KEY_Z        	0x1A
#define ASUSEC_KEYPAD_KEY_X        	0x22
#define ASUSEC_KEYPAD_KEY_C        	0x21
#define ASUSEC_KEYPAD_KEY_V        	0x2A
#define ASUSEC_KEYPAD_KEY_B        	0x32
#define ASUSEC_KEYPAD_KEY_N        	0x31
#define ASUSEC_KEYPAD_KEY_M        	0x3A
#define ASUSEC_KEYPAD_KEY_COMMA    	0x41
#define ASUSEC_KEYPAD_KEY_DOT   	0x49
#define ASUSEC_KEYPAD_KEY_SLASH    	0x4A
#define ASUSEC_KEYPAD_KEY_RIGHTSHIFT   	0x59

#define ASUSEC_KEYPAD_KEY_LEFT   	0xE06B
#define ASUSEC_KEYPAD_KEY_RIGHT   	0xE074
#define ASUSEC_KEYPAD_KEY_UP		0xE075
#define ASUSEC_KEYPAD_KEY_DOWN		0xE072

#define ASUSEC_KEYPAD_RIGHTWIN		0xE027
#define ASUSEC_KEYPAD_LEFTCTRL		0x14
#define ASUSEC_KEYPAD_LEFTWIN		0xE01F
#define ASUSEC_KEYPAD_LEFTALT		0x11
#define ASUSEC_KEYPAD_KEY_SPACE		0x29
#define ASUSEC_KEYPAD_RIGHTALT		0xE011
#define ASUSEC_KEYPAD_WINAPP		0xE02F
#define ASUSEC_KEYPAD_RIGHTCTRL		0xE014
#define ASUSEC_KEYPAD_HOME			0xE06C
#define ASUSEC_KEYPAD_PAGEUP		0xE07D
#define ASUSEC_KEYPAD_PAGEDOWN		0xE07A
#define ASUSEC_KEYPAD_END			0xE069
/************  JP keys *************/                       
#define ASUSEC_HANKAKU_ZENKAKU		0x5F                
#define ASUSEC_YEN					0x6A
#define ASUSEC_MUHENKAN				0x67        
#define ASUSEC_HENKAN				0x64        
#define ASUSEC_HIRAGANA_KATAKANA	0x13
#define ASUSEC_RO					0x51
/********************************/    
/************  UK keys *************/
#define ASUSEC_EUROPE_2				0x61
/********************************/


#define ASUSEC_KEYPAD_LOCK			0xE071

#define ASUSEC_KEYPAD_KEY_BREAK   	0xF0
#define ASUSEC_KEYPAD_KEY_EXTEND   	0xE0

/*************scan 2 make code mapping***************/

/************* SMI event ********************/
#define ASUSEC_SMI_HANDSHAKING		0x50
#define ASUSEC_SMI_WAKE				0x53
#define ASUSEC_SMI_RESET			0x5F
/*************IO control setting***************/
#define ASUSEC_IOCTL_HEAVY	2
#define ASUSEC_IOCTL_NORMAL	1
#define ASUSEC_IOCTL_END	0
#define ASUSEC_CPAS_LED_ON	1
#define ASUSEC_CPAS_LED_OFF	0
#define ASUSEC_TP_ON	1
#define ASUSEC_TP_OFF	0
#define ASUSEC_EC_ON	1
#define ASUSEC_EC_OFF	0
#define ASUSEC_IOC_MAGIC	0xf4
#define ASUSEC_IOC_MAXNR	7
#define ASUSEC_POLLING_DATA _IOR(ASUSEC_IOC_MAGIC,	1,	int)
#define ASUSEC_FW_UPDATE 	_IOR(ASUSEC_IOC_MAGIC,	2,	int)
#define ASUSEC_CPASLOCK_LED	_IOR(ASUSEC_IOC_MAGIC,	3,	int)
#define ASUSEC_INIT			_IOR(ASUSEC_IOC_MAGIC,	4,	int)
#define ASUSEC_TP_CONTROL	_IOR(ASUSEC_IOC_MAGIC,	5,	int)
#define ASUSEC_EC_WAKEUP	_IOR(ASUSEC_IOC_MAGIC,	6,	int)
#define ASUSEC_FW_DUMMY		_IOR(ASUSEC_IOC_MAGIC,	7,	int)

/*************IO control setting***************/

/************* EC FW update ***********/
#define EC_BUFF_LEN  256
/********************** ***********/

/*
 * The x/y limits are taken from the Synaptics TouchPad interfacing Guide,
 * section 2.3.2, which says that they should be valid regardless of the
 * actual size of the sensor.
 */
#define XMIN_NOMINAL 0
#define XMAX_NOMINAL 1279
#define YMIN_NOMINAL 0
#define YMAX_NOMINAL 799

/*
 * data struct
 */

struct asusec_keypad{
	int value;
	int input_keycode;
	int extend;	
};

struct asusec_touchpad_relative{
	int y_overflow;
	int x_overflow;
	int y_sign;
	int x_sign;
	int left_btn;
	int right_btn;
	int delta_x;
	int delta_y;
};

struct asusec_touchpad_absolute{
	int w_val;
	int x_pos;
	int y_pos;
	int z_val;
	int left;
	int right;
	int x_prev;
	int y_prev;
	int z_prev;
	int x2_pos;
	int y2_pos;
	int z2_val;
};

struct asusec_chip {
	struct input_dev	*indev;
	struct switch_dev 	dock_sdev;
	struct i2c_client	*client;
	struct mutex		lock;
	struct mutex		kbc_lock;
	struct mutex		input_lock;
	struct mutex		dock_init_lock;
	struct wake_lock 	wake_lock;
	struct wake_lock 	wake_lock_init;
	struct delayed_work asusec_work;
	struct delayed_work asusec_dock_init_work;
	struct delayed_work asusec_fw_update_work;
	struct delayed_work asusec_led_on_work;
	struct delayed_work asusec_led_off_work;	
	struct asusec_keypad keypad_data;
	struct elantech_data *private;
	struct timer_list asusec_timer;
#if TOUCHPAD_MODE	
	struct asusec_touchpad_absolute t_abs;
#else
	struct asusec_touchpad_relative touchpad_data;
#endif
	int ret_val;
	u8 ec_data[32];
	u8 i2c_data[32];
	u8 i2c_dm_data[32];
	int bc;			// byte counter
	int index;		// for message
	int status;
	int touchpad_member;
	char ec_model_name[32];
	char ec_version[32];
	char dock_pid[32];
	int polling_rate;
	int dock_in;	// 0: without dock, 1: with dock
	int op_mode;	// 0: normal mode, 1: fw update mode	
	int kbc_value;	// capslock_led 0: led off, 1: led on
	int dock_det;	// dock-in interrupt count
	int dock_init;	// 0: dock not init, 1: dock init successfully
	int d_index;	// touchpad byte counter
	int suspend_state; // 0: normal, 1: suspend
	int init_success; // 0: ps/2 not ready. 1: init OK
	int wakeup_lcd;		// 0 : keep lcd state 1: make lcd on
	int tp_wait_ack;	// 0 : normal mode, 1: waiting for an ACK
	int tp_enable;		// 0 : touchpad has not enabled, 1: touchpad has enabled
	int re_init;		// 0 : first time init, not re-init, 1: in re-init procedure
	int ec_wakeup;		// 0 : ec shutdown when PAD in LP0, 1 : keep ec active when PAD in LP0,
	int ap_wake_wakeup;	// 0 : no ap_wake wakeup signal, 1: get ap_wake wakeup signal
};

#endif
