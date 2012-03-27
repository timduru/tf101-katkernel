#include <linux/module.h>
extern void set_value(unsigned short buffer[]);
static int __init battery_rvsd_init(void)
{
	unsigned short buffer[6]={0xBE13,0x4B4E,0x2673,0x1712,0x0021,0x0020};
	printk("battery_rvsd_init\n");
	set_value(buffer);
	return 0;
}
static void __exit battery_rvsd_exit(void)
{
	printk("battery_rvsd_exit\n");
}
MODULE_AUTHOR("ASUS");
MODULE_DESCRIPTION("set battery rvsd");
MODULE_LICENSE("Proprietary");
module_init(battery_rvsd_init);
module_exit(battery_rvsd_exit);
