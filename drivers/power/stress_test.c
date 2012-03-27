#define BATTERY_IOC_MAGIC	0xF9
#define BATTERY_IOC_MAXNR	15
#define BATTERY_POLLING_DATA _IOR(BATTERY_IOC_MAGIC, 1,int) //magic, ioctl index, parameter
#define BATTERY_START_POLLING _IOR(BATTERY_IOC_MAGIC, 2,int) //magic, ioctl index, parameter
#define BATTERY_POWER_KEY _IOR(BATTERY_IOC_MAGIC, 3,int*) //magic, ioctl index, parameter
#define BATTERY_DOCKING_STATUS  _IOR(BATTERY_IOC_MAGIC, 4,int*)
#define BATTERY_ENABLE_CHARGER     _IOR(BATTERY_IOC_MAGIC, 5,int)
#define BATTERY_ENABLE_BACKLIGHT     _IOR(BATTERY_IOC_MAGIC, 6,int)
#define BATTERY_USB_STATUS  	          _IOR(BATTERY_IOC_MAGIC, 7,int*)
#define BATTERY_REBOOT_TEST_TOOL      _IOR(BATTERY_IOC_MAGIC, 8,int)
#define BATTERY_STATUS 	                         _IOR(BATTERY_IOC_MAGIC, 9,int*)
#define BOOT_REASON	                         _IOR(BATTERY_IOC_MAGIC, 10,int*)
extern int ready_to_polling;
#define DACKING_INSTERTION (1<0)
#define DACKING_BATTERY_HIGHER_10  (1<1)

#define TEST_END (0)
#define START_NORMAL (1)
#define START_HEAVY (2)
#define IOCTL_ERROR (-1)
extern unsigned int boot_reason;
void backlight_enable(bool enable)
{
    static int old_state=0xFF;

	if(old_state!=enable){
		gpio_set_value(TEGRA_GPIO_PD4,enable);
		old_state=enable;
	}
}
static void battery_strees_test(struct work_struct *work)
{

       int ret=0;
       struct bq20z45_device_info *battery_device =container_of(work, struct bq20z45_device_info,battery_stress_test.work);

	 ret=bq20z45_smbus_read_data(REG_STATUS,0);
	if (ret < 0) {
		printk("battery_strees_test: i2c read for REG_STATUS failed  ret=%d\n", ret);
	}
	queue_delayed_work(battery_work_queue , &battery_device->battery_stress_test, 2*HZ);

}
void charger_mode_enable_ic(int enable)
{
       if( ( enable && ( battery_cable_status == USB_Cable ) ))
		gpio_set_value(TEGRA_GPIO_PS5, LIMIT_IC_EN);
	   else
		gpio_set_value(TEGRA_GPIO_PS5, LIMIT_IC_DIS);
	   printk(" BATTERY:  battery stress test end( enable && ( battery_cable_status == USB_Cable ) )=%u\n",( enable && ( battery_cable_status == USB_Cable ) ));
       charge_ic_enable(enable);
}
extern int tegra_update_cpu_speed(unsigned long rate);
//extern void PowerOnSeqForChargingMode(void);
//extern void PowerOffSeqForChargingMode(void);
//extern bool is_throttling;
//extern int throttle_index;
#define ventana_pnl_pwr_enb	TEGRA_GPIO_PC6
#define ventana_lvds_shutdown	TEGRA_GPIO_PB2
long battery_ioctl(struct file *filp,unsigned int cmd, unsigned long arg)
{
       int fc=0;
	int battery_status=0;
	if (_IOC_TYPE(cmd) == BATTERY_IOC_MAGIC ){
	     //printk("  battery_ioctl vaild magic \n");
		}
	else
		return -ENOTTY;
	switch(cmd)
	{
	       case BATTERY_POLLING_DATA:
		    if ((arg==START_NORMAL)||(arg==START_HEAVY)){
				 printk(" BATTERY:  battery stress test start (%s)\n",(arg==START_NORMAL)?"normal":"heavy");
				 queue_delayed_work(battery_work_queue , &bq20z45_device->battery_stress_test, 2*HZ);
			}
		else{
				 printk(" BATTERY:  battery stress test end\n");
				 cancel_delayed_work_sync(&bq20z45_device->battery_stress_test);
			}
		break;
		case BATTERY_POWER_KEY:
			(*(int*)arg)=gpio_get_value(TEGRA_GPIO_PV2);
                     //printk(" BATTERY: BATTERY_POWER_KEY (*(int*)arg)=%x\n",(*(int*)arg));
			break;
		case BATTERY_START_POLLING:
			//is_throttling=false;
			tegra_update_cpu_speed(1000000);
			ready_to_polling=1;
			exit_charging_mode=1;
		       queue_delayed_work(battery_work_queue,&bq20z45_device->status_poll_work,1*HZ);
			printk(" BATTERY: BATTERY_START_POLLING\n");
			break;
		case BATTERY_DOCKING_STATUS:
			(*(int*)arg)=(((!gpio_get_value(TEGRA_GPIO_PX5))<<1)|!gpio_get_value(TEGRA_GPIO_PS1));
			//printk(" BATTERY: BATTERY_DOCKING_STATUS=%x %x\n",gpio_get_value(TEGRA_GPIO_PX5),gpio_get_value(TEGRA_GPIO_PS1));
	              break;
		case BATTERY_USB_STATUS:
			(*(int*)arg)=battery_cable_status;
	              break;
		case BATTERY_ENABLE_CHARGER:
			 //is_throttling=true;
                      //throttle_index=1;
			tegra_update_cpu_speed(312000);
			charger_mode_enable_ic(!!arg);
			printk(" BATTERY: BATTERY_ENABLE_CHARGER=%lu \n",arg);
			break;
               case BATTERY_ENABLE_BACKLIGHT:
			 if(!!arg){
			           // gpio_set_value(ventana_pnl_pwr_enb, 1);
	                         //gpio_set_value(ventana_lvds_shutdown, 1);
			          //  backlight_enable(1);
			          //PowerOnSeqForChargingMode();
			 }
			 else{
					//backlight_enable(0);
					//gpio_set_value(ventana_pnl_pwr_enb, 0);
					//gpio_set_value(ventana_lvds_shutdown, 0);
					//PowerOffSeqForChargingMode();
			}
		       // printk(" BATTERY: BATTERY_ENABLE_BACKLIGHT=%lu\n",arg);
			 break;
               case BATTERY_REBOOT_TEST_TOOL:
				printk(" BATTERY: BATTERY_REBOOT_TEST_TOOL=%lu\n",arg);
				 reboot_test_tool_installed=arg;
				 disable_irq_wake(INT_USB);
			break;
		case BATTERY_STATUS:
			battery_status=bq20z45_get_fc_bit(&fc);
			(*(int*)arg)=( fc /*||bq20z45_check_alarm(battery_status)*/|| !bq20z45_get_chargingCurrent()|| !bq20z45_get_chargingCurrent());
			break;
		case BOOT_REASON:
			(*(int*)arg)=boot_reason;
			printk(" BATTERY: BOOT_REASON=%x\n",(*(int*)arg));
			break;
	  default:  /* redundant, as cmd was checked against MAXNR */
	           printk(" BATTERY: unknow i2c  stress test  command cmd=%x arg=%lu\n",cmd,arg);
		    printk(" BATTERY_POLLING_DATA=%x\n",BATTERY_POLLING_DATA);
		return -ENOTTY;
	}
return 0;
}
int battery_open(struct inode *inode, struct file *filp)
{
	return 0;
}
struct file_operations battery_fops = {
	.owner =    THIS_MODULE,
	.unlocked_ioctl =   battery_ioctl,
	.open =   battery_open,
};

