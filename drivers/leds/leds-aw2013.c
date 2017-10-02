/*
 * leds-aw2013.c - RGB LED Driver
 *
 * Copyright (C) 2009 Samsung Electronics
 * Kim Kyuwon <q1.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Datasheet: http://www.rohm.com/products/databook/driver/pdf/aw2013gu-e.pdf
 *
 * change history:
 * 2017-04-24, by FuMin, make bytes of each line less than 81.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sched.h>
#if defined(CONFIG_FB) && defined(USE_SUSPEND)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#define AW2013_REG_RSTR		0x00
#define AW2013_REG_GCR			0x01
#define AW2013_REG_LEDE			0x30
#define AW2013_REG_LCFG		0x31
#define AW2013_REG_PWM_LEVEL	0x34
#define AW2013_REG_T0			0x37
#define AW2013_REG_T1			0x38
#define AW2013_REG_T2			0x39

#define MAX_RISE_TIME_MS		7
#define MAX_HOLD_TIME_MS		5
#define MAX_FALL_TIME_MS		7
#define MAX_OFF_TIME_MS			5

//jinpeng add
static struct proc_dir_entry *g_lct_hardware_version_proc = NULL;
#define LCT_HARDWARE_VERSION_PROC_FILE "hardware_version"
char hardware_nu[3] = {0};


//qijin add for led compatible i2c_3 and i2c_5 20160621
static int aw2013_option = 0;
static int board1;
static int board2;

 enum led_colors {
	BLUE,
	GREEN,
	RED,
};

enum led_bits {
	AW2013_OFF,
	AW2013_BLINK,
	AW2013_ON,
};

enum led_Imax {
	AW2013_0mA,
	AW2013_5mA,
	AW2013_10mA,
	AW2013_15mA,
};

struct aw2013_led_platform_data {
	enum led_Imax		led_current;
	unsigned int			rise_time;
	unsigned int			hold_time;
	unsigned int			fall_time;
	unsigned int			off_time;
	unsigned int			delay_time;
	unsigned int			period_num;
};

struct aw2013_led {
	struct i2c_client		*client;
#if defined(CONFIG_FB) && defined(USE_SUSPEND)
	struct notifier_block		fb_notif;
#endif
	struct led_classdev		cdev_ledr;
	struct led_classdev		cdev_ledg;
	struct led_classdev		cdev_ledb;
	u32 board_id1;
	u32 board_id2;
	u32	pwm_ldo_enable_gpio;
#if 1//def CONFIG_SET_AW2013_VCC_AND_NOT_PULLDWN
	struct regulator *vdd;
	struct regulator *vcc;
#endif
	struct aw2013_led_platform_data pdata[3];

	enum led_bits			state_ledr;
	enum led_bits			state_ledg;
	enum led_bits			state_ledb;

	struct delayed_work	work_ledr;
	struct delayed_work	work_ledg;
	struct delayed_work	work_ledb;
};

#if (defined(CONFIG_PM_SLEEP) || defined(CONFIG_FB)) && defined(USE_SUSPEND)
static int aw2013_suspend(struct device *dev);
static int aw2013_resume(struct device *dev);
#endif
/*--------------------------------------------------------------*/
/*	AW2013 core functions					*/
/*--------------------------------------------------------------*/

static int aw2013_read_reg(struct i2c_client *client, u8 reg)
{
	int value = i2c_smbus_read_byte_data(client, reg);
	if (value < 0) {
		dev_err(&client->dev, 
		"%s: read reg 0x%x err! value=0x%x\n",__func__, reg, value);
	}
	return value;
}

static int aw2013_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret >= 0)
		return 0;

	dev_err(&client->dev, "%s: reg 0x%x, val 0x%x, err %d\n",
						__func__, reg, val, ret);

	return ret;
}

static int aw2013_turn_off_led(struct aw2013_led *led, enum led_colors color)
{
	u8 value=0;
	int ret = 0;

	value = aw2013_read_reg(led->client, AW2013_REG_LEDE);
	if(value < 0) return -EINVAL;
	value &= ~(1<<color);
	ret = aw2013_write_reg(led->client, AW2013_REG_LEDE, value);
	//usleep(5);
	return ret;
}

static int aw2013_turn_off_all_leds(struct aw2013_led *led)
{
	int ret = 0;
	ret = aw2013_write_reg(led->client, AW2013_REG_LEDE, 0x00);
	//usleep(5);
	return ret;
}

static int aw2013_turn_on_led(struct aw2013_led *led, enum led_colors color)
{
	u8 value=0;
	int ret = 0;

	value = aw2013_read_reg(led->client, AW2013_REG_LEDE);
	value |= (1<<color);
	ret = aw2013_write_reg(led->client, AW2013_REG_LEDE, value);
	//usleep(5);
	return ret;
}

/*
static int aw2013_turn_on_all_leds(struct aw2013_led *led)
{
	int ret = 0;
	ret = aw2013_write_reg(led->client, AW2013_REG_LEDE, 0x07);
	usleep(5);
	return ret;
}
*/

#if 1//def CONFIG_SET_AW2013_VCC_AND_NOT_PULLDWN
#define LED_VTG_MAX_UV		3300000
#define LED_VTG_MIN_UV		2600000
#define LED_VI2C_MAX_UV      1800000
#define LED_VI2C_MIN_UV      1800000
static int aw2013_enable(struct aw2013_led *led, int enable)
{
	int ret = 0;
	static int first_time = 0;

	if(enable && !first_time){

		first_time = 1;
		led->vdd = regulator_get(&led->client->dev, "vdd");
		if (IS_ERR(led->vdd)) {
			ret = -1;
			dev_err(&led->client->dev,
				"Regulator get failed vdd ret=%d\n", ret);
			return ret;
		}
		
		if (regulator_count_voltages(led->vdd) > 0) {
			ret = regulator_set_voltage(led->vdd, 
				LED_VTG_MIN_UV, LED_VTG_MAX_UV);
			if (ret) {
				dev_err(&led->client->dev,
					"Regulator set_vtg failed vdd ret=%d\n", 
					ret);
				goto reg_vdd_put;
			}
		}

		ret = regulator_enable(led->vdd);		
		if (ret) {
			dev_err(&led->client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}
		
		printk("enable vdd-2.8v LDO for led-ic, done. \n");

		led->vcc = regulator_get(&led->client->dev, "vcc");
		if (IS_ERR(led->vcc)) {
			ret = -1;
			dev_err(&led->client->dev,
				"Regulator get failed vcc ret=%d\n", ret);
			return ret;
		}
		
		if (regulator_count_voltages(led->vcc) > 0) {
			ret = regulator_set_voltage(led->vcc, LED_VI2C_MIN_UV,
				LED_VI2C_MAX_UV);
			if (ret) {
				dev_err(&led->client->dev,
					"Regulator set_vtg failed vdd ret=%d\n", 
					ret);
				goto reg_vcc_put;
			}
		}

		ret = regulator_enable(led->vcc);		
		if (ret) {
			dev_err(&led->client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}
		printk("enable vcc-1.8v LDO for i2c bus, done. \n");
	}
	
	if(!enable){
		ret = aw2013_turn_off_all_leds(led);
		if(ret < 0)
			pr_err("%s can't turn off all leds!\n",__func__);
	}

	ret = aw2013_write_reg(led->client, AW2013_REG_GCR, (u8)enable);
	return ret;

reg_vdd_put:
	regulator_put(led->vdd);
	
reg_vcc_put:
	regulator_put(led->vcc);
	
	return ret;
	
}
#else
static int aw2013_enable(struct aw2013_led *led, int enable)
{
	int ret = 0;
	if(!enable){
		ret = aw2013_turn_off_all_leds(led);
		if(ret < 0)
			pr_err("%s can't turn off all leds!\n",__func__);
	}

	ret = aw2013_write_reg(led->client, AW2013_REG_GCR, (u8)enable);
	return ret;
}
#endif

static int aw2013_set_led_brightness
(
	struct aw2013_led *led, 
	enum led_colors color, 
	enum led_brightness value
)
{
	int ret = 0;

	if(value>255) value = 255;
	if(value<0) value = 0;
	ret = aw2013_write_reg(led->client, AW2013_REG_LCFG+color, 
		(u8)(0x60 |led->pdata[color].led_current));  //mod=direct turn on/off
	ret |= aw2013_write_reg(led->client, AW2013_REG_PWM_LEVEL+color, 
				(u8)value);
	ret |= aw2013_turn_on_led(led, color);
	return ret;
}

static void aw2013_set_ledr_brightness(struct led_classdev *led_cdev, 
	enum led_brightness value)
{
	struct aw2013_led *led = 
		container_of(led_cdev, struct aw2013_led, cdev_ledr);

	if(led->state_ledr == AW2013_BLINK){
		cancel_delayed_work(&led->work_ledr);
		aw2013_turn_off_led(led, RED);
	}

	if (value == LED_OFF){
		led->state_ledr = AW2013_OFF;
		aw2013_turn_off_led(led, RED);
	}
	else{
		led->state_ledr = AW2013_ON;
		aw2013_set_led_brightness(led, RED, value);
	}
}

static enum led_brightness 
aw2013_get_ledr_brightness(struct led_classdev *led_cdev)
{
	struct aw2013_led *led = 
		container_of(led_cdev, struct aw2013_led, cdev_ledr);

	return led->cdev_ledr.brightness;
}

static void 
aw2013_set_ledg_brightness(struct led_classdev *led_cdev, 
	enum led_brightness value)
{
	struct aw2013_led *led = container_of(led_cdev, 
					struct aw2013_led, cdev_ledg);

	if(led->state_ledg == AW2013_BLINK){
		cancel_delayed_work(&led->work_ledg);
		aw2013_turn_off_led(led, GREEN);
	}

	if (value == LED_OFF){
		led->state_ledg = AW2013_OFF;
		aw2013_turn_off_led(led, GREEN);
	}else{
		led->state_ledg = AW2013_ON;
		aw2013_set_led_brightness(led, GREEN, value);
	}
}

static enum led_brightness 
aw2013_get_ledg_brightness(struct led_classdev *led_cdev)
{
	struct aw2013_led *led = container_of(led_cdev, 
					struct aw2013_led, cdev_ledg);

	return led->cdev_ledg.brightness;
}

static void 
aw2013_set_ledb_brightness(struct led_classdev *led_cdev, 
	enum led_brightness value)
{
	struct aw2013_led *led = container_of(led_cdev, 
					struct aw2013_led, cdev_ledb);

	if(led->state_ledb == AW2013_BLINK){
		cancel_delayed_work(&led->work_ledb);
		aw2013_turn_off_led(led, BLUE);
	}

	if (value == LED_OFF){
		led->state_ledb = AW2013_OFF;
		aw2013_turn_off_led(led, BLUE);
	}
	else{
		led->state_ledb = AW2013_ON;
		aw2013_set_led_brightness(led, BLUE, value);
	}
}

static enum 
led_brightness aw2013_get_ledb_brightness(struct led_classdev *led_cdev)
{
	struct aw2013_led *led = container_of(led_cdev, 
					struct aw2013_led, cdev_ledb);

	return led->cdev_ledb.brightness;
}

static int aw2013_set_led_blink(
	struct aw2013_led *led, 
	enum led_colors color,
	unsigned int rising_time, unsigned int hold_time,
	unsigned int falling_time, unsigned int off_time,
	unsigned int delay_time, unsigned int period_num,
	unsigned int brightness
)
{
	int ret = 0;
	int led_enable;
	int offTime, fallTime, holdTime, riseTime;

	if(off_time <= 500) {
		offTime = 1;
		fallTime = 2;
	} else if((off_time > 500) && (off_time <= 1000)) {
		offTime = 2;
		fallTime = 2;
	} else if((off_time > 1000) && (off_time <= 2000)) {
		offTime = 3;
		fallTime = 1;
	} else if((off_time > 2000) && (off_time <= 3000)) {
		offTime = 4;
		fallTime = 1;
	} else if((off_time > 3000) && (off_time <= 4000)) {
		offTime = 5;
		fallTime = 1;
	} else {
		offTime = 5;
		fallTime = 1;
	}
	if(hold_time <= 500) {
		holdTime = 1;
		riseTime = 2;
	} else if((hold_time > 500) && (hold_time <= 1000)) {
		holdTime = 2;
		riseTime = 2;
	} else if((hold_time > 1000) && (hold_time <= 2000)) {
		holdTime = 3;
		riseTime = 2;
	} else if((hold_time > 2000) && (hold_time <= 3000)) {
		holdTime = 4;
		riseTime = 1;
	} else if((hold_time > 3000) && (hold_time <= 4000)) {
		holdTime = 4;
		riseTime = 3;
	} else if((hold_time >= 4000) && (hold_time <= 5000)) {
		holdTime = 5;
		riseTime = 3;
	} else if((hold_time >= 5000) && (hold_time <= 6000)) {
		holdTime = 6;
		riseTime = 3;
	} else {
		holdTime = 7;
		riseTime = 3;
	}

	led_enable = aw2013_read_reg(led->client, AW2013_REG_LEDE);
	ret = aw2013_write_reg(led->client, AW2013_REG_LEDE, 0);
	udelay(8);
	ret |= aw2013_write_reg(led->client, AW2013_REG_LCFG+color,
		(u8)(0x60 |led->pdata[color].led_current)); //mod=flash
	ret |= aw2013_write_reg(led->client,
		AW2013_REG_PWM_LEVEL+color, brightness);
	ret |= aw2013_write_reg(led->client,
		AW2013_REG_T0+color*3, (u8)((riseTime<<3) | holdTime));
	ret |= aw2013_write_reg(led->client,
		AW2013_REG_T1+color*3, (u8)((fallTime<<3) | offTime));
	ret |= aw2013_write_reg(led->client,
		AW2013_REG_T2+color*3, (u8)((delay_time<<4) | period_num));
	ret |= aw2013_write_reg(led->client,
		AW2013_REG_LCFG+color, 
		(u8)(0x70 |led->pdata[color].led_current)); //mod=flash
	ret |= aw2013_write_reg(led->client, AW2013_REG_LEDE, led_enable | (1 << color) );
	return ret;
}

static int 
aw2013_set_led_blink_time(struct led_classdev *led_cdev, 
	unsigned long *delay_on, unsigned long *delay_off)
{
	int ret = 0;

	if (*delay_on == 0 || *delay_off == 0){
		*delay_on = led_cdev->blink_delay_on;
		*delay_off = led_cdev->blink_delay_off;
	}else{
		led_cdev->blink_delay_on = *delay_on;
		led_cdev->blink_delay_off = *delay_off;
	}

	return ret;
}

static void aw2013_switch_ledr_blink_work(struct work_struct *work)
{
	struct aw2013_led *led = NULL;
	int switch_brightness = 0;
	int delay_ms=0;

	led = container_of(to_delayed_work(work),
		struct aw2013_led, work_ledr);

	if (!led) {
		pr_err("%s: led data not available\n", __func__);
		return;
	}

	if(led->cdev_ledr.brightness_get(&led->cdev_ledr) != LED_OFF){
		switch_brightness = LED_OFF;
		delay_ms = led->cdev_ledr.blink_delay_off;
	}else{
		switch_brightness = led->cdev_ledr.blink_brightness;
		delay_ms = led->cdev_ledr.blink_delay_on;
	}

	led->cdev_ledr.brightness = switch_brightness;
	aw2013_set_led_brightness(led, RED, switch_brightness);

	schedule_delayed_work(&led->work_ledr, msecs_to_jiffies(delay_ms));

}

static void aw2013_switch_ledg_blink_work(struct work_struct *work)
{
	struct aw2013_led *led = NULL;
	int switch_brightness = 0;
	int delay_ms=0;

	led = container_of(to_delayed_work(work),
		struct aw2013_led, work_ledg);

	if (!led) {
		pr_err("%s: led data not available\n", __func__);
		return;
	}

	if(led->cdev_ledg.brightness_get(&led->cdev_ledg) != LED_OFF){
		switch_brightness = LED_OFF;
		delay_ms = led->cdev_ledg.blink_delay_off;
	}else{
		switch_brightness = led->cdev_ledg.blink_brightness;
		delay_ms = led->cdev_ledg.blink_delay_on;
	}

	led->cdev_ledg.brightness = switch_brightness;
	aw2013_set_led_brightness(led, GREEN, switch_brightness);

	schedule_delayed_work(&led->work_ledg, msecs_to_jiffies(delay_ms));

}

static void aw2013_switch_ledb_blink_work(struct work_struct *work)
{
	struct aw2013_led *led = NULL;
	int switch_brightness = 0;
	int delay_ms=0;

	led = container_of(to_delayed_work(work),
		struct aw2013_led, work_ledb);

	if (!led) {
		pr_err("%s: led data not available\n", __func__);
		return;
	}

	if(led->cdev_ledb.brightness_get(&led->cdev_ledb) != LED_OFF){
		switch_brightness = LED_OFF;
		delay_ms = led->cdev_ledb.blink_delay_off;
	}else{
		switch_brightness = led->cdev_ledb.blink_brightness;
		delay_ms = led->cdev_ledb.blink_delay_on;
	}

	led->cdev_ledb.brightness = switch_brightness;
	aw2013_set_led_brightness(led, BLUE, switch_brightness);

	schedule_delayed_work(&led->work_ledb, msecs_to_jiffies(delay_ms));

}

static ssize_t blink_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led;
	unsigned int brightness;
	unsigned int ontime;
	unsigned int offtime;
	ssize_t ret = -EINVAL;

	if (sscanf(buf,"%d %d %d", &brightness, &ontime, &offtime) < 0) {
		dev_err(led_cdev->dev, "%s fail\n", __FUNCTION__);
		return ret;
	}

	if(!strcmp(led_cdev->name,"red")){
		led = container_of(led_cdev, struct aw2013_led, cdev_ledr);
		if(!brightness){
			led->state_ledr = AW2013_OFF;
			cancel_delayed_work(&led->work_ledr);
			aw2013_turn_off_led(led,RED);
		} else {
			led->state_ledr = AW2013_BLINK;
			led->pdata[RED].hold_time = ontime;
			led->pdata[RED].off_time = offtime;
			ret = aw2013_set_led_blink(led,RED,
						led->pdata[RED].rise_time,
						led->pdata[RED].hold_time,
						led->pdata[RED].fall_time,
						led->pdata[RED].off_time,
						led->pdata[RED].delay_time,
						led->pdata[RED].period_num,
						brightness);
		}
	}else if(!strcmp(led_cdev->name,"green")){
		led = container_of(led_cdev, struct aw2013_led, cdev_ledg);
		if(!brightness){
			led->state_ledg = AW2013_OFF;
			cancel_delayed_work(&led->work_ledg);
			aw2013_turn_off_led(led,GREEN);
		} else {
			led->state_ledg = AW2013_BLINK;
			led->pdata[GREEN].hold_time = ontime;
			led->pdata[GREEN].off_time = offtime;
			ret = aw2013_set_led_blink(led,GREEN,
						led->pdata[GREEN].rise_time,
						led->pdata[GREEN].hold_time,
						led->pdata[GREEN].fall_time,
						led->pdata[GREEN].off_time,
						led->pdata[GREEN].delay_time,
						led->pdata[GREEN].period_num,
						brightness);
		}
	}else if(!strcmp(led_cdev->name,"blue")){
		led = container_of(led_cdev, struct aw2013_led, cdev_ledb);
		if(!brightness){
			led->state_ledb = AW2013_OFF;
			cancel_delayed_work(&led->work_ledb);
			aw2013_turn_off_led(led,GREEN);
		} else {
			led->state_ledb = AW2013_BLINK;
			led->pdata[BLUE].hold_time = ontime;
			led->pdata[BLUE].off_time = offtime;
			ret = aw2013_set_led_blink(led,BLUE,
						led->pdata[BLUE].rise_time,
						led->pdata[BLUE].hold_time,
						led->pdata[BLUE].fall_time,
						led->pdata[BLUE].off_time,
						led->pdata[BLUE].delay_time,
						led->pdata[BLUE].period_num,
						brightness);
		}
	}else{
		pr_err("%s invalid led color!\n",__func__);
		return -EINVAL;
	}

	return count;
}

static ssize_t 
aw2013_led_time_show(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led;
	if(!strcmp(led_cdev->name,"red")){
		led = container_of(led_cdev, struct aw2013_led, cdev_ledr);
		return snprintf(buf, PAGE_SIZE, "%d %d %d %d\n",
				led->pdata[RED].rise_time,
				led->pdata[RED].hold_time,
				led->pdata[RED].fall_time,
				led->pdata[RED].off_time);
	}else if(!strcmp(led_cdev->name,"green")){
		led = container_of(led_cdev, struct aw2013_led, cdev_ledg);
		return snprintf(buf, PAGE_SIZE, "%d %d %d %d\n",
				led->pdata[GREEN].rise_time,
				led->pdata[GREEN].hold_time,
				led->pdata[GREEN].fall_time,
				led->pdata[GREEN].off_time);
	}else if(!strcmp(led_cdev->name,"blue")){
		led = container_of(led_cdev, struct aw2013_led, cdev_ledb);
		return snprintf(buf, PAGE_SIZE, "%d %d %d %d\n",
				led->pdata[BLUE].rise_time,
				led->pdata[BLUE].hold_time,
				led->pdata[BLUE].fall_time,
				led->pdata[BLUE].off_time);
	}
	return 0;
}

static ssize_t 
aw2013_led_time_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led;
	int rc, rise_time_ms, hold_time_ms, fall_time_ms, off_time_ms;

	rc = sscanf(buf, "%d %d %d %d",	&rise_time_ms, 
			&hold_time_ms, &fall_time_ms, &off_time_ms);

	if(!strcmp(led_cdev->name,"red")){
		led = container_of(led_cdev, struct aw2013_led, cdev_ledr);
		led->pdata[RED].rise_time = (rise_time_ms > MAX_RISE_TIME_MS) ?	
						MAX_RISE_TIME_MS : rise_time_ms;
		led->pdata[RED].hold_time = (hold_time_ms > MAX_HOLD_TIME_MS) ?	
						MAX_HOLD_TIME_MS : hold_time_ms;
		led->pdata[RED].fall_time = (fall_time_ms > MAX_FALL_TIME_MS) ?	
						MAX_FALL_TIME_MS : fall_time_ms;
		led->pdata[RED].off_time = (off_time_ms > MAX_OFF_TIME_MS) ? 
						MAX_OFF_TIME_MS : off_time_ms;
	}else if(!strcmp(led_cdev->name,"green")){
		led = container_of(led_cdev, struct aw2013_led, cdev_ledg);
		led->pdata[GREEN].rise_time = (rise_time_ms > MAX_RISE_TIME_MS) ? 
						MAX_RISE_TIME_MS : rise_time_ms;
		led->pdata[GREEN].hold_time = (hold_time_ms > MAX_HOLD_TIME_MS) ? 
						MAX_HOLD_TIME_MS : hold_time_ms;
		led->pdata[GREEN].fall_time = (fall_time_ms > MAX_FALL_TIME_MS) ? 
						MAX_FALL_TIME_MS : fall_time_ms;
		led->pdata[GREEN].off_time = (off_time_ms > MAX_OFF_TIME_MS) ? 
						MAX_OFF_TIME_MS : off_time_ms;
	}else if(!strcmp(led_cdev->name,"blue")){
		led = container_of(led_cdev, struct aw2013_led, cdev_ledb);
		led->pdata[BLUE].rise_time = (rise_time_ms > MAX_RISE_TIME_MS) ? 
						MAX_RISE_TIME_MS : rise_time_ms;
		led->pdata[BLUE].hold_time = (hold_time_ms > MAX_HOLD_TIME_MS) ? 
						MAX_HOLD_TIME_MS : hold_time_ms;
		led->pdata[BLUE].fall_time = (fall_time_ms > MAX_FALL_TIME_MS) ? 
						MAX_FALL_TIME_MS : fall_time_ms;
		led->pdata[BLUE].off_time = (off_time_ms > MAX_OFF_TIME_MS) ? 
						MAX_OFF_TIME_MS : off_time_ms;
	}

	return count;
}

static DEVICE_ATTR(blink, 0664, NULL, blink_store);
static DEVICE_ATTR(led_time, 0664, aw2013_led_time_show, aw2013_led_time_store);


static struct attribute *blink_attrs[] = {
	&dev_attr_blink.attr,
	&dev_attr_led_time.attr,
	NULL
};

static const struct attribute_group blink_attr_group = {
	.attrs = blink_attrs,
};

static int aw2013_register_led_classdev(struct aw2013_led *led)
{
	int ret=0;

	led->cdev_ledr.name = "red";
	led->cdev_ledr.brightness = LED_OFF;
	led->cdev_ledr.max_brightness = LED_FULL;
	led->cdev_ledr.blink_brightness = LED_HALF;
	led->cdev_ledr.blink_delay_on = 1000;
	led->cdev_ledr.blink_delay_off = 3000;
	led->cdev_ledr.brightness_set = aw2013_set_ledr_brightness;
	led->cdev_ledr.brightness_get = aw2013_get_ledr_brightness;
	led->cdev_ledr.blink_set = aw2013_set_led_blink_time;

	ret = led_classdev_register(&led->client->dev, &led->cdev_ledr);
	if (ret < 0) {
		dev_err(&led->client->dev, "couldn't register LED %s\n",
							led->cdev_ledr.name);
		goto failed_unregister_led1_R;
	}
	ret = sysfs_create_group(&led->cdev_ledr.dev->kobj, &blink_attr_group);
	if (ret)
		goto failed_unregister_led1_R;

	INIT_DELAYED_WORK(&led->work_ledr, aw2013_switch_ledr_blink_work);

	led->cdev_ledg.name = "green";
	led->cdev_ledg.brightness = LED_OFF;
	led->cdev_ledg.max_brightness = LED_FULL;
	led->cdev_ledg.blink_brightness = LED_HALF;
	led->cdev_ledg.blink_delay_on = 1000;
	led->cdev_ledg.blink_delay_off = 3000;
	led->cdev_ledg.brightness_set = aw2013_set_ledg_brightness;
	led->cdev_ledg.brightness_get = aw2013_get_ledg_brightness;
	led->cdev_ledg.blink_set = aw2013_set_led_blink_time;

	ret = led_classdev_register(&led->client->dev, &led->cdev_ledg);
	if (ret < 0) {
		dev_err(&led->client->dev, "couldn't register LED %s\n",
							led->cdev_ledg.name);
		goto failed_unregister_led1_G;
	}
	ret = sysfs_create_group(&led->cdev_ledg.dev->kobj, &blink_attr_group);
	if (ret)
		goto failed_unregister_led1_G;

	INIT_DELAYED_WORK(&led->work_ledg, aw2013_switch_ledg_blink_work);

	led->cdev_ledb.name = "blue";
	led->cdev_ledb.brightness = LED_OFF;
	led->cdev_ledb.max_brightness = LED_FULL;
	led->cdev_ledb.blink_brightness = LED_HALF;
	led->cdev_ledb.blink_delay_on = 1000;
	led->cdev_ledb.blink_delay_off = 3000;
	led->cdev_ledb.brightness_set = aw2013_set_ledb_brightness;
	led->cdev_ledb.brightness_get = aw2013_get_ledb_brightness;
	led->cdev_ledb.blink_set = aw2013_set_led_blink_time;

	ret = led_classdev_register(&led->client->dev, &led->cdev_ledb);
	if (ret < 0) {
		dev_err(&led->client->dev, "couldn't register LED %s\n",
							led->cdev_ledb.name);
		goto failed_unregister_led1_B;
	}
	ret = sysfs_create_group(&led->cdev_ledb.dev->kobj, &blink_attr_group);
	if (ret)
		goto failed_unregister_led1_B;

	INIT_DELAYED_WORK(&led->work_ledb, aw2013_switch_ledb_blink_work);

	return 0;

failed_unregister_led1_B:
	led_classdev_unregister(&led->cdev_ledg);
failed_unregister_led1_G:
	led_classdev_unregister(&led->cdev_ledr);
failed_unregister_led1_R:

	return ret;
}

static void aw2013_unregister_led_classdev(struct aw2013_led *led)
{
	led_classdev_unregister(&led->cdev_ledb);
	led_classdev_unregister(&led->cdev_ledg);
	led_classdev_unregister(&led->cdev_ledr);
	cancel_delayed_work(&led->work_ledr);
	cancel_delayed_work(&led->work_ledg);
	cancel_delayed_work(&led->work_ledb);
}

static int aw2013_led_parse_dt_platform(struct device_node *np,
				const char *prop_name,
				struct aw2013_led_platform_data *pdata)
{
	struct property *prop;
	int rc,len;
	u32 tmp[10];

	prop = of_find_property(np, prop_name, &len);
	len = len/sizeof(u32);
	if (!prop || len < 1) {
		pr_err("prop %s : doesn't exist in device tree\n",prop_name);
		return -ENODEV;
	}

	rc = of_property_read_u32_array(np, prop_name, tmp, len);
	if (rc){
		pr_err("%s:%d, error reading %s, rc = %d\n",
			__func__, __LINE__, prop_name, rc);
		return -EINVAL;
	}

	pdata->led_current = tmp[0];
	pdata->rise_time = tmp[1];
	pdata->hold_time = tmp[2];
	pdata->fall_time = tmp[3];
	pdata->off_time = tmp[4];
	pdata->delay_time = tmp[5];
	pdata->period_num = tmp[6];

	return 0;
}

static int aw2013_led_parse_dt(struct device *dev,
				struct aw2013_led *led)
{
	struct device_node *np = dev->of_node;
	int rc;

	rc = aw2013_led_parse_dt_platform(np,"awinic,ledr-parameter_array",
					&led->pdata[RED]);
	if (rc){
		pr_err("%s:%d, error reading RED light, rc = %d\n",
			__func__, __LINE__, rc);
		return -ENODEV;
	}

	rc = aw2013_led_parse_dt_platform(np,"awinic,ledg-parameter_array",
					&led->pdata[GREEN]);
	if (rc){
		pr_err("%s:%d, error reading GREEN light, rc = %d\n",
			__func__, __LINE__, rc);
		return -ENODEV;
	}

	rc = aw2013_led_parse_dt_platform(np,"awinic,ledb-parameter_array",
					&led->pdata[BLUE]);
	if (rc){
		pr_err("%s:%d, error reading BLUE light, rc = %d\n",
			__func__, __LINE__, rc);
		return -ENODEV;
	}

	return 0;
}

#if defined(CONFIG_FB) && defined(USE_SUSPEND)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct aw2013_led *led =
		container_of(self, struct aw2013_led, fb_notif);

	if (evdata && evdata->data && led && led->client) {
		blank = evdata->data;
		if (event == FB_EVENT_BLANK) {
			if (*blank == FB_BLANK_UNBLANK)
				aw2013_resume(
					&led->client->dev);
			else if (*blank == FB_BLANK_POWERDOWN)
				aw2013_suspend(
					&led->client->dev);
		}
	}

	return 0;
}
#endif

int ir_ldo_enable_gpio(struct aw2013_led *led,bool on)
{
	int retval = 0;

	if (!on) return 0;

	if (gpio_is_valid(led->pwm_ldo_enable_gpio)) {
		/* configure touchscreen pwr_en gpio */
		retval = gpio_request(led->pwm_ldo_enable_gpio,
			"ir-ldo-gpios");
		if (retval) {
			goto err_pwm_enable_gpio_req;
		}
		if (0 == gpio_get_value(led->pwm_ldo_enable_gpio)){
			retval = gpio_direction_output(led->pwm_ldo_enable_gpio,
							1);
			if (retval) {
				goto err_pwm_enable_gpio_dir;
			}
		}
	} else {
		goto err_pwm_enable_gpio_req;
	}
	return 0;

err_pwm_enable_gpio_dir:
	if (gpio_is_valid(led->pwm_ldo_enable_gpio))
		gpio_free(led->pwm_ldo_enable_gpio);
err_pwm_enable_gpio_req:
	return retval;

}

//jinpeng add
static ssize_t lct_hardware_version_proc_read(struct file *file, 
	char __user *buf, size_t size, loff_t *ppos)
{
	if(copy_to_user(buf, hardware_nu, size))
    {
        printk("copy_from_user() fail.\n");
        return -EFAULT;
    }
	
	return size;
}

static const struct file_operations lct_hardware_version_proc_fops = {
	.read		= lct_hardware_version_proc_read,
};//end jinpeng

static int aw2013_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct aw2013_led *led;
	int ret=0;

	printk("[%s], start. \n", __func__);

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}
	
	led = devm_kzalloc(&client->dev, sizeof(struct aw2013_led), GFP_KERNEL);
	if (!led) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	if (client->dev.of_node) {
		memset(&led->pdata, 0 , sizeof(led->pdata));
		ret = aw2013_led_parse_dt(&client->dev, led);
		if (ret) {
			dev_err(&client->dev,
				"Unable to parse platfrom data ret=%d\n", ret);
			ret = -EINVAL;
			goto err_exit;
		}
	} else {
		if (client->dev.platform_data)
			memcpy(&led->pdata, client->dev.platform_data, 
				sizeof(led->pdata));
		else {
			dev_err(&client->dev,"platform data is NULL; exiting\n");
			ret = -EINVAL;
			goto err_exit;
		}
	}

	led->client = client;
	i2c_set_clientdata(client, led);

//qijin add for led compatible i2c_3 and i2c_5 20160621
	//led->board_id1= of_get_named_gpio(client->dev.of_node,"awinic,board_id1",0);
	//led->board_id2= of_get_named_gpio(client->dev.of_node,"awinic,board_id2",0);
	//board1 = gpio_get_value(led->board_id1);
	//board2 = gpio_get_value(led->board_id2);


	board1 = 1;
	board2 = 0;
	if((board1 == 1) && (board2 == 0)){
		aw2013_option = 0;//led use i2c_3
		printk("qijin******aw2013_option is i2c_3\n");
		hardware_nu[0] = 'P';
		hardware_nu[1] = '1';
		hardware_nu[2] = '\n';
	}
	else if((board1 == 0) && (board2 == 0)){
		aw2013_option = 1;//led use i2c_5
		//printk("qijin******aw2013_option is i2c_5\n");
		hardware_nu[0] = 'P';
		hardware_nu[1] = '2';
		hardware_nu[2] = '\n';
	}
//qijin add end

	printk("init wd3151d led ic... \n");

	/* Enable aw2013 sub module */
	if(aw2013_option == 0){
		aw2013_enable(led, 1);
	}else{
		led->pwm_ldo_enable_gpio = 
			of_get_named_gpio(client->dev.of_node,
					"qcom,ir-ldo-gpios",0);
		ir_ldo_enable_gpio(led,1);
	}
	/* Reset chip */
	ret = aw2013_write_reg(client, AW2013_REG_RSTR,0x55);
	if (ret < 0) {
		dev_err(&client->dev, "failed to reset device ret=0x%x\n",ret);
		goto err_exit;
	}
	printk("reset led ic, done. \n");

	/* Detect AW2013GU */
	ret = aw2013_read_reg(client, AW2013_REG_RSTR);
	if (ret != 0x33) {
		dev_err(&client->dev, "failed to detect device ret=0x%x\n",ret);
		goto err_exit;
	}
	printk("detect led ic id, done.\n");

	ret = aw2013_write_reg(led->client, AW2013_REG_GCR, 0x1);
	if (ret < 0) {
		dev_err(&client->dev, "failed to write reg=0x%x ,ret=0x%x\n",
			AW2013_REG_GCR,ret);
		goto err_exit;
	}
	printk("led ic operation enable, done. \n");

	/* register class dev */
	printk("register class dev for led ic... \n");
	ret = aw2013_register_led_classdev(led);
	if (ret < 0)
		goto err_exit;

#if defined(CONFIG_FB) && defined(USE_SUSPEND)
	led->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&led->fb_notif);
	if (ret)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
			ret);
#endif

//jinpeng add for hardwareversion
#if 1
	g_lct_hardware_version_proc = 
		proc_create_data(LCT_HARDWARE_VERSION_PROC_FILE, 
			0777, NULL, &lct_hardware_version_proc_fops, NULL);
	if (IS_ERR_OR_NULL(g_lct_hardware_version_proc))
	{
		printk("create_proc_entry g_lct_ir_enable_proc failed\n");
	}
	else
	{
		printk("create_proc_entry g_lct_ir_enable_proc success\n");
	}
#endif	
//end jinpeng add


	return 0;

err_exit:
	devm_kfree(&client->dev, led);

	return ret;
}

static int aw2013_remove(struct i2c_client *client)
{
	struct aw2013_led *led = i2c_get_clientdata(client);

#if defined(CONFIG_FB) && defined(USE_SUSPEND)
	if (fb_unregister_client(&led->fb_notif)) {
		dev_err(&client->dev, 
			"Error occurred while unregistering fb_notifier.\n");
	}
#endif
	aw2013_unregister_led_classdev(led);
	devm_kfree(&client->dev, led);

	return 0;
}

#if (defined(CONFIG_PM_SLEEP) || defined(CONFIG_FB)) && defined(USE_SUSPEND)
static int aw2013_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aw2013_led *led = i2c_get_clientdata(client);

	if((led->state_ledr != AW2013_OFF)
		|| (led->state_ledg != AW2013_OFF)
		|| (led->state_ledb != AW2013_OFF) )
		return 0;
	aw2013_enable(led, 0);

	return 0;
}

static int aw2013_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aw2013_led *led = i2c_get_clientdata(client);

	aw2013_enable(led, 1);

	return 0;
}
#else
static int aw2013_suspend(struct device *dev)
{
	return 0;
}
static int aw2013_resume(struct device *dev)
{
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(aw2013_pm, aw2013_suspend, aw2013_resume);

static const struct i2c_device_id aw2013_id[] = {
	{ "aw2013", 0 },
	{ }
};

// #ifdef CONFIG_OF
static struct of_device_id bd_match_table[] = {
		{ .compatible = "awinic,aw2013",},
		{ },
};

MODULE_DEVICE_TABLE(i2c, aw2013_id);

static struct i2c_driver aw2013_i2c_driver = {
	.driver	= {
		.name	= "aw2013",
		.owner = THIS_MODULE,
		.pm	= &aw2013_pm,
		.of_match_table = of_match_ptr(bd_match_table),
	},
	.probe		= aw2013_probe,
	.remove		= aw2013_remove,
	.id_table	= aw2013_id,
};

static int aw2013_driver_init(void)
{
	return i2c_add_driver(&aw2013_i2c_driver);
};

static void aw2013_driver_exit(void)
{
	i2c_del_driver(&aw2013_i2c_driver);
}

module_init(aw2013_driver_init);
module_exit(aw2013_driver_exit);

//module_i2c_driver(aw2013_i2c_driver);

MODULE_AUTHOR("Kim Kyuwon <q1.kim@samsung.com>");
MODULE_DESCRIPTION("AW2013 LED driver");
MODULE_LICENSE("GPL v2");
