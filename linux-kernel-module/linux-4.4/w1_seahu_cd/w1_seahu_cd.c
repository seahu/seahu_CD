/*
 * w1_seahu_cd.c - w1 family cc (SEAHU_CC - Universal multiple sensors and output devices) driver
 * based on w1_ds2413.c by Mariusz Bialonczyk <manio@skyboo.net>
 *
 * Copyright (c) 2016 Ondrej Lycka <ondra@spsstavbrno.cz>
 *
 * This source code is licensed under the GNU General Public License,
 * Version 2. See the file COPYING for more details.
 */


/*
 My own notes:
	chnages files:
		w1_nover_df.v
		Makefile
		Kcofig
*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/crc16.h>


#include "../w1.h"
#include "../w1_int.h"
#include "../w1_family.h"


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ondrej Lycka <info@seahu.cz>");
MODULE_DESCRIPTION("w1 family cc driver for opensource communication standard for slave 1-wire devices (etc. measurement temperature, humidity, light, AD, RGB ouput, DA, PIO, PWM outpit,...");
MODULE_ALIAS("w1-family-CC");

#define W1_SLAVE_DATA_SIZE						1
#define W1_SLAVE_DATA_SIZE_BYTE					4
#define W1_SLAVE_DATA_SIZE_VALUE				4
#define W1_SLAVE_DATA_SIZE_DEVICE_DESCRIPTION	64
#define W1_SLAVE_DATA_SIZE_DESCRIPTION			64
#define W1_SLAVE_DATA_SIZE_USER_NOTE			32
#define W1_FAMILY_SEAHU_CD  					0xcd
#define W1_FCC_RETRIES                     		3
#define NODELAY									0
//#define NOLOCK									0
//#define LOCK									1
#define ERR_UNLOCK								2
#define SKIP_ROM								0
#define SELECT_ROM								1
#define RESUME_ROM								2
#define TRUE									1
#define FALSE									0
#define ERROR									1
#define NOERROR									0
#define MAX_MEASURE_TIME						2000	// in ms

/* OW COMMANDS CODE */
#define _1W_FCC_START_MEASURE			0x44 // measure command (measure selected section)
#define _1W_FCC_READ_DEVICE_DESCRIPTION	0xF1 // read factory description device (64Bytes)
#define _1W_FCC_READ_NUMBER_SECTIONS	0xF2 // read number of sections (8bit)
#define _1W_FCC_READ_CONTROL_BYTE		0xF3 // read from CONTROL register (8bit) - control register selected section
#define _1W_FCC_READ_ACTUAL_VALUE		0xF5 // read from ACTUAL_VALUE register (1-4Byte depends on setting in control register)
#define _1W_FCC_READ_DESCRIPTION		0xF7 // read from DESCRIPTION register (32Byte)
#define _1W_FCC_READ_USER_NOTE			0xF8 // read from NOTE register (64Bytes)
#define _1W_FCC_READ_MIN_ALARM_VALUE	0xFA // read from MIN_ALARM_VALUE register (1-4Bytes depends on setting in control register)
#define _1W_FCC_READ_MAX_ALARM_VALUE	0xFB // read from MAX_ALARM_VALUE register (1-4Bytes depends on setting in control register)
#define _1W_FCC_WRITE_CONTROL_BYTE		0x53 // write into CONTROL register (8bit) - control register selected section
#define _1W_FCC_WRITE_ACTUAL_VALUE		0x55 // write into ACTUAL_VALUE (1-4Bytes depends on setting in control register)
#define _1W_FCC_WRITE_MIN_ALARM_VALUE	0x5A // write into MIN_ALARM_VALUE (1-4 Byte depends on setting in control register)
#define _1W_FCC_WRITE_MAX_ALARM_VALUE	0x5B // write into MAX_ALARM_VALUE (1-4 Byte depends on setting in control register)
#define _1W_FCC_WRITE_USER_NOTE			0x58 // write into NOTE register (64Bytes)

/* STATUS BYTE MASKS */
#define _MASK_ENABLE_READ		0b00000001
#define _MASK_ENABLE_WRITE		0b00000010
#define _MASK_GIVE_ALARM		0b00000100
#define _MASK_TYPE_VALUE		0b00011000
#define _MASK_STATUS_ALARM		0b00100000
#define _MASK_MIN_ALARM			0b01000000
#define _MASK_MAX_ALARM			0b10000000

/* OW DELAY */
#define DEVICE_DESCRIPTION_DELAY	100 // delay for slave to read device description from PROM to RAM
#define READ_MIN_MAX_ALARM_DELAY	50  // delay for slave to red min or max alarm value from EEPROM to RAM
#define DESCRIPTION_DELAY			100 // delay for slave to read section description from PROM to RAM
#define READ_USER_NOTE_DEALY		100 // delay for slave to read section user note from EEROM to RAM
#define WRITE_MIN_ALARM_DEALY		100 // delay for slave to store new min or max section value to EEPROM
#define WRITE_USER_NOTE_DELAY		100 // delay for slave to store new section user note to EEPROM


/* union representing more types of actual value */
typedef union {
    u8 		         	Bool;
    u_int32_t      		uInt;
    int32_t				Int;
    u8					Buf[4];
  } val;


 /* this driver create dynamically entries in SYSFS, next structures help more organize dynamically allocated data */

 /*
  * One slave device may integrate more devices as thermometer, barometer, GPIO, .. . Every this device have own section
  * This section is in SYSFS representing by own directory.
  * This structure contain space for one section. I.e. attributes for SYSFS array with pointer into this attributes and
  *  sysfs group (directory) link into attribute array
  */
struct section
{
	char name[11];
	u8	section_number;
	struct attribute_group sysfs_group_attr;
	struct bin_attribute *bin_attrs[14];
	struct bin_attribute control_byte;
	struct bin_attribute enable_read;
	struct bin_attribute enable_write;
	struct bin_attribute give_alarm;
	struct bin_attribute status_alarm;
	struct bin_attribute value_min_alarm;
	struct bin_attribute value_max_alarm;
	struct bin_attribute measure;
	struct bin_attribute actual_value;
	struct bin_attribute min_alarm_value;
	struct bin_attribute max_alarm_value;
	struct bin_attribute description;
	struct bin_attribute user_note;
};

/*
 * This structure representing specific slave device data. 
 * Contain pointer into start of array with sections, pointer into array with pointers into sysfs groups and number of section.
 * Memory for this entries must be allocated during process who add new slave device (i.e. in function 'seahu_cc_add_slave_v_dynamic')
 * Store poiters on allocated memory is important for will free memory on exit module and
 * number of section is often used constat, stored here reduce 1-wire comunication
 */
struct slave
{
	struct section *p_sections;
	const struct attribute_group **p_to_p_groups;
	u8 number_sections;
};


/* ---------------------------- W1 FUNCTION ------------------------------------------*/
/* do lock w1 bus */
/**
 * do_lock() - do lock w1 bus
 *
 * @ls:		- pointer to slave device
 * Return:	- void
 */
static void do_lock(struct w1_slave *sl){
	//mutex_lock(&sl->master->bus_mutex); //for newest kernel
	dev_dbg(&sl->dev, "do mutex locked");
	mutex_lock(&sl->master->mutex);
	dev_dbg(&sl->dev, "mutex locked");
}

/**
 * do_unlock() - do unlock w1 bus
 *
 * @ls:		- pointer to slave device
 * Return:	- void
 */
static void do_unlock(struct w1_slave *sl){
	dev_dbg(&sl->dev, "do mutex unlocked");
	mutex_unlock(&sl->master->mutex);
	dev_dbg(&sl->dev, "mutex unlocked");
}

/**
 * delay() - delay interruptible
 *
 * @Delay:		- delay in us
 * Return:		- void
 */
void delay(int Delay)
{
	unsigned long msecs;
	if (Delay!=0){
		//msecs=(Delay)/1000;
		msecs=(unsigned long)Delay;
		if ( msecs==0 ) msecs=1;
		msleep_interruptible(msecs);
	}
}

/**
 * w1_seahu_select() - select slave device on w1 bus
 *
 * @ls:			- pointer to slave device
 * @w1_select:	- way howto select slave  (SKIP_ROM, SELECT_ROM, RESUME_ROM)
 * 						SKIP_ROM - skip selection, in case the device has already been selected before
 * 						SELECT_ROM - w1 reset and w1 select device by ROM code
 * 						RESUME_ROM - w1 reset and resume device who has already been selected before
 * Return:		- ERROR or NOERROR
 */
static int  w1_seahu_select(struct w1_slave *sl, u8 w1_select){
	switch (w1_select){
		case SKIP_ROM:
			return NOERROR;
		case SELECT_ROM:
			if (w1_reset_select_slave(sl)){
				dev_dbg(&sl->dev, "Failed select device");
				return ERROR;
			}
			return NOERROR;
		case RESUME_ROM:
			if (w1_reset_resume_command(sl->master)){
				dev_dbg(&sl->dev, "Failed resume device");
				return ERROR;
			}
			return NOERROR;
	}
	return ERROR;
}


/**
 * w1_seahu_read_from_device() - read X bytes from slave device with CRC16 check
 *
 * @ls:			- pointer to slave device
 * @cmd:		- command code
 * @data:		- pointer to data
 * @len:		- len of data
 * @Delay:		- delay in [us] to slave prepare data
 * Return:			- ERROR or NOERROR
 */
static int  w1_seahu_read_from_device(struct w1_slave *sl, u8 cmd, u8 * data, int len, int Delay )
{
	u16 crc_read;
	u16 crc_calc;

	if (w1_seahu_select(sl, SELECT_ROM)) return ERROR;	// select w1 device
	w1_write_8(sl->master, cmd);						// send cmd
	delay(Delay);										// delay to slave will be ready
	w1_read_block(sl->master, data, len);				// read data
	w1_read_block(sl->master, (u8 *)&crc_read, 2);		// read crc16
	crc_calc=crc16( 0, &cmd , 1);						// calculate crc16 (cmd+data)
	crc_calc=~crc16( crc_calc, data, len);
	if (crc_read != crc_calc) {							// compare crc16
  		dev_dbg(&sl->dev, "w1 - bad check data");
		return ERROR;
  	}
	return NOERROR;
}

/**
 * w1_seahu_read_from_section() - read X bytes from section and CRC16 check
 *
 * @ls:			- pointer to slave device
 * @cmd:		- command code
 * @section:	- number of section
 * @data:		- pointer to data
 * @len:		- len of data
 * @Delay:		- delay in [us] to slave prepare data
 * @w1_select:	- way howto select slave  (SKIP_ROM, SELECT_ROM, RESUME_ROM)
 * Return:			- ERROR or NOERROR
 */
static int  w1_seahu_read_from_section(struct w1_slave *sl, u8 cmd, u8 section,  u8 * data, int len, int Delay, u8 w1_select)
{
	u16 crc_read;
	u16 crc_calc;

	if (w1_seahu_select(sl, w1_select)!=0) return ERROR;	// select w1 device
	w1_write_8(sl->master, cmd);							// send cmd
	w1_write_8(sl->master, section);						// send section
	delay(Delay);											// delay to slave will be ready
	w1_read_block(sl->master, data, len);					// read data
	w1_read_block(sl->master, (u8 *)&crc_read, 2);			// read crc16
	crc_calc=crc16( 0, &cmd , 1);							// calculate crc16 (cmd+section+data)
	crc_calc=crc16( crc_calc, &section , 1);
	crc_calc=~crc16( crc_calc, data, len);
	if (crc_read != crc_calc) {								// compare crc16
  		dev_dbg(&sl->dev, "w1 - bad check data");
		return ERROR;
  	}
	return NOERROR;
}

/**
 * w1_seahu_write_to_section() - write X bytes to section and CRC16 check
 *
 * @ls:			- pointer to slave device
 * @cmd:		- command code
 * @section:	- number of section
 * @data:		- pointer to data
 * @len:		- len of data
 * @Delay:		- delay in [us] to slave prepare data
 * @w1_select:	- way howto select slave  (SKIP_ROM, SELECT_ROM, RESUME_ROM)
 * Return:		- ERROR or NOERROR
 */
static int  w1_seahu_write_to_section(struct w1_slave *sl, u8 cmd, u8 section,  u8 * data, int len, int Delay, u8 w1_select)
{
	u16 crc_calc;

	if (w1_seahu_select(sl, w1_select)!=0) return ERROR;	// select w1 device
	w1_write_8(sl->master, cmd);							// send cmd
	w1_write_8(sl->master, section);						// send section
	w1_write_block(sl->master, data, len);					// send data
	crc_calc=crc16( 0, &cmd , 1);							// calculate crc16 (cmd+section+data)
	crc_calc=crc16( crc_calc, &section , 1);
	crc_calc=~crc16( crc_calc, data, len);
	w1_write_block(sl->master, (u8 *)&crc_calc, 2); 		// send crc16
	delay(Delay);											// delay
  	if (w1_read_8(sl->master) != 0xAA) {					// confirm answer
  		dev_dbg(&sl->dev, "w1 - bad check data");
		return ERROR;
  	}
	return NOERROR;
}



/* 
 * get_number_sections() - get number of sections without lock bus (used for ini module)
 *
 * @ls:			- pointer to slave device
 * Return:		- if all is OK return number of sections else return 0
 */
static u8 get_number_sections(struct w1_slave *sl)
{
	u8 number_sections;
	int i;
	printk(KERN_INFO "Go to read number of sections:\n");
	for (i=0;i<W1_FCC_RETRIES;i++) {
		if ( !w1_seahu_read_from_device(sl, _1W_FCC_READ_NUMBER_SECTIONS, &number_sections, 1, 0 )) {
			printk(KERN_INFO "1-Wire (0xCD family) Number of sections: %d \n", (int)number_sections);
			return number_sections;
		}
	}
	return 0;
}

/*
 * get_control_byte() - get control_byte of section
 *
 * @ls:				- pointer to slave device
 * @control_byte	- pointer to control byte (for return value)
 * @section:		- number of section
 * Return:			- ERROR or NOERROR
 */
static int get_control_byte(struct w1_slave *sl, u8 *control_byte, u8 section)
{
	printk(KERN_INFO "1-Wire : get_control_byte\n");
	if ( w1_seahu_read_from_section(sl, _1W_FCC_READ_CONTROL_BYTE, section,  control_byte, 1, 0, SELECT_ROM )){ // read control byte
		return ERROR;
	}
	return NOERROR;
}

/*
 * set control_byte() - set control_byte of section
 *
 * @ls:				- pointer to slave device
 * @control_byte	- new value of control byte
 * @section:		- number of section
 * Return:			- ERROR or NOERROR
 */
static int set_control_byte(struct w1_slave *sl, u8 control_byte, u8 section)
{
	if ( w1_seahu_write_to_section(sl, _1W_FCC_WRITE_CONTROL_BYTE, section,  &control_byte, 1, 0, SELECT_ROM )){ // read control byte
		return ERROR;
	}
	return NOERROR;
}


/* ---------------------------- save cumulation code function --------------------------*/

/*
 * print_int() - print signed integer value to output buffer
 *
 * @val:			- signed integer value
 * @buf				- output buffer
 * @off				- buffer offset
 * @count			- size of buffer
 * Return:			-  -EINVAL (ivaliet value) or count of buffer
 */
static ssize_t print_int( s32 val, char *buf,  loff_t off, size_t count)
{
	if (off!=0 )
		return -EINVAL;
	if (!buf)
		return -EINVAL;
	memset(buf, 0, count); // fill buffer to zero
	sprintf(buf , "%d\n", val);
	return count;
}

/*
 * print_int() - print unsigned integer value to output buffer
 *
 * @val:			- unsigned integer value
 * @buf:			- output buffer
 * @off:			- buffer offset
 * @count:			- size of buffer
 * Return:			- count of buffer or -EINVAL (invalid value)
 */
static ssize_t print_uint( u32 val, char *buf,  loff_t off, size_t count)
{
	if (off!=0 )
		return -EINVAL;
	if (!buf)
		return -EINVAL;
	memset(buf, 0, count); // fill buffer to zero
	sprintf(buf , "%d\n", val);
	return count;
}

/*
 * read_value() - read value from section (used for actual, min and max value)
 *
 * @file:			- pointer to file
 * @kobj:			- pointer to kernel object
 * @bin_attr:		- pointer to attributes
 * @buf:			- pointer to input buffer
 * @off:			- buffer offset
 * @count:			- size of buffer
 * @cmd: 			- command code (determine who value form slave device will be read)
 * Return:			- count of buffer or -EINVAL (invalid value) or -EIO (input/output error)
 */
static ssize_t read_value(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count, u8 cmd)
{
	u8 control_byte;
	u8 section_number=*((u8 *)bin_attr->private); // section number is stored on pointer bin_attr->private
	struct w1_slave *sl = kobj_to_w1_slave(kobj);
	val value;
	value.uInt=0; // prepare zerro value

	do_lock(sl);
	if (get_control_byte(sl, &control_byte, section_number)){ // get control byte + set actual section
		do_unlock(sl);
		return -EIO;
	}
	memset(&value.Buf,0,sizeof(value.Buf)); // erase old value
	if (w1_seahu_read_from_section(sl, cmd, section_number, value.Buf, 4, 0, SKIP_ROM )){ // read value
		dev_dbg(&sl->dev, "1-Wire ERROR read value\n");
		do_unlock(sl);
		return -EIO;
	}
	do_unlock(sl);
	
	if ( (control_byte & _MASK_TYPE_VALUE) == 0b00011000  ) {
		return print_uint( (u32)value.uInt, buf, off, count); // return unsignet value
	}
	else {
		return print_uint( (s32)value.Int, buf, off, count);	//return signed value
	}
}

/*
 * write_value() - write value to section (used for actualValue, min_alarm_value or max_alarm_value)
 *
 * @file:			- pointer to file
 * @kobj:			- pointer to kernel object
 * @bin_attr:		- pointer to attributes
 * @buf:			- pointer to output buffer
 * @off:			- buffer offset
 * @count:			- size of buffer
 * @cmd: 			- command code (determine who value form slave device will be read)
 * Return:			- count of buffer or -EINVAL (invalid value)
 */
static ssize_t write_value(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count, u8 cmd)
{
	u8 control_byte;
	u8 section_number=*((u8 *)bin_attr->private); // section number is stored on pointer bin_attr->private
	struct w1_slave *sl = kobj_to_w1_slave(kobj);
	val value;
	long res;
	
	// get control byte + set actual section
	do_lock(sl);
	if (get_control_byte(sl, &control_byte, section_number)){
		do_unlock(sl);
		return -EIO;
	}

	// string number to bunary number
	memset(&value.Buf,0,sizeof(value.Buf)); // erase old value
	if ( (control_byte & _MASK_TYPE_VALUE) == 0b00011000 ) { // unsigned
		if (kstrtoul (buf,0, &res)!=0) { // conversion unsigned string number into binary number
			do_unlock(sl);
			return -EIO; //bat converse string to long
		}
		value.uInt = (u32)res; // return unsignet value
	}
	else { // signed
		if (kstrtol (buf,0, &res)!=0) { // conversion signed string number into binary number
			do_unlock(sl);
			return -EIO; //bat converse string to long
		}
		value.Int = (s32)res;	//return signed value
	}

	// w1 write data
	if ( w1_seahu_write_to_section(sl, cmd, section_number,  value.Buf, 4, 0, SKIP_ROM) ) {
		do_unlock(sl);
		return -EIO;
	}
	do_unlock(sl);
	return count; // return number of successfully write bytes
}


/* ---------------------------- ATTR FUNCTION ------------------------------------------*/

/*
static ssize_t fs_example(struct file *filp, struct kobject *kobj,
			       struct bin_attribute *bin_attr,
			       char *buf, loff_t off, size_t count)
{
	char *tmp;
	char *pathname;
	struct path *path;
	u8 number_sections;
	char pwd[128];

	struct w1_slave *sl = kobj_to_w1_slave(kobj);
	dev_dbg(&sl->dev,
		"Reading %s kobj: %p, off: %0#10x, count: %zu, buff addr: %p",
		bin_attr->attr.name, kobj, (unsigned int)off, count, buf);
	//kernfs_name(kobj->sd, pwd, 128);
	//printk(KERN_INFO "1-Wire Name: %s \n", pwd);
	//printk(KERN_INFO "1-Wire Name: %s \n", kobj->parent->name);
	printk(KERN_INFO "1-Wire Name: %s \n", kobj->name);
	printk(KERN_INFO "1-Wire private: %d \n", (int)bin_attr->private);
	path = &filp->f_path;
	path_get(path);
	tmp = (char *)__get_free_page(GFP_KERNEL);

	if (!tmp) {
	    path_put(path);
	    return -ENOMEM;
	}

	pathname = d_path(path, tmp, PAGE_SIZE);
	path_put(path);

	if (IS_ERR(pathname)) {
	    free_page((unsigned long)tmp);
	    return PTR_ERR(pathname);
	}

	//printk(KERN_INFO "1-Wire file Name: %d \n", filp->f_path);
	printk(KERN_INFO "1-Wire file Name: %s \n", pathname);
	//w1_seahu_read_byte(sl, &number_sections, _1W_FCC_READ_NUMBER_SECTIONS);
	free_page((unsigned long)tmp);
	number_sections=2;
	return print_int((int)number_sections, buf, off, count);
}
*/

/*
 * fs_r_number_sections() - write value to section (used for actualValue, min_alarm_value or max_alarm_value)
 *
 * @file:			- pointer to file
 * @kobj:			- pointer to kernel object
 * @bin_attr:		- pointer to attributes
 * @buf:			- pointer to output buffer
 * @off:			- buffer offset
 * @count:			- size of buffer
 * @cmd: 			- command code (determine who value form slave device will be read)
 * Return:			- count of buffer or -EINVAL (invalid value)
 */
static ssize_t fs_r_number_sections(struct file *filp, struct kobject *kobj,
			       struct bin_attribute *bin_attr,
			       char *buf, loff_t off, size_t count)
{
	struct slave			*p_slave;
	struct w1_slave *sl = kobj_to_w1_slave(kobj);

	p_slave=(struct slave *)sl->family_data; //retype specific slave data into structure used in this module
	return print_int(p_slave->number_sections, buf, off, count);
}


//---------------- service of connection ----------------------------------------
static ssize_t fs_r_control_byte(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	u8 control_byte;
	u8 section_number=*((u8 *)bin_attr->private); // section number is stored on pointer bin_attr->private
	struct w1_slave *sl = kobj_to_w1_slave(kobj);
	
	printk(KERN_INFO "1-Wire fs_r_control_byte\n");

	do_lock(sl);
	if ( get_control_byte(sl, &control_byte, section_number) ) {
		do_unlock(sl);
		return -EIO;
	}
	do_unlock(sl);
	return print_int((int)control_byte, buf, off, count);
}

static ssize_t fs_w_control_byte(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	u8 control_byte;
	u8 section_number=*((u8 *)bin_attr->private); // section number is stored on pointer bin_attr->private
	struct w1_slave *sl = kobj_to_w1_slave(kobj);
	long res;
	
	// char conversion unsigned into binary number
	if (kstrtoul (buf,0, &res)!=0) { //bat converse string to long
		return -EIO;
	}
	control_byte=(u8)res;

	// write control byte
	do_lock(sl);
	if (set_control_byte(sl, control_byte, section_number )){
		do_unlock(sl);
		return -EIO;
	}
	do_unlock(sl);
	return count; // return number of successfully write bytes
}

static ssize_t fs_r_control_byte_flag(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count, u8 mask)
{
	u8 control_byte;
	u8 section_number=*((u8 *)bin_attr->private); // section number is stored on pointer bin_attr->private
	struct w1_slave *sl = kobj_to_w1_slave(kobj);
	int ret;
	
	do_lock(sl);
	if ( get_control_byte(sl, &control_byte, section_number) ){
		do_unlock(sl);
		return -EIO;
	}
	do_unlock(sl);
	if ( control_byte & mask ) 	ret = 1;
	else 						ret = 0;
	return print_int((s32)ret, buf, off, count);
}

static ssize_t fs_w_control_byte_flag(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count, u8 mask)
{
	u8 val;
	u8 section_number=*((u8 *)bin_attr->private); // section number is stored on pointer bin_attr->private
	u8 control_byte;
	u8 new_control_byte;
	struct w1_slave *sl = kobj_to_w1_slave(kobj);
	
	printk(KERN_INFO "1-Wire off:%d count:%d\n",(int)off ,(int)count);

	if ( (int)off   != 0 ) return -EINVAL;
	if ( (int)count != 1 ) return -EINVAL;

	switch (buf[0]) {
		case '0':
			val=0;
			break;
		case '1':
			val=1;
			break;
		default:
			return -EINVAL;
	}
	printk(KERN_INFO "1-Wire val:%d\n",(int)val);
	do_lock(sl);
	if ( get_control_byte(sl, &control_byte, section_number) ){
		do_unlock(sl);
		return -EIO;
	}

	printk(KERN_INFO "1-Wire control_byte:%d\n",(int)control_byte);
	new_control_byte=control_byte;
	if ( val==1 ) 	new_control_byte=control_byte | mask;
	if ( val==0 ) 	new_control_byte=control_byte & (~mask);
	printk(KERN_INFO "1-Wire new_control_byte:%d\n",(int)new_control_byte);
	if (new_control_byte!=control_byte){
		printk(KERN_INFO "1-Wire set new_control_byte\n");
		if (set_control_byte(sl, new_control_byte, section_number)){
			do_unlock(sl);
			return -EIO;
		}
	}
	do_unlock(sl);
	printk(KERN_INFO "1-Wire new_control_byte write OK\n");
	printk(KERN_INFO "return: %d\n", (int)count);
	return count; // return number of successfully write bytes
}

static ssize_t fs_r_enable_read(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	return fs_r_control_byte_flag(filp, kobj, bin_attr, buf, off, count, _MASK_ENABLE_READ);
}

static ssize_t fs_r_enable_write(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	return fs_r_control_byte_flag(filp, kobj, bin_attr, buf, off, count, _MASK_ENABLE_WRITE);
}

static ssize_t fs_r_give_alarm(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	return fs_r_control_byte_flag(filp, kobj, bin_attr, buf, off, count, _MASK_GIVE_ALARM);
}

static ssize_t fs_r_status_alarm(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	return fs_r_control_byte_flag(filp, kobj, bin_attr, buf, off, count, _MASK_STATUS_ALARM);
}

static ssize_t fs_r_value_min_alarm(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	return fs_r_control_byte_flag(filp, kobj, bin_attr, buf, off, count, _MASK_MIN_ALARM);
}

static ssize_t fs_w_value_min_alarm(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	return fs_w_control_byte_flag(filp, kobj, bin_attr, buf, off, count, _MASK_MIN_ALARM);
}

static ssize_t fs_r_value_max_alarm(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	return fs_r_control_byte_flag(filp, kobj, bin_attr, buf, off, count, _MASK_MAX_ALARM);
}

static ssize_t fs_w_value_max_alarm(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	return fs_w_control_byte_flag(filp, kobj, bin_attr, buf, off, count, _MASK_MAX_ALARM);
}

static ssize_t fs_r_actual_value(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	return read_value(filp, kobj, bin_attr, buf, off, count, _1W_FCC_READ_ACTUAL_VALUE);
}

static ssize_t fs_w_actual_value(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	return write_value(filp, kobj, bin_attr, buf, off, count, _1W_FCC_WRITE_ACTUAL_VALUE);
}

static ssize_t fs_r_measure(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	struct w1_slave *sl = kobj_to_w1_slave(kobj);
	u8 control_byte;
	struct timespec time_spec1, time_spec2;
	unsigned long delta_t;
	u8 section_number=*((u8 *)bin_attr->private); // section number is stored on pointer bin_attr->private

	do_lock(sl);
	if (w1_seahu_write_to_section(sl, _1W_FCC_START_MEASURE, section_number,  NULL, 0, 0, SELECT_ROM)) { // select (resume) slave device
		do_unlock(sl);
		return -EIO;
	}
	do_unlock(sl);

	delta_t=0;
	getnstimeofday(&time_spec1);
	getnstimeofday(&time_spec2);

	while ( ((time_spec2.tv_sec-time_spec1.tv_sec)*1000000000+ time_spec2.tv_nsec-time_spec1.tv_nsec) < (1000000*MAX_MEASURE_TIME) ){ // wait to be ready for read new actual value
		do_lock(sl);
		if ( get_control_byte(sl, &control_byte, section_number)==NOERROR ){
			if (control_byte & _MASK_ENABLE_READ) {
				do_unlock(sl);
				return fs_r_actual_value(filp, kobj, bin_attr, buf, off, count); // return new actual value
			}
		}
		do_unlock(sl);
		msleep_interruptible(50);
		getnstimeofday(&time_spec2);
	}
	dev_dbg(&sl->dev,"Measure time out.");
	return -EIO;
}

static ssize_t fs_r_min_alarm_value(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	return read_value(filp, kobj, bin_attr, buf, off, count, _1W_FCC_READ_MIN_ALARM_VALUE);
}

static ssize_t fs_w_min_alarm_value(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	return write_value(filp, kobj, bin_attr, buf, off, count, _1W_FCC_WRITE_MIN_ALARM_VALUE);
}

static ssize_t fs_r_max_alarm_value(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	return read_value(filp, kobj, bin_attr, buf, off, count, _1W_FCC_READ_MAX_ALARM_VALUE);
}

static ssize_t fs_w_max_alarm_value(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	return write_value(filp, kobj, bin_attr, buf, off, count, _1W_FCC_WRITE_MAX_ALARM_VALUE);
}

static ssize_t fs_r_description(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	u8 data[W1_SLAVE_DATA_SIZE_DESCRIPTION];
	struct w1_slave *sl = kobj_to_w1_slave(kobj);
	u8 section_number=*((u8 *)bin_attr->private); // section number is stored on pointer bin_attr->private

	if ( (off+count) > W1_SLAVE_DATA_SIZE_DESCRIPTION ) return -EINVAL;

	do_lock(sl);
	if ( w1_seahu_read_from_section(sl, _1W_FCC_READ_DESCRIPTION, section_number,  data, W1_SLAVE_DATA_SIZE_DESCRIPTION, DESCRIPTION_DELAY, SELECT_ROM) ){
		do_unlock(sl);
		return -EIO; // read description
	}
	do_unlock(sl);

	memcpy(buf, data+off, count);
	return count;
}

static ssize_t fs_r_user_note(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	u8 data[W1_SLAVE_DATA_SIZE_USER_NOTE];
	struct w1_slave *sl = kobj_to_w1_slave(kobj);
	u8 section_number=*((u8 *)bin_attr->private); // section number is stored on pointer bin_attr->private

	if ( (off+count) > W1_SLAVE_DATA_SIZE_USER_NOTE ) return -EINVAL;

	do_lock(sl);
	if ( w1_seahu_read_from_section(sl, _1W_FCC_READ_USER_NOTE, section_number,  data, W1_SLAVE_DATA_SIZE_USER_NOTE, READ_USER_NOTE_DEALY, SELECT_ROM) ){ // read user note
		do_unlock(sl);
		return -EIO;
	}
	do_unlock(sl);

	memcpy(buf, data+off, count);
	return count;

}

static ssize_t fs_w_user_note(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	u8 data[W1_SLAVE_DATA_SIZE_USER_NOTE];
	struct w1_slave *sl = kobj_to_w1_slave(kobj);
	u8 section_number=*((u8 *)bin_attr->private); // section number is stored on pointer bin_attr->private

	printk(KERN_INFO "1-Wire off:%d count:%d\n",(int)off ,(int)count);
	if ( (int)off != 0 ) return -EINVAL;
	if ( (int)count > W1_SLAVE_DATA_SIZE_USER_NOTE ) return -EINVAL;
	if ( (int)count == 0) return count;
	printk(KERN_INFO "1-Wire pokracuji\n");
	
	memset(data,0,W1_SLAVE_DATA_SIZE_USER_NOTE);
	memcpy(data, buf, count);

	do_lock(sl);
	if ( w1_seahu_write_to_section(sl, _1W_FCC_WRITE_USER_NOTE, section_number,  data, W1_SLAVE_DATA_SIZE_USER_NOTE, WRITE_USER_NOTE_DELAY, SELECT_ROM) ){ // write user note
		do_unlock(sl);
		return -EIO;
	}
	do_unlock(sl);

	return count;
}

static ssize_t fs_r_device_description(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	u8 data[W1_SLAVE_DATA_SIZE_DEVICE_DESCRIPTION];
	struct w1_slave *sl = kobj_to_w1_slave(kobj);

	if ( (off+count) > W1_SLAVE_DATA_SIZE_DESCRIPTION ) return -EINVAL;
	//memset(data,0,W1_SLAVE_DATA_SIZE_DESCRIPTION);

	do_lock(sl);
	if ( w1_seahu_read_from_device(sl, _1W_FCC_READ_DEVICE_DESCRIPTION, data, W1_SLAVE_DATA_SIZE_DEVICE_DESCRIPTION, DEVICE_DESCRIPTION_DELAY) ){
		do_unlock(sl);
		return -EIO; // read description
	}
	do_unlock(sl);

	memcpy(buf, data+off, count);
	return count;
}


/* ---------------------- SUPPORT FOR SYSFS  --------------------------------------------------------------*/
/*
 * DEFINE SHARE ATTRIBUTES OVER ALL SECTIONS
*/
static BIN_ATTR(number_sections,	S_IRUGO | S_IRUSR | S_IRGRP, fs_r_number_sections,		NULL, W1_SLAVE_DATA_SIZE);
static BIN_ATTR(device_description,	S_IRUGO | S_IRUSR | S_IRGRP, fs_r_device_description,	NULL, W1_SLAVE_DATA_SIZE_DEVICE_DESCRIPTION);

static struct bin_attribute *w1_seahu_cc_share_bin_attrs[] = {
	&bin_attr_number_sections,
	&bin_attr_device_description,
	NULL,
};

static const struct attribute_group w1_seahu_cc_share_group = {
	.name = NULL,
	.is_visible = NULL,
	.is_bin_visible = NULL,
	.attrs = NULL,
	.bin_attrs = w1_seahu_cc_share_bin_attrs,
};

/*
 * Help function who set dynamically created attributes
 */
void set_attribute( u8 *section_number, struct bin_attribute *p_struct_bin_attribute,
		const char *name,
		umode_t	mode,
		ssize_t (*read)(struct file *, struct kobject *, struct bin_attribute *, char *, loff_t, size_t),
		ssize_t (*write)(struct file *, struct kobject *, struct bin_attribute *, char *, loff_t, size_t),
		size_t	size
		)
{
	p_struct_bin_attribute->attr.name = name;
	p_struct_bin_attribute->attr.mode =  mode;
	p_struct_bin_attribute->read = read;
	p_struct_bin_attribute->write  = write;
	p_struct_bin_attribute->size   = size;
	//p_struct_bin_attribute->private = (int)section_number;
	//p_struct_bin_attribute->private = (void *)section_number;
	p_struct_bin_attribute->private = section_number;
}

/*
 * fill bin_attr array entries for specified section during dynamically created attributes
 */
void set_attributes(int section_number, struct section *p_section){
	// store section number (for use in pointer bin_attribute->private)
	p_section->section_number=section_number;
	// fill seahu_cc_bin_attrs[section][0] array with bin_attr structures
	set_attribute( &p_section->section_number ,&p_section->control_byte,		"control_byte",		S_IRUGO | S_IRUGO, fs_r_control_byte,		fs_w_control_byte,		W1_SLAVE_DATA_SIZE_BYTE );
	set_attribute( &p_section->section_number ,&p_section->enable_read,			"enable_read",		S_IRUGO | S_IRUGO, fs_r_enable_read,		NULL,					W1_SLAVE_DATA_SIZE );
	set_attribute( &p_section->section_number ,&p_section->enable_write,		"enable_write",		S_IRUGO | S_IRUGO, fs_r_enable_write,		NULL,					W1_SLAVE_DATA_SIZE );
	set_attribute( &p_section->section_number ,&p_section->give_alarm,			"give_alarm",		S_IRUGO | S_IRUGO, fs_r_give_alarm,			NULL,					W1_SLAVE_DATA_SIZE );
	set_attribute( &p_section->section_number ,&p_section->status_alarm,		"status_alarm",		S_IRUGO | S_IRUGO, fs_r_status_alarm,		NULL,					W1_SLAVE_DATA_SIZE );
	set_attribute( &p_section->section_number ,&p_section->value_min_alarm,	"value_min_alarm", S_IRUGO | S_IWUSR, fs_r_value_min_alarm,	fs_w_value_min_alarm,	W1_SLAVE_DATA_SIZE );
	set_attribute( &p_section->section_number ,&p_section->value_max_alarm,	"value_max_alarm", S_IRUGO | S_IWUSR, fs_r_value_max_alarm,	fs_w_value_max_alarm,	W1_SLAVE_DATA_SIZE );
	set_attribute( &p_section->section_number ,&p_section->measure,				"measure", 			S_IRUGO | S_IRUGO, fs_r_measure,			NULL,					W1_SLAVE_DATA_SIZE_VALUE );
	set_attribute( &p_section->section_number ,&p_section->actual_value,		"actual_value", 	S_IRUGO | S_IWUSR, fs_r_actual_value,		fs_w_actual_value,		W1_SLAVE_DATA_SIZE_VALUE );
	set_attribute( &p_section->section_number ,&p_section->min_alarm_value,		"min_alarm_value", 	S_IRUGO | S_IWUSR, fs_r_min_alarm_value,	fs_w_min_alarm_value,	W1_SLAVE_DATA_SIZE_VALUE );
	set_attribute( &p_section->section_number ,&p_section->max_alarm_value,		"max_alarm_value", 	S_IRUGO | S_IWUSR, fs_r_max_alarm_value,	fs_w_max_alarm_value,	W1_SLAVE_DATA_SIZE_VALUE );
	set_attribute( &p_section->section_number ,&p_section->description,			"description", 		S_IRUGO | S_IRUGO, fs_r_description,		NULL,					W1_SLAVE_DATA_SIZE_DESCRIPTION );
	set_attribute( &p_section->section_number ,&p_section->user_note,			"user_note", 		S_IRUGO | S_IWUSR, fs_r_user_note,			fs_w_user_note,			W1_SLAVE_DATA_SIZE_USER_NOTE );
	// fill pointer into array of bin_attributes
	p_section->bin_attrs[0]=&p_section->control_byte;
	p_section->bin_attrs[1]=&p_section->enable_read;
	p_section->bin_attrs[2]=&p_section->enable_write;
	p_section->bin_attrs[3]=&p_section->give_alarm;
	p_section->bin_attrs[4]=&p_section->status_alarm;
	p_section->bin_attrs[5]=&p_section->value_min_alarm;
	p_section->bin_attrs[6]=&p_section->value_max_alarm;
	p_section->bin_attrs[7]=&p_section->measure;
	p_section->bin_attrs[8]=&p_section->actual_value;
	p_section->bin_attrs[9]=&p_section->min_alarm_value;
	p_section->bin_attrs[10]=&p_section->max_alarm_value;
	p_section->bin_attrs[11]=&p_section->description;
	p_section->bin_attrs[12]=&p_section->user_note;
	p_section->bin_attrs[13]=NULL;
}

/*
 * dynamic allocated memory ,create and fill all necessary structures by number of sections
 */
static int seahu_cd_add_slave_dynamic(struct w1_slave *sl){
	u8 number_sections;
	int i;
	struct slave			*p_slave;
	struct section			*p_sections;
	const struct attribute_group	**p_to_p_groups;
	
	printk(KERN_INFO "w1- sl :%s.\n", sl->name);
	// first get number of sections
	sl->master->slave_count++; // this device is not calculated in sl->master->slave_count jet. But select slave depend off slave_count
	number_sections=get_number_sections(sl); // (in w1 communication can not use bus lock because this is already lock)
	sl->master->slave_count--; // return back. sl->master->slave_count will be increased on end of add procedure
	printk(KERN_INFO "w1- number section :%d.\n", number_sections);
	//if (number_sections>100) {
	//	printk(KERN_INFO "w1- quantity of section (%d) exceed max number section (100).\n", number_sections);
	//	return 1; // max section is 100 (section0 - section99)
	//}

	// allocate memory for dynamically created entries for all sections
	printk(KERN_INFO "w1- start reserve memory (malloc)\n");
	p_sections		= kmalloc(sizeof(struct section)*number_sections, GFP_KERNEL); //reserve memory for sections
	if (!p_sections)	return 1;
	p_to_p_groups	= kmalloc(sizeof(struct attribute_group **)*(number_sections+2), GFP_KERNEL); //reserve memory for array with pointers into sysfs groups.
	if (!p_to_p_groups) return 1;
	p_slave			= kmalloc(sizeof(struct slave), GFP_KERNEL); //reserve memory for slave structure
	if (!p_slave)		return 1;
	printk(KERN_INFO "w1- reserve memory (malloc) OK.\n");

	// fill sections structures, bin_attribute and sysfs_group for all section and share attribute
	printk(KERN_INFO "w1- start cycle for ini sections.\n");
	for (i=0; i<number_sections; i++){
		printk(KERN_INFO "w1- start ini section number %d.\n", i);
		set_attributes(i, p_sections+i); // fill attributes and array of attributes for section
		sprintf((p_sections+i)->name,"section%d",i); // prepare string of name sections
		(p_sections+i)->sysfs_group_attr.name=&(p_sections+i)->name[0]; // set pointer to name of sysfs group
		(p_sections+i)->sysfs_group_attr.attrs=NULL;
		(p_sections+i)->sysfs_group_attr.is_visible=NULL;
		(p_sections+i)->sysfs_group_attr.is_bin_visible=NULL;
		(p_sections+i)->sysfs_group_attr.bin_attrs=&(p_sections+i)->bin_attrs[0]; // set pointer to array of bin attributes for sysfs group
		*(p_to_p_groups+i)=&(p_sections+i)->sysfs_group_attr; // add link to sysfs group (into array with pointers to sysfs groups)
		//printk(KERN_INFO "w1- section name: %s.\n", (p_sections+i)->name);
		//printk(KERN_INFO "w1- 1.attr name: %s.\n", (p_sections+i)->control_byte.attr.name);
		//printk(KERN_INFO "w1- 1.attr name: %s.\n", ((p_sections+i)->sysfs_group_attr).bin_attrs[0]->attr.name);
	}
	printk(KERN_INFO "w1- value of i: %d.\n", i);
	*(p_to_p_groups+i)=&w1_seahu_cc_share_group; // add link to share group (this group don't have own name, this mean all atributes is placed into root directory of device)
	i++;
	*(p_to_p_groups+i)=NULL; // add end links of groups

	// remember kmaloc address for further free in remote slave function
	p_slave->p_sections=p_sections;
	p_slave->p_to_p_groups=p_to_p_groups;
	p_slave->number_sections=number_sections; // number_sections is constant value therefore remember it for whole live slave
	sl->family_data=p_slave; // remember kmaloc address for further free in remote slave function
	printk(KERN_INFO "w1- goto back to crete groups.\n");
	sl->family->fops->groups=p_to_p_groups; // join syfs groups to w1
	return 0;
}

/*
 * free dynamically reserve memory by slave
 */
static void w1_seahu_cd_remove_slave(struct w1_slave *sl){
	struct slave			*p_slave;

	printk(KERN_INFO "w1 - remove.\n");
	if ( *sl->family->fops->groups==NULL) return;
	p_slave=(struct slave *)sl->family_data; //data for free is organized by struct sections *p_slave
	// actual kernel 2018-01 have bug frist remove slave than remove sysfs entries, right way is vice versa (fortunately is not critical bug)
	kfree(p_slave->p_sections);
	kfree(p_slave->p_to_p_groups);
	kfree(p_slave);
	return;
}

/*---- START AND STOP  W1 SEAHU CC DRIVER --- */

static struct w1_family_ops w1_seahu_cd_fops = {
	.add_slave	= seahu_cd_add_slave_dynamic,
	.remove_slave = w1_seahu_cd_remove_slave,
	//.groups		= w1_seahu_cc_groups, // is set in function seahu_cc_add_slave_dynamic (sl->family->fops->groups=p_to_p_groups;)
};

static struct w1_family w1_seahu_cc = {
	.fid = W1_FAMILY_SEAHU_CD,
	.fops = &w1_seahu_cd_fops,
};

static int __init w1_seahu_cc_init(void)
{
	//w1_seahu_cc_init_attrs_array();
	return w1_register_family(&w1_seahu_cc);
}

static void __exit w1_seahu_cc_exit(void)
{
	w1_unregister_family(&w1_seahu_cc);
}

module_init(w1_seahu_cc_init);
module_exit(w1_seahu_cc_exit);
