/*
 * Copyright (c) 2020 MADMACHINE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <errno.h>

#include "SwiftI2C.h"
#include "SwiftDigitalIO.h"
#include "SwiftPlatform.h"

#include "MadGt9x.h"

#define LOG_ERR
#define LOG_INF
#define LOG_DBG

#define GT9X_I2C_ADDRESS_A  0x14
#define GT9X_I2C_ADDRESS_B  0x5D

#define MADGT9X_I2C_ADDR GT9X_I2C_ADDRESS_B

#define TOUCH_CONTINUE

#define GT9X_COOR_ADDR    0x814E
#define GT9X_REG_SLEEP         0x8040
#define GT9X_REG_SENSOR_ID     0x814A
#define GT9X_REG_CONFIG_DATA   0x8050
#define GT9X_REG_VERSION       0x8140

#define GT9X_REG_NUMBER 256
#define GT9X_REG_ADDR_LEN   2

#define GT9X_VERSION_LEN        6

#define GT9X_MAX_TOUCH  5
#define GT9X_PER_TOUCH_LEN  8
#define GT9X_TOUCH_STATUS_LEN   1
#define GT9X_TOUCH_RESERVED_LEN 1

static unsigned char gt9x_cfg_regs[] = {
	0x97, 0x20, 0x03, 0xE0, 0x01, 0x0A, 0x35, 0x04, 0x00, 0x69,
	0x09, 0x0F, 0x50, 0x32, 0x33, 0x11, 0x00, 0x32, 0x11, 0x11,
	0x28, 0x8C, 0xAA, 0xDC, 0x58, 0x04, 0x00, 0x00, 0x1E, 0x3C,
	0x00, 0x00, 0x00, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40,
	0x32, 0x00, 0x00, 0x50, 0x38, 0x00, 0x8D, 0x20, 0x16, 0x4E,
	0x4C, 0x7C, 0x05, 0x28, 0x3E, 0x28, 0x0D, 0x43, 0x24, 0x00,
	0x01, 0x39, 0x6B, 0xC0, 0x94, 0x84, 0x2D, 0x00, 0x54, 0xB0,
	0x41, 0x9D, 0x49, 0x8D, 0x52, 0x7F, 0x5A, 0x75, 0x62, 0x6C,
	0x42, 0x50, 0x14, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x50, 0x3C,
	0x88, 0x88, 0x27, 0x50, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x00, 0x02, 0x78,
	0x0A, 0x50, 0xFF, 0xE4, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x3C, 0xB0, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x56, 0xA2, 0x07, 0x50, 0x1E,
	0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
	0x0F, 0x10, 0x12, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B,
	0x1D, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0x1F, 0x1E, 0x1D, 0x1C, 0x1B, 0x1A, 0x19, 0x18,
	0x17, 0x15, 0x14, 0x13, 0x12, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x30, 0x7F, 0x7F, 0x7F, 0xFF,
	0x54, 0x64, 0x00, 0x80, 0x46, 0x07, 0x50, 0x3C, 0x32, 0x14,
	0x0A, 0x64, 0x32, 0x00, 0x00, 0x00, 0x00, 0x11, 0x02, 0x62,
	0x32, 0x03, 0x14, 0x50, 0x0C, 0xE2, 0x14, 0x50, 0x00, 0x54,
	0x10, 0x00, 0x32, 0xA2, 0x07, 0x64, 0xA4, 0xB6, 0x01
};

struct touch_point_info {
	unsigned char id;
	unsigned short x;
	unsigned short y;
	unsigned short size;
};


static void *gpio_rst;
static void *gpio_int;
static void *i2c_touch;
static madGt9x_callback_t key_callback = NULL;

static int gt9x_write_regs(unsigned char dev_addr,
			   unsigned short reg_addr, unsigned char *regs, unsigned short reg_num)
{
	unsigned char buf[GT9X_REG_ADDR_LEN + GT9X_REG_NUMBER] =
	{ reg_addr >> 8, reg_addr & 0xFF, };

	if (reg_num > GT9X_REG_NUMBER) {
		return -1;
	}
	memcpy(&buf[GT9X_REG_ADDR_LEN], regs, reg_num);

	return swiftHal_i2cWrite(i2c_touch, dev_addr, buf, reg_num + GT9X_REG_ADDR_LEN);
}


static int gt9x_read_regs(unsigned char dev_addr,
			  unsigned short reg_addr, unsigned char *regs, unsigned short reg_num)
{
	unsigned char addr[2] = { reg_addr >> 8, reg_addr & 0xFF };


	swiftHal_i2cWrite(i2c_touch, dev_addr, addr, 2);
	return swiftHal_i2cRead(i2c_touch, dev_addr, regs, reg_num);
	// return swiftHal_i2cWriteRead(&i2c_touch, dev_addr, addr, 2, regs, reg_num);
}

int madGt9x_read(void)
{
	unsigned char buf[GT9X_PER_TOUCH_LEN * GT9X_MAX_TOUCH];
	int ret = 0;
	unsigned char status = 0;
	unsigned char i, j;
	unsigned char touch_number = 0;
	struct touch_point_info cur_touch[GT9X_MAX_TOUCH] = { 0 };

	static unsigned char pre_touch_number = 0;
	static struct touch_point_info pre_touch[GT9X_MAX_TOUCH] = { 0 };

	// read touch status
	ret = gt9x_read_regs(MADGT9X_I2C_ADDR,
			     GT9X_COOR_ADDR, &status, 1);
	if (ret) {
		LOG_ERR("Could not read point\r\n");
		return -EIO;
	}

	// LOG_DBG("read %x", status);

	/* No data */
	if (status == 0) {
		return 0;
	}

	do {
		// data no ready
		if ((status & 0x80) == 0) {
			break;
		}

		touch_number = status & 0x0f;
		if (touch_number > GT9X_MAX_TOUCH) {
			break;
		}

		if (touch_number == 0) {
			for (i = 0; i < pre_touch_number; i++) {
				// printk("Release %u %u\n", pre_touch[i].x, pre_touch[i].y);
				if (key_callback != NULL) {
					LOG_INF("Release[%d] %u %u\r\n", pre_touch[i].id, pre_touch[i].x, pre_touch[i].y);
					key_callback(pre_touch[i].y, pre_touch[i].x, 0);
				}
			}
			pre_touch_number = 0;
			break;
		}

		// LOG_DBG("Touch %d", touch_number);

		ret = gt9x_read_regs(MADGT9X_I2C_ADDR,
				     GT9X_COOR_ADDR + GT9X_TOUCH_STATUS_LEN,
				     buf, touch_number * GT9X_PER_TOUCH_LEN);

		if (ret) {
			LOG_ERR("Could not read point\r\n");
			ret = -EIO;
			break;
		}

		// check press key
		for (i = 0; i < touch_number; i++) {
			cur_touch[i].id = buf[i * GT9X_PER_TOUCH_LEN];
			cur_touch[i].x = buf[i * GT9X_PER_TOUCH_LEN + 2];
			cur_touch[i].x = (cur_touch[i].x << 8) | buf[i * GT9X_PER_TOUCH_LEN + 1];
			cur_touch[i].y = buf[i * GT9X_PER_TOUCH_LEN + 4];
			cur_touch[i].y = (cur_touch[i].y << 8) | buf[i * GT9X_PER_TOUCH_LEN + 3];
			cur_touch[i].size = buf[i * GT9X_PER_TOUCH_LEN + 6];
			cur_touch[i].size = (cur_touch[i].size << 8) | buf[i * GT9X_PER_TOUCH_LEN + 5];
			// LOG_INF("Current Touch %u %u", cur_touch[i].x, cur_touch[i].y);
#ifdef TOUCH_CONTINUE
			// printk("Touch %u %u -- %x\n", cur_touch[i].x, cur_touch[i].y, key_callback);
			if (key_callback != NULL) {
				LOG_INF("Touch[%d] %u %u\r\n", cur_touch[i].id, cur_touch[i].x, cur_touch[i].y);
				key_callback(cur_touch[i].y, cur_touch[i].x, 1);
			}
#else
			for (j = 0; j < pre_touch_number; j++) {
				if (cur_touch[i].id == pre_touch[j].id) {
					break;
				}
			}

			if (j >= pre_touch_number) {
				LOG_INF("Touch[%d] %u %u\r\n", cur_touch[i].id, cur_touch[i].x, cur_touch[i].y);
				// printk("Touch %u %u -- %x\n", cur_touch[i].x, cur_touch[i].y, key_callback);
				if (key_callback != NULL) {
					key_callback(cur_touch[i].y, cur_touch[i].x, 1);
				}
			}
#endif
		}

		// check release key
		for (i = 0; i < pre_touch_number; i++) {
			for (j = 0; j < touch_number; j++) {
				if (cur_touch[j].id == pre_touch[i].id) {
					break;
				}
			}

			if (j >= touch_number) {
				LOG_INF("Release[%d] %u %u\r\n", pre_touch[i].id, pre_touch[i].x, pre_touch[i].y);
				// printk("Release %u %u\n", cur_touch[i].x, cur_touch[i].y);
				if (key_callback != NULL) {
					key_callback(pre_touch[i].y, pre_touch[i].x, 0);
				}
			}
		}

		if (touch_number != pre_touch_number) {
			LOG_DBG("====================================================================\r\n");
			LOG_DBG("touch %d pre touch %d\r\n", touch_number, pre_touch_number);
			for (i = 0; i < touch_number; i++) {
				LOG_DBG("   touch[%d] %u %u\r\n", cur_touch[i].id, cur_touch[i].x, cur_touch[i].y);
			}
			for (i = 0; i < pre_touch_number; i++) {
				LOG_DBG("   pretouch[%d] %u %u\r\n", pre_touch[i].id, pre_touch[i].x, pre_touch[i].y);
			}
			LOG_DBG("====================================================================\r\n");
		}

		memcpy(pre_touch, cur_touch, sizeof(cur_touch));
		pre_touch_number = touch_number;
	} while (0);

	status = 0;
	gt9x_write_regs(MADGT9X_I2C_ADDR,
			GT9X_COOR_ADDR, &status, GT9X_TOUCH_STATUS_LEN);


	return ret;
}


int madGt9x_init(int width, int height, int touch)
{
	unsigned char version[GT9X_VERSION_LEN];
	int ret = -1;
	unsigned short check_sum;
	unsigned short reg_num;

	i2c_touch = swiftHal_i2cInit(I2CId1);
	if (i2c_touch == NULL) {
		return -1;
	}

	gpio_rst = swiftHal_gpioInit(D3, dirOutput, pushPull);
	gpio_int = swiftHal_gpioInit(D0, dirOutput, pushPull);

	swiftHal_gpioWrite(gpio_rst, 0);
	swiftHal_gpioWrite(gpio_int, 0);

	swiftHal_msSleep(1);

	if (MADGT9X_I2C_ADDR == GT9X_I2C_ADDRESS_A) {
		swiftHal_gpioWrite(gpio_int, 1);
	} else if (MADGT9X_I2C_ADDR == GT9X_I2C_ADDRESS_B) {
		swiftHal_gpioWrite(gpio_int, 0);
	} else {
		return -1;
	}

	swiftHal_msSleep(1);
	swiftHal_gpioWrite(gpio_rst, 1);

	swiftHal_msSleep(6);

	swiftHal_gpioDeinit(gpio_int);
	gpio_int = swiftHal_gpioInit(D0, dirInput, pullNone);

	ret = gt9x_read_regs(MADGT9X_I2C_ADDR,
			     GT9X_REG_VERSION, version, GT9X_VERSION_LEN);

	if (ret < 0) {
		LOG_DBG("I2C read/write reg fail\r\n");
		return 0;
	}

	if (version[0] == '9' && version[1] == '1' && version[2] == '7' && version[3] == 'S') {
		LOG_DBG("Current Device GT917S\r\n");
	} else if (version[0] == '9' && version[1] == '1' && version[2] == '4' && version[3] == '7') {
		LOG_DBG("Current Device GT9147\r\n");
	} else {
		LOG_DBG("No support %c%c%c%c\r\n", version[0], version[1], version[2], version[3]);
	}

	gt9x_cfg_regs[1] = width & 0xFF;
	gt9x_cfg_regs[2] = width >> 8;
	gt9x_cfg_regs[3] = height & 0xFF;
	gt9x_cfg_regs[4] = height >> 8;

	reg_num = sizeof(gt9x_cfg_regs);
	check_sum = 0;
	for (int i = 0; i < reg_num; i += 2) {
		check_sum += (gt9x_cfg_regs[i] << 8) + gt9x_cfg_regs[i + 1];
	}
	check_sum = 0 - check_sum;

	unsigned char buf[reg_num + 3];
	memcpy(buf, gt9x_cfg_regs, reg_num);
	buf[reg_num] = (check_sum >> 8) & 0xFF;
	buf[reg_num + 1] =  check_sum & 0xFF;
	buf[reg_num + 2] =  0x01;
	reg_num += 3;

	ret = gt9x_write_regs(MADGT9X_I2C_ADDR,
			      GT9X_REG_CONFIG_DATA, buf, reg_num);

	if (ret != 0) {
		LOG_ERR("confiure gt9x fail\r\n");
		return -EINVAL;
	}

	swiftHal_msSleep(50);
	/*
	   memset(buf, 0, sizeof(buf));
	   ret = gt9x_read_regs(MADGT9X_I2C_ADDR,
	                     GT9X_REG_CONFIG_DATA, buf, sizeof(gt9x_cfg_regs));
	   if (ret != 0) {
	        LOG_ERR("read gt9x fail\n");
	   }

	   ret = memcmp(buf, gt9x_cfg_regs, sizeof(gt9x_cfg_regs));
	   LOG_INF("cmp cfg result %d\n", ret); */

	return 0;
}



int madGt9x_configure(madGt9x_callback_t callback)
{
	if (!callback) {
		return -EINVAL;
	}

	key_callback = callback;

	return 0;
}

int madGt9x_start(int ms)
{
	return 0;
}

int madGt9x_stop(void)
{
	return 0;
}



