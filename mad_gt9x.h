/*
 * @Copyright (c) 2020, MADMACHINE LIMITED
 * @Author: Frank Li(lgl88911@163.com)
 * @SPDX-License-Identifier: MIT
 */

#ifndef _MADGT9X_H_
#define _MADGT9X_H_

struct mad_gt9x_hw {
	void *i2c;
	void *gpio_rst;
	void *gpio_int;
};

typedef struct mad_gt9x_hw mad_gt9x_hw_t;

struct mad_gt9x_cfg {
	int width;
	int height;
	int touch;
};

typedef struct mad_gt9x_cfg mad_gt9x_cfg_t;

struct mad_gt9x_key {
	int row;
	int column;
	int pressed;
};

typedef struct mad_gt9x_key mad_gt9x_key_t;

typedef void (*mad_gt9x_callback_t)(int row, int column, int pressed);

int mad_gt9x_init(mad_gt9x_hw_t *hw, mad_gt9x_cfg_t *cfg);
int mad_gt9x_configure(mad_gt9x_callback_t callback);
int mad_gt9x_read(mad_gt9x_key_t *keys, int key_num);
int mad_gt9x_start(int ms);
int mad_gt9x_stop(void);



#endif /* _MADGT9X_H_ */