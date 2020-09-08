/*
 * @Copyright (c) 2020, MADMACHINE LIMITED
 * @Author: Frank Li(lgl88911@163.com)
 * @SPDX-License-Identifier: MIT
 */

#ifndef _MADGT9X_H_
#define _MADGT9X_H_

typedef struct {
	void *i2c;
	void *gpio_rst;
	void *gpio_int;
} madGt9x_hw;

typedef struct {
	int width;
	int height;
	int touch;
} madGt9x_cfg;

typedef struct {
	int row;
	int column;
	int pressed;
} madGt9x_key;

typedef void (*madGt9x_callback_t)(int row, int column, int pressed);

int madGt9x_init(madGt9x_hw *hw, madGt9x_cfg *cfg);
int madGt9x_configure(madGt9x_callback_t callback);
int madGt9x_read(madGt9x_key *keys, int key_num);
int madGt9x_start(int ms);
int madGt9x_stop(void);



#endif /* _MADGT9X_H_ */