/*
 * @Copyright (c) 2020, MADMACHINE LIMITED
 * @Author: Frank Li(lgl88911@163.com)
 * @SPDX-License-Identifier: MIT
 */

#ifndef _MADGT9X_H_
#define _MADGT9X_H_

/**
 * @brief GT9X hardware interface
 *
 * @param i2c  I2c handle for communicating with gt9x
 * @param gpio_rst GPIO handle that controls gt9x reset
 * @param gpio_int GPIO handle that controls gt9x address and receive gt9x int signal
 */
struct mad_gt9x_hw {
	void *i2c;
	void *gpio_rst;
	void *gpio_int;
};

typedef struct mad_gt9x_hw mad_gt9x_hw_t;

/**
 * @brief GT9X touch screen config paramrater
 *
 * @param width Valid width of touch screen
 * @param hight Valid hight of touch screen
 * @param touch Maximum touch point
 */
struct mad_gt9x_cfg {
	int width;
	int height;
	int touch;
};

typedef struct mad_gt9x_cfg mad_gt9x_cfg_t;

/**
 * @brief GT9X touch point status
 *
 * @param row Row coordinates
 * @param column Column coordinates
 * @param touch Press status
 */
struct mad_gt9x_key {
	int row;
	int column;
	int pressed;
};

typedef struct mad_gt9x_key mad_gt9x_key_t;

typedef void (*mad_gt9x_callback_t)(int row, int column, int pressed);

/**
 * @brief Initialize the gt9x device driver
 *
 * @param hw GT9X hardware interface handle , use @ref mad_gt9x_hw
 * @param cfg touch screen config paramrater, use @ref mad_gt9x_cfg
 *
 * @retval 0 If successful.
 * @retval Negative errno code if failure.
 */
int mad_gt9x_init(mad_gt9x_hw_t *hw, mad_gt9x_cfg_t *cfg);

/**
 * @brief configure key event callback
 *
 * Currently, interrupt callback is not supported.
 * You can create a task to periodically call mad_gt9x_read.
 * If a key event occurs, callback will be called in mad_gt9x_read.
 *
 * @param callback key event calloback
 *
 * @retval 0 If successful.
 * @retval Negative errno code if failure.
 */
int mad_gt9x_configure(mad_gt9x_callback_t callback);

/**
 * @brief Read touch key
 *
 * @param keys Output key buffer
 * @param key_num  key buffer size
 *
 * @return Number of key events
 */
int mad_gt9x_read(mad_gt9x_key_t *keys, int key_num);

/**
 * @brief Start capture Key
 *
 * Reserved function, effective after interrupt callback is supported
 *
 * @param ms key scan period time
 *
 * @retval 0 If successful.
 * @retval Negative errno code if failure.
 */
int mad_gt9x_start(int ms);

/**
 * @brief Stop capture Key
 *
 * Reserved function, effective after interrupt callback is supported
 *
 * @retval 0 If successful.
 * @retval Negative errno code if failure.
 */
int mad_gt9x_stop(void);



#endif /* _MADGT9X_H_ */