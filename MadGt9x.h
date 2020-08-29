#ifndef _MADGT9X_H_
#define _MADGT9X_H_

typedef void (*madGt9x_callback_t)(int row, int column, int pressed);

int madGt9x_init(int width, int height, int touch);
int madGt9x_configure(madGt9x_callback_t callback);
int madGt9x_read(void);
int madGt9x_start(int ms);
int madGt9x_stop(void);



#endif /* _MADGT9X_H_ */