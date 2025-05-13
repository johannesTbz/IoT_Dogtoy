#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>
#include "esp_err.h"

void mpu9250_init(void);
int16_t read_accel_x(void);

#endif
