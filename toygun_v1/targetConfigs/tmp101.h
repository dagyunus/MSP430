#ifndef TMP101_H
#define TMP101_H

#include <stdint.h>
#include "i2c_drv.h"

#define TMP101_REG_TEMP    0x00
#define TMP101_REG_CONFIG  0x01

i2c_status_t tmp101_init(const I2C_Bus *bus, uint8_t addr);
i2c_status_t tmp101_read_temp(const I2C_Bus *bus, uint8_t addr, float *temp);

#endif