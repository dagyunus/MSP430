#ifndef ADS1015_H
#define ADS1015_H

#include <stdint.h>
#include "i2c_drv.h"

#define ADS1015_REG_CONV    0x00
#define ADS1015_REG_CONFIG  0x01

i2c_status_t ads1015_write_config(const I2C_Bus *bus, uint8_t addr, uint16_t cfg);
i2c_status_t ads1015_read_raw(const I2C_Bus *bus, uint8_t addr, uint16_t *raw12);
i2c_status_t ads1015_read_channel(const I2C_Bus *bus, uint8_t addr, uint8_t ch, uint16_t *raw12, float *volt);

#endif