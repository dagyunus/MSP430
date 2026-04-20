#ifndef DS1682_H
#define DS1682_H

#include <stdint.h>
#include "i2c_drv.h"

#define DS1682_REG_CONFIG  0x00
#define DS1682_REG_ALARM   0x01
#define DS1682_REG_ETC     0x05
#define DS1682_REG_EVENT   0x09

i2c_status_t ds1682_read_config(const I2C_Bus *bus, uint8_t addr, uint8_t *config);
i2c_status_t ds1682_read_etc(const I2C_Bus *bus, uint8_t addr, uint32_t *qsec);
i2c_status_t ds1682_read_event_count(const I2C_Bus *bus, uint8_t addr, uint16_t *count);
i2c_status_t ds1682_set_alarm(const I2C_Bus *bus, uint8_t addr, uint32_t quarter_seconds);

#endif