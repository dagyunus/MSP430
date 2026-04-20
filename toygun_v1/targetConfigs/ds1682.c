#include "ds1682.h"

i2c_status_t ds1682_read_config(const I2C_Bus *bus, uint8_t addr, uint8_t *config)
{
    if (config == 0)
        return I2C_ERR_PARAM;

    return i2c_read_reg8(bus, addr, DS1682_REG_CONFIG, config);
}

i2c_status_t ds1682_read_etc(const I2C_Bus *bus, uint8_t addr, uint32_t *qsec)
{
    uint8_t buf[4];
    i2c_status_t err;

    if (qsec == 0)
        return I2C_ERR_PARAM;

    err = i2c_read_bytes(bus, addr, DS1682_REG_ETC, buf, 4);
    if (err != I2C_OK)
        return err;

    *qsec  = (uint32_t)buf[0];
    *qsec |= (uint32_t)buf[1] << 8;
    *qsec |= (uint32_t)buf[2] << 16;
    *qsec |= (uint32_t)buf[3] << 24;

    return I2C_OK;
}

i2c_status_t ds1682_read_event_count(const I2C_Bus *bus, uint8_t addr, uint16_t *count)
{
    uint8_t buf[2];
    i2c_status_t err;

    if (count == 0)
        return I2C_ERR_PARAM;

    err = i2c_read_bytes(bus, addr, DS1682_REG_EVENT, buf, 2);
    if (err != I2C_OK)
        return err;

    *count  = (uint16_t)buf[0];
    *count |= (uint16_t)buf[1] << 8;

    return I2C_OK;
}

i2c_status_t ds1682_set_alarm(const I2C_Bus *bus, uint8_t addr, uint32_t quarter_seconds)
{
    uint8_t buf[4];

    buf[0] = (uint8_t)(quarter_seconds & 0xFF);
    buf[1] = (uint8_t)((quarter_seconds >> 8) & 0xFF);
    buf[2] = (uint8_t)((quarter_seconds >> 16) & 0xFF);
    buf[3] = (uint8_t)((quarter_seconds >> 24) & 0xFF);

    return i2c_write_reg_multi(bus, addr, DS1682_REG_ALARM, buf, 4);
}