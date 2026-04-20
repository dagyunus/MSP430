#include "tmp101.h"

i2c_status_t tmp101_init(const I2C_Bus *bus, uint8_t addr)
{
    return i2c_write_reg8(bus, addr, TMP101_REG_CONFIG, 0x60);
}

i2c_status_t tmp101_read_temp(const I2C_Bus *bus, uint8_t addr, float *temp)
{
    uint8_t b[2];
    int16_t raw;
    i2c_status_t err;

    if ((bus == 0) || (temp == 0))
        return I2C_ERR_PARAM;

    err = i2c_read_bytes(bus, addr, TMP101_REG_TEMP, b, 2);
    if (err != I2C_OK)
        return err;

    raw = (int16_t)((((uint16_t)b[0] << 8) | b[1]) >> 4);

    if (raw & 0x0800)
        raw |= (int16_t)0xF000;

    *temp = (float)raw * 0.0625f;
    return I2C_OK;
}