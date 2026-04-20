#include "ads1015.h"

i2c_status_t ads1015_write_config(const I2C_Bus *bus, uint8_t addr, uint16_t cfg)
{
    uint8_t pkt[2];
    pkt[0] = (uint8_t)(cfg >> 8);
    pkt[1] = (uint8_t)(cfg & 0xFF);
    return i2c_write_reg_multi(bus, addr, ADS1015_REG_CONFIG, pkt, 2);
}

i2c_status_t ads1015_read_raw(const I2C_Bus *bus, uint8_t addr, uint16_t *raw12)
{
    uint8_t buf[2];
    uint16_t raw16;
    i2c_status_t err;

    if (raw12 == 0)
        return I2C_ERR_PARAM;

    err = i2c_read_bytes(bus, addr, ADS1015_REG_CONV, buf, 2);
    if (err != I2C_OK)
        return err;

    raw16 = ((uint16_t)buf[0] << 8) | buf[1];
    *raw12 = (raw16 >> 4) & 0x0FFF;

    return I2C_OK;
}

i2c_status_t ads1015_read_channel(const I2C_Bus *bus, uint8_t addr, uint8_t ch, uint16_t *raw12, float *volt)
{
    uint16_t cfg;
    uint16_t raw;
    i2c_status_t err;

    if ((raw12 == 0) || (volt == 0))
        return I2C_ERR_PARAM;

    switch (ch)
    {
        case 0: cfg = 0xC383; break;
        case 1: cfg = 0xD383; break;
        case 2: cfg = 0xE383; break;
        case 3: cfg = 0xF383; break;
        default: return I2C_ERR_PARAM;
    }

    err = ads1015_write_config(bus, addr, cfg);
    if (err != I2C_OK)
        return err;

    __delay_cycles(5000);

    err = ads1015_read_raw(bus, addr, &raw);
    if (err != I2C_OK)
        return err;

    *raw12 = raw;
    *volt = ((float)raw * 4.096f) / 2048.0f;

    return I2C_OK;
}