#ifndef I2C_DRV_H
#define I2C_DRV_H

#include <msp430.h>
#include <stdint.h>

typedef enum
{
    I2C_OK = 0,
    I2C_ERR_NACK = 1,
    I2C_ERR_TIMEOUT = 2,
    I2C_ERR_PARAM = 3
} i2c_status_t;

typedef struct
{
    volatile uint8_t  *ctl1;
    volatile uint8_t  *ifg;
    volatile uint8_t  *stat;
    volatile uint8_t  *txbuf;
    volatile uint8_t  *rxbuf;
    volatile uint16_t *i2csa;
} I2C_Bus;

extern const I2C_Bus g_i2c_ucb0;
extern const I2C_Bus g_i2c_ucb2;

void i2c_init_ucb0(void);
void i2c_init_ucb2(void);

i2c_status_t i2c_write_bytes(const I2C_Bus *bus, uint8_t dev_addr, const uint8_t *data, uint8_t len);
i2c_status_t i2c_write_reg8(const I2C_Bus *bus, uint8_t dev_addr, uint8_t reg, uint8_t value);
i2c_status_t i2c_write_reg_multi(const I2C_Bus *bus, uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint8_t len);
i2c_status_t i2c_read_bytes(const I2C_Bus *bus, uint8_t dev_addr, uint8_t reg, uint8_t *data, uint8_t len);
i2c_status_t i2c_read_reg8(const I2C_Bus *bus, uint8_t dev_addr, uint8_t reg, uint8_t *value);

#endif