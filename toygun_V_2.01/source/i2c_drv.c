#include "../include/i2c_drv.h"

const I2C_Bus g_i2c_ucb0 =
{
    &UCB0CTL1, &UCB0IFG, &UCB0STAT,
    &UCB0TXBUF, &UCB0RXBUF, &UCB0I2CSA
};

const I2C_Bus g_i2c_ucb2 =
{
    &UCB2CTL1, &UCB2IFG, &UCB2STAT,
    &UCB2TXBUF, &UCB2RXBUF, &UCB2I2CSA
};

static i2c_status_t wait_flag(const I2C_Bus *bus, uint8_t flag)
{
    volatile uint32_t t = 50000UL;

    while (((*bus->ifg) & flag) == 0U)
    {
        if ((*bus->stat) & UCNACKIFG)
        {
            (*bus->ctl1) |= UCTXSTP;
            (*bus->stat) &= (uint8_t)(~UCNACKIFG);
            return I2C_ERR_NACK;
        }

        if (--t == 0UL)
        {
            (*bus->ctl1) |= UCTXSTP;
            return I2C_ERR_TIMEOUT;
        }
        __no_operation();
    }

    return I2C_OK;
}

void i2c_init_ucb0(void)
{
    /* P3.1 (SDA), P3.2 (SCL) configured in gpio_init() */

    UCB0CTL1 |= UCSWRST;
    UCB0CTL0  = UCMST | UCMODE_3 | UCSYNC;
    UCB0CTL1  = UCSWRST | UCSSEL_2;
    UCB0BR0   = 10;
    UCB0BR1   = 0;
    UCB0CTL1 &= (uint8_t)(~UCSWRST);
    UCB0IFG   = 0;
}

void i2c_init_ucb2(void)
{
    /* P9.1 (SDA), P9.2 (SCL) configured in gpio_init() */

    UCB2CTL1 |= UCSWRST;
    UCB2CTL0  = UCMST | UCMODE_3 | UCSYNC;
    UCB2CTL1  = UCSWRST | UCSSEL_2;
    UCB2BR0   = 10;
    UCB2BR1   = 0;
    UCB2CTL1 &= (uint8_t)(~UCSWRST);
    UCB2IFG   = 0;
}

static void wait_stop(const I2C_Bus *bus)
{
    volatile uint16_t timeout = 10000U;
    while ((*bus->ctl1) & UCTXSTP)
    {
        if (--timeout == 0) break;
        __no_operation();
    }
}

i2c_status_t i2c_write_bytes(const I2C_Bus *bus, uint8_t dev_addr, const uint8_t *data, uint8_t len)
{
    i2c_status_t err;

    if ((bus == 0) || (data == 0) || (len == 0))
        return I2C_ERR_PARAM;

    wait_stop(bus);

    *bus->i2csa = dev_addr;
    (*bus->stat) &= (uint8_t)(~UCNACKIFG);
    (*bus->ctl1) |= UCTR | UCTXSTT;

    while (len--)
    {
        err = wait_flag(bus, UCTXIFG);
        if (err != I2C_OK)
            return err;

        *bus->txbuf = *data++;
    }

    err = wait_flag(bus, UCTXIFG);
    if (err != I2C_OK)
        return err;

    (*bus->ctl1) |= UCTXSTP;
    wait_stop(bus);

    return I2C_OK;
}

i2c_status_t i2c_write_reg8(const I2C_Bus *bus, uint8_t dev_addr, uint8_t reg, uint8_t value)
{
    uint8_t pkt[2];
    pkt[0] = reg;
    pkt[1] = value;
    return i2c_write_bytes(bus, dev_addr, pkt, 2);
}

i2c_status_t i2c_write_reg_multi(const I2C_Bus *bus, uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint8_t len)
{
    i2c_status_t err;

    if ((bus == 0) || (data == 0) || (len == 0))
        return I2C_ERR_PARAM;

    wait_stop(bus);

    *bus->i2csa = dev_addr;
    (*bus->stat) &= (uint8_t)(~UCNACKIFG);
    (*bus->ctl1) |= UCTR | UCTXSTT;

    err = wait_flag(bus, UCTXIFG);
    if (err != I2C_OK)
        return err;
    *bus->txbuf = reg;

    while (len--)
    {
        err = wait_flag(bus, UCTXIFG);
        if (err != I2C_OK)
            return err;

        *bus->txbuf = *data++;
    }

    err = wait_flag(bus, UCTXIFG);
    if (err != I2C_OK)
        return err;

    (*bus->ctl1) |= UCTXSTP;
    wait_stop(bus);

    return I2C_OK;
}

i2c_status_t i2c_read_bytes(const I2C_Bus *bus, uint8_t dev_addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    i2c_status_t err;

    if ((bus == 0) || (data == 0) || (len == 0))
        return I2C_ERR_PARAM;

    wait_stop(bus);

    *bus->i2csa = dev_addr;
    (*bus->stat) &= (uint8_t)(~UCNACKIFG);

    (*bus->ctl1) |= UCTR | UCTXSTT;

    err = wait_flag(bus, UCTXIFG);
    if (err != I2C_OK)
        return err;
    *bus->txbuf = reg;

    err = wait_flag(bus, UCTXIFG);
    if (err != I2C_OK)
        return err;

    (*bus->ctl1) &= (uint8_t)(~UCTR);
    (*bus->ctl1) |= UCTXSTT;

    {
        volatile uint16_t start_timeout = 10000U;
        while ((*bus->ctl1) & UCTXSTT)
        {
            if ((*bus->stat) & UCNACKIFG)
            {
                (*bus->ctl1) |= UCTXSTP;
                (*bus->stat) &= (uint8_t)(~UCNACKIFG);
                return I2C_ERR_NACK;
            }
            if (--start_timeout == 0)
            {
                (*bus->ctl1) |= UCTXSTP;
                return I2C_ERR_TIMEOUT;
            }
            __no_operation();
        }
    }

    while (len--)
    {
        if (len == 0U)
            (*bus->ctl1) |= UCTXSTP;

        err = wait_flag(bus, UCRXIFG);
        if (err != I2C_OK)
            return err;

        *data++ = *bus->rxbuf;
    }

    wait_stop(bus);

    return I2C_OK;
}

i2c_status_t i2c_read_reg8(const I2C_Bus *bus, uint8_t dev_addr, uint8_t reg, uint8_t *value)
{
    return i2c_read_bytes(bus, dev_addr, reg, value, 1);
}