#include "../include/i2c_drv.h"

#define I2C_MAX_TX_BYTES      8U
#define I2C_MAX_RX_BYTES      8U
#define I2C_TIMEOUT_TICKS     20U
#define I2C_IE_ALL            (UCNACKIE | UCALIE | UCSTPIE | UCTXIE | UCRXIE)

typedef enum
{
    I2C_STATE_IDLE = 0,
    I2C_STATE_TX,
    I2C_STATE_RX,
    I2C_STATE_DONE,
    I2C_STATE_ERROR
} i2c_state_t;

typedef struct
{
    const I2C_Bus *bus;
    volatile i2c_state_t state;
    volatile i2c_status_t status;
    volatile uint8_t busy;
    volatile uint8_t done;
    volatile uint8_t tx_idx;
    volatile uint8_t rx_idx;
    volatile uint8_t tx_len;
    volatile uint8_t rx_len;
    volatile uint16_t timeout;
    uint8_t tx[I2C_MAX_TX_BYTES];
    uint8_t rx[I2C_MAX_RX_BYTES];
} i2c_context_t;

const I2C_Bus g_i2c_ucb0 =
{
    &UCB0CTL1, &UCB0IFG, &UCB0STAT,
    &UCB0TXBUF, &UCB0RXBUF, &UCB0I2CSA, &UCB0IE
};

const I2C_Bus g_i2c_ucb2 =
{
    &UCB2CTL1, &UCB2IFG, &UCB2STAT,
    &UCB2TXBUF, &UCB2RXBUF, &UCB2I2CSA, &UCB2IE
};

static i2c_context_t g_ucb0_ctx = { &g_i2c_ucb0 };
static i2c_context_t g_ucb2_ctx = { &g_i2c_ucb2 };

static i2c_context_t *ctx_from_bus(const I2C_Bus *bus)
{
    if (bus == &g_i2c_ucb0) return &g_ucb0_ctx;
    if (bus == &g_i2c_ucb2) return &g_ucb2_ctx;
    return 0;
}

static void i2c_finish(i2c_context_t *ctx, i2c_status_t status)
{
    *(ctx->bus->ie) &= (uint8_t)(~I2C_IE_ALL);
    ctx->status = status;
    ctx->busy = 0;
    ctx->done = 1;
    ctx->state = (status == I2C_OK) ? I2C_STATE_DONE : I2C_STATE_ERROR;
}

static i2c_status_t i2c_start_job(const I2C_Bus *bus, uint8_t dev_addr,const uint8_t *tx, uint8_t tx_len,uint8_t rx_len)
{
    i2c_context_t *ctx = ctx_from_bus(bus);
    uint8_t i;

    if ((ctx == 0) || (tx == 0) || (tx_len == 0U) || (tx_len > I2C_MAX_TX_BYTES) || (rx_len > I2C_MAX_RX_BYTES))
        return I2C_ERR_PARAM;
    
    if ((ctx->busy != 0U) || (((*bus->ctl1) & UCTXSTP) != 0U))
        return I2C_ERR_BUSY;

    i = tx_len;
    while (i--)
        ctx->tx[i] = tx[i];

    ctx->tx_len = tx_len;
    ctx->rx_len = rx_len;
    ctx->tx_idx = 0U;
    ctx->rx_idx = 0U;
    ctx->status = I2C_OK;
    ctx->done = 0U;
    ctx->busy = 1U;
    ctx->timeout = I2C_TIMEOUT_TICKS;
    ctx->state = I2C_STATE_TX;

    *bus->i2csa = dev_addr;
    *bus->ifg = 0U;
    (*bus->stat) &= (uint8_t)(~(UCNACKIFG | UCALIFG));
    *bus->ie = (UCNACKIE | UCALIE | UCTXIE);
    (*bus->ctl1) |= UCTR | UCTXSTT;

    return I2C_OK;
}

void i2c_init_ucb0(void)
{
    UCB0CTL1 |= UCSWRST;
    UCB0CTL0  = UCMST | UCMODE_3 | UCSYNC;
    UCB0CTL1  = UCSWRST | UCSSEL_2;
    UCB0BR0   = 10;
    UCB0BR1   = 0;
    UCB0IE    = 0;
    UCB0IFG   = 0;
    UCB0CTL1 &= (uint8_t)(~UCSWRST);
}

void i2c_init_ucb2(void)
{
    UCB2CTL1 |= UCSWRST;
    UCB2CTL0  = UCMST | UCMODE_3 | UCSYNC;
    UCB2CTL1  = UCSWRST | UCSSEL_2;
    UCB2BR0   = 10;
    UCB2BR1   = 0;
    UCB2IE    = 0;
    UCB2IFG   = 0;
    UCB2CTL1 &= (uint8_t)(~UCSWRST);
}

void i2c_service_tick(void)
{
    i2c_context_t *contexts[2] = { &g_ucb0_ctx, &g_ucb2_ctx };
    uint8_t i;

    i = 2U;
    while (i--)
    {
        i2c_context_t *ctx = contexts[i];
        if (ctx->busy != 0U)
        {
            if (ctx->timeout > 0U)
                ctx->timeout--;
            else
            {
                *(ctx->bus->ctl1) |= UCTXSTP;
                i2c_finish(ctx, I2C_ERR_TIMEOUT);
            }
        }
    }
}

uint8_t i2c_is_busy(const I2C_Bus *bus)
{
    i2c_context_t *ctx = ctx_from_bus(bus);
    if (ctx == 0) return 0U;
    return ctx->busy;
}

i2c_status_t i2c_async_write_reg_multi(const I2C_Bus *bus, uint8_t dev_addr,uint8_t reg, const uint8_t *data,uint8_t len)
{
    uint8_t tx[I2C_MAX_TX_BYTES];
    uint8_t i;

    if ((data == 0) || (len == 0U) || ((uint8_t)(len + 1U) > I2C_MAX_TX_BYTES))
        return I2C_ERR_PARAM;

    tx[0] = reg;
    i = len;
    while (i--)
        tx[i + 1U] = data[i];

    return i2c_start_job(bus, dev_addr, tx, (uint8_t)(len + 1U), 0U);
}

i2c_status_t i2c_async_read_bytes(const I2C_Bus *bus, uint8_t dev_addr, uint8_t reg, uint8_t len)
{
    return i2c_start_job(bus, dev_addr, &reg, 1U, len);
}

i2c_status_t i2c_async_poll(const I2C_Bus *bus, uint8_t *data, uint8_t *len)
{
    i2c_context_t *ctx = ctx_from_bus(bus);
    uint8_t i;

    if ((ctx == 0) || (len == 0)) return I2C_ERR_PARAM;

    if (ctx->busy != 0U) return I2C_ERR_BUSY;
    
    if (ctx->done == 0U)  return I2C_ERR_BUSY;
    
    ctx->done = 0U;
    *len = ctx->rx_len;

    if ((data != 0) && (ctx->status == I2C_OK))
    {
        i = ctx->rx_len;
        while (i--)
            data[i] = ctx->rx[i];
    }

    return ctx->status;
}

i2c_status_t i2c_write_bytes(const I2C_Bus *bus, uint8_t dev_addr, const uint8_t *data, uint8_t len)
{
    return i2c_start_job(bus, dev_addr, data, len, 0U);
}

i2c_status_t i2c_write_reg8(const I2C_Bus *bus, uint8_t dev_addr, uint8_t reg, uint8_t value)
{
    return i2c_async_write_reg_multi(bus, dev_addr, reg, &value, 1U);
}

i2c_status_t i2c_write_reg_multi(const I2C_Bus *bus, uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint8_t len)
{
    return i2c_async_write_reg_multi(bus, dev_addr, reg, data, len);
}

i2c_status_t i2c_read_bytes(const I2C_Bus *bus, uint8_t dev_addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    i2c_status_t status;
    uint8_t rx_len = 0U;

    status = i2c_async_read_bytes(bus, dev_addr, reg, len);
    if (status != I2C_OK) return status;

    status = i2c_async_poll(bus, data, &rx_len);
    return (status == I2C_ERR_BUSY) ? I2C_OK : status;
}

i2c_status_t i2c_read_reg8(const I2C_Bus *bus, uint8_t dev_addr, uint8_t reg, uint8_t *value)
{
    return i2c_read_bytes(bus, dev_addr, reg, value, 1U);
}

static inline void i2c_isr(i2c_context_t *ctx, uint16_t iv)
{
    const I2C_Bus *bus = ctx->bus;
    volatile uint8_t dummy;

    ctx->timeout = I2C_TIMEOUT_TICKS;

    switch (iv)
    {
        case USCI_I2C_UCALIFG:
            (*bus->ctl1) |= UCTXSTP;
            i2c_finish(ctx, I2C_ERR_NACK);
            break;

        case USCI_I2C_UCNACKIFG:
            (*bus->ctl1) |= UCTXSTP;
            i2c_finish(ctx, I2C_ERR_NACK);
            break;

        case USCI_I2C_UCRXIFG:
            if (ctx->rx_idx >= ctx->rx_len)
            {
                dummy = *bus->rxbuf;
                (void)dummy;
                break;
            }

            if (ctx->rx_idx == (uint8_t)(ctx->rx_len - 1U))
                (*bus->ctl1) |= UCTXSTP;

            ctx->rx[ctx->rx_idx++] = *bus->rxbuf;

            if (ctx->rx_idx >= ctx->rx_len)
                i2c_finish(ctx, I2C_OK);
            
            break;

        case USCI_I2C_UCTXIFG:
            if (ctx->tx_idx < ctx->tx_len)
                *bus->txbuf = ctx->tx[ctx->tx_idx++];
            
            else if (ctx->rx_len > 0U)
            {
                ctx->state = I2C_STATE_RX;
                *bus->ie = (UCNACKIE | UCALIE | UCRXIE);
                (*bus->ctl1) &= (uint8_t)(~UCTR);
                (*bus->ctl1) |= UCTXSTT;
            }
            else
            {
                (*bus->ctl1) |= UCTXSTP;
                i2c_finish(ctx, I2C_OK);
            }
            break;

        default:
            break;
    }
}

#pragma vector=USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
    i2c_isr(&g_ucb0_ctx, UCB0IV);
    __bic_SR_register_on_exit(LPM0_bits);
}

#pragma vector=USCI_B2_VECTOR
__interrupt void USCI_B2_ISR(void)
{
    i2c_isr(&g_ucb2_ctx, UCB2IV);
    __bic_SR_register_on_exit(LPM0_bits);
}
