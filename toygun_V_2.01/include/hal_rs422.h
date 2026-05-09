#ifndef HAL_RS422_H_
#define HAL_RS422_H_

#include <msp430.h>
#include <stdint.h>

#define RS422_BUFFER_SIZE 128U

/* UART ayarları (115200 baud @ 1MHz SMCLK) */
#define HAL_RS422_BR0_VALUE       8U
#define HAL_RS422_BR1_VALUE       0U
#define HAL_RS422_MCTL_VALUE      UCBRS_6

/* RS422 DE/RE geçiş süreleri (TimerA1 sayacı cinsinden) */
#define HAL_RS422_TX_PRE_DELAY    500U
#define HAL_RS422_TX_POST_DELAY   1000U

typedef struct
{
    struct
    {
        uint8_t  Data[RS422_BUFFER_SIZE];
        uint16_t Length;
        uint16_t Counter;
        volatile uint8_t Busy;
    } SentData;

    struct
    {
        uint8_t  Data[RS422_BUFFER_SIZE];
        volatile uint16_t Counter;
        volatile uint16_t Idle;
        volatile uint8_t  Overflow;
    } RecData;

} RS422_Handle_t;

extern RS422_Handle_t RS422;

void HAL_RS422_Init(void);
void HAL_RS422_SendPacket(void);
void HAL_RS422_ClearRx(void);
uint8_t HAL_RS422_TakeRxFrame(uint8_t *dst, uint16_t *len, uint16_t max_len);
uint8_t HAL_RS422_TickRxIdle(uint8_t *dst, uint16_t *len, uint16_t max_len, uint16_t idle_timeout);

#endif
