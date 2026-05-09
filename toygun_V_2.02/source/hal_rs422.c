#include <msp430.h>
#include <stdint.h>
#include "../include/hal_rs422.h"
#include "../include/gpio.h"

RS422_Handle_t RS422;

void HAL_RS422_Init(void)
{
    /* RS422 UART (USCI_A2) Pins: P9.4 (TX), P9.5 (RX) */
    P9SEL |= (RS422_TX_PIN | RS422_RX_PIN);

    /* Direction Control Pins: P9.6 (RE#), P9.7 (DE) */
    P9DIR |= (RS422_RE_PIN | RS422_DE_PIN);
    RS422_RX_ENABLE(); // Başlangıçta Alıcı modu

    /* 115200 Baud @ 1MHz SMCLK */
    UCA2CTL1 |= UCSWRST;
    UCA2CTL1 |= UCSSEL_2; 
    UCA2BR0 = HAL_RS422_BR0_VALUE;
    UCA2BR1 = HAL_RS422_BR1_VALUE;
    UCA2MCTL = HAL_RS422_MCTL_VALUE;
    UCA2CTL1 &= ~UCSWRST;

    UCA2IE |= UCRXIE;     // Enable RX Interrupt

    /* Clear handles */
    RS422.RecData.Counter = 0;
    RS422.RecData.Idle = 0;
}

uint8_t HAL_RS422_TickRxIdle(uint8_t *dst, uint16_t *len, uint16_t max_len, uint16_t timeout_ms)
{
    uint16_t i;
    uint16_t rx_len;

    if (RS422.RecData.Counter == 0) return 1;
    
    /* Sessizlik süresi (Idle) Timer ISR içinde artırılır */
    if (RS422.RecData.Idle < timeout_ms) return 1;

    rx_len = RS422.RecData.Counter;
    if (rx_len > max_len) rx_len = max_len;
    
    i = rx_len;
    while (i > 0U)
    {
        i--;
        dst[i] = RS422.RecData.Data[i];
    }

    *len = rx_len;
    RS422.RecData.Counter = 0;
    RS422.RecData.Idle = 0;
    return 0;
}

void HAL_RS422_SendPacket(void)
{
    if (RS422.SentData.Length == 0) return;

    /* Verici moduna geç (DE=1, RE#=1) */
    RS422_TX_ENABLE();
    RS422_RX_DISABLE();

    __delay_cycles(100); 

    RS422.SentData.Busy = 1;
    RS422.SentData.Counter = 0;

    /* İlk byte'ı göndererek kesme döngüsünü başlat */
    UCA2TXBUF = RS422.SentData.Data[RS422.SentData.Counter++];
    UCA2IE |= UCTXIE;
}

#pragma vector=USCI_A2_VECTOR
__interrupt void USCI_A2_ISR(void)
{
    switch(__even_in_range(UCA2IV, 4))
    {
        case 2: // RXIFG
            if (RS422.RecData.Counter < RS422_BUFFER_SIZE)
            {
                RS422.RecData.Data[RS422.RecData.Counter++] = UCA2RXBUF;
                RS422.RecData.Idle = 0;
                __bic_SR_register_on_exit(LPM0_bits); // Uyan ve veriyi isle
            }
            break;

        case 4: // TXIFG
            if (RS422.SentData.Counter < RS422.SentData.Length)
            {
                UCA2TXBUF = RS422.SentData.Data[RS422.SentData.Counter++];
            }
            else
            {
                UCA2IE &= ~UCTXIE;
                
                /* Paket bitti, Alıcı moduna geri dön */
                while (UCA2STAT & UCBUSY);    
                
                RS422_RX_ENABLE();
                RS422_TX_DISABLE();
                RS422.SentData.Busy = 0;
            }
            break;
    }
}
