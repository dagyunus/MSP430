#include <msp430.h>
#include "../include/hal_system.h"
#include "../include/hal_rs422.h"

static volatile uint16_t g_timer_ms_countdown = 0;

void HAL_System_ClockInit(void)
{
    UCSCTL3 = SELREF_2;
    UCSCTL4 = SELA_2 | SELS_4 | SELM_4;
    __bis_SR_register(SCG0);
    UCSCTL0 = 0x0000;
    UCSCTL1 = DCORSEL_2;
    UCSCTL2 = FLLD_1 | 30; // ~1 MHz
    __bic_SR_register(SCG0);
    
    do {
        UCSCTL7 &= ~(XT2OFFG | XT1LFOFFG | DCOFFG);
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG);
}

void HAL_System_TimerInit(void)
{
    /* 1MHz'de 499 -> 500us (0.5ms) hassasiyet. Modbus T3.5 için çok önemli. */
    TA0CCR0 = 499;
    TA0CCTL0 = CCIE;
    TA0CTL = TASSEL_2 | MC_1 | TACLR;
}

void HAL_System_SleepMs(uint16_t ms)
{
    /* Artık sayacımız 0.5ms olduğu için ms değerini 2 ile çarpıyoruz */
    g_timer_ms_countdown = ms * 2;
    while (g_timer_ms_countdown > 0)
    {
        __bis_SR_register(LPM0_bits | GIE);
    }
}

extern volatile uint32_t g_update_timer;

#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{
    /* RS422 Idle Counter increment (Her 0.5ms'de bir artar) */
    RS422.RecData.Idle++;
    
    /* update_timer saniye hesabı yapıldığı için 1ms olması adına her 2 turda 1 arttırmak mantıklı ama
     * main'de >= 2000 olarak kontrol ediyorduk. Şimdi >= 2000 demek 1 saniye oldu. Bu da güzel. */
    g_update_timer++;

    if (g_timer_ms_countdown > 0)
    {
        g_timer_ms_countdown--;
    }
    
    __bic_SR_register_on_exit(LPM0_bits);
}
