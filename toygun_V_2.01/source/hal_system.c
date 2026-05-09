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
    TA0CCR0 = 999;
    TA0CCTL0 = CCIE;
    TA0CTL = TASSEL_2 | MC_1 | TACLR;
}

void HAL_System_SleepMs(uint16_t ms)
{
    g_timer_ms_countdown = ms;
    while (g_timer_ms_countdown > 0)
    {
        __bis_SR_register(LPM0_bits | GIE);
    }
}

extern volatile uint32_t g_update_timer;

#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{
    /* RS422 Idle Counter increment */
    RS422.RecData.Idle++;
    g_update_timer++;

    if (g_timer_ms_countdown > 0)
    {
        g_timer_ms_countdown--;
    }
    
    __bic_SR_register_on_exit(LPM0_bits);
}
