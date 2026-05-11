#include <msp430.h>
#include "../include/gpio.h"

void gpio_init(void)
{
    /* Port 1 & 2 -> Hepsi Çıkış */
    P1DIR = 0xFF; P1OUT = 0x00;
    P2DIR = 0xFF; P2OUT = 0x00;
    
    /* Port 3: I2C Pinleri (P3.1, P3.2) Giriş/Special, Diğerleri Çıkış */
    P3SEL |= (BIT1 | BIT2); 
    P3DIR = 0xF9; // 1111 1001 (BIT1 ve BIT2 Giriş)
    P3OUT = FAN_POW_OFF_LED_PIN;
    
    /* Port 4: FORCE_LTC4364_UV_PIN (P4.7) Çıkış (LOW) olmalı! */
    P4DIR = 0xD8; // 1101 1000 (Bit 7 Çıkış)
    P4OUT = 0x00; // Tüm pinler başlangıçta LOW
    
    /* Port 5: Hepsi Çıkış */
    P5OUT &= ~(BIT0 | BIT1 | BIT4 | BIT5 | BIT6 | BIT7);
    P5DIR |=  (BIT0 | BIT1 | BIT4 | BIT5 | BIT6 | BIT7);
    P5DIR &= ~(BIT2 | BIT3);   // XT2IN / XT2OUT output yapılmaz
    
    P6DIR &= ~(BIT0 | BIT1 | BIT2 | BIT3 | BIT4);
    P6SEL |=  (BIT0 | BIT1 | BIT2 | BIT3 | BIT4);
    
    /* Port 7: FORCE_LTC4364_SHDN (P7.3) Çıkış (LOW) olmalı! */
    P7OUT &= ~(BIT3 | BIT4 | BIT5 | BIT6); // P7.3 LOW yapıldı!
    P7DIR |= (BIT3 | BIT4 | BIT5 | BIT6);  // Bu pinleri output yap
    
    /* Port 8: FORCE_LTC4364_OV_PIN (P8.2) Çıkış (LOW) olmalı! */
    P8DIR = 0x8D; // 1000 1101 (Bit 2 Çıkış)
    P8OUT = 0x00; // P8.2 LOW yapıldı!

    
    /* Port 9: I2C ve UART Pinleri */
    P9SEL |= (BIT1 | BIT2); // I2C SDA/SCL
    P9DIR = 0xF9; // 1111 1001
    P9OUT = 0x00;
    
    /* Başlangıç LED ve Güç Durumları */
    P7OUT |= ON_MODE_LED_PIN; 
    P4OUT |= VOUT_PWR_CTRL_PIN;
    P4OUT |= FAN_PWR_CTRL_PIN;
    P4OUT |= EXT_FAULT_FLAG_PIN;
    P3OUT |= FAN_POW_OFF_LED_PIN;

    /* ULP 4.1: Port E (P9/P10) ve Port F (P11/PJ) kullanılmayan pinleri Çıkış (Low) yap */
    P10DIR = 0xFF; P10OUT = 0x00;
    P11DIR = 0xFF; P11OUT = 0x00;
    PJDIR  = 0xFF; PJOUT  = 0x00;
}
