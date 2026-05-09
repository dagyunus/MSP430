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
    P3OUT = 0x00;
    
    /* Port 4: Girişler: P4.0, 4.1, 4.2, 4.5. Diğerleri Çıkış. */
    P4DIR = 0xD8; // 1101 1000
    P4OUT = 0x00;
    
    /* Port 5: Hepsi Çıkış */
    P5DIR = 0xFF; P5OUT = 0x00;
    
    /* Port 6: ADC Analog Modu (Hepsi Giriş) */
    P6DIR = 0x00;
    P6SEL = 0xFF;
    
    /* Port 7: Hepsi Çıkış */
    P7DIR = 0xFF; P7OUT = 0x00;
    
    /* Port 8: Girişler: P8.1, P8.4 (ETR ALARM), P8.5, P8.6. Diğerleri Çıkış. */
    /* LED'i söndürmek için P8.4'ü Giriş yapmalıyız. */
    P8DIR = 0x8D; // 1000 1101 (BIT1,4,5,6 Giriş)
    P8OUT = 0x00;
    
    /* Port 9: I2C ve UART Pinleri */
    P9SEL |= (BIT1 | BIT2); // I2C SDA/SCL
    P9DIR = 0xF9; // 1111 1001
    P9OUT = 0x00;
    
    /* Başlangıç LED ve Güç Durumları */
    P7OUT |= ON_MODE_LED_PIN; 
    P4OUT |= VOUT_PWR_CTRL_PIN;
    P4OUT |= FAN_PWR_CTRL_PIN;

    /* ULP 4.1: Port E (P9/P10) ve Port F (P11/PJ) kullanılmayan pinleri Çıkış (Low) yap */
    P10DIR = 0xFF; P10OUT = 0x00;
    P11DIR = 0xFF; P11OUT = 0x00;
    PJDIR  = 0xFF; PJOUT  = 0x00;
}
