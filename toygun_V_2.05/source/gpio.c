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
    
    /* Port 7: LED'ler ve Kontrol Pinleri */
    P7SEL &= ~(BIT3 | BIT4 | BIT5 | BIT6); // Dijital I/O modu seçildi
    P7OUT &= ~(BIT3 | BIT4 | BIT5 | BIT6); 
    P7DIR |= (BIT3 | BIT4 | BIT5 | BIT6);  // Çıkış yapıldı
    
    /* Port 8: FORCE_LTC4364_OV_PIN (P8.2) Çıkış (LOW) olmalı! */
    P8DIR = 0x8D; // 1000 1101 (Bit 2 Çıkış)
    P8OUT = 0x00; // P8.2 LOW yapıldı!

    
    /* Port 9: I2C ve UART Pinleri */
    P9SEL |= (BIT1 | BIT2); // I2C SDA/SCL
    P9DIR = 0xF9; // 1111 1001
    P9OUT = 0x00;
    
    /* Port 4 Özel Ayarlar: VOUT_PWR_CTRL (P4.4), FAN_PWR_CTRL (P4.3) ve Kontrol Pinleri */
    P4SEL &= ~(VOUT_PWR_CTRL_PIN | FAN_PWR_CTRL_PIN | EXT_ON_OFF_CTRL_PIN);
    P4DIR |= (VOUT_PWR_CTRL_PIN | FAN_PWR_CTRL_PIN);
    P4DIR &= ~EXT_ON_OFF_CTRL_PIN; // P4.5 Giriş (ON/OFF Anahtarı)
    P4REN |= EXT_ON_OFF_CTRL_PIN;  // Pull-up/down direnci aktif
    P4OUT |= EXT_ON_OFF_CTRL_PIN;  // Pull-up seçildi (Boşta HIGH/OFF kalsın)

    /* Başlangıç LED ve Güç Durumları */
    P7OUT |= VOUT_OFF_LED_PIN;     // Başlangıçta çıkış kapalı LED'i aktif
    P4OUT &= ~VOUT_PWR_CTRL_PIN;   // Başlangıçta Çıkış KAPALI (LOW)
    P4OUT &= ~FAN_PWR_CTRL_PIN;    // Fan Gücü AÇIK (LOW)
    P4OUT |= EXT_FAULT_FLAG_PIN;   // Hata Bayrağı Pasif (HIGH)
    P3OUT |= FAN_POW_OFF_LED_PIN;  // P3.6 LED'i

    /* ULP 4.1: Kullanılmayan pinleri Çıkış (Low) yap */
    P10DIR = 0xFF; P10OUT = 0x00;
    P11DIR = 0xFF; P11OUT = 0x00;
    PJDIR  = 0xFF; PJOUT  = 0x00;
}
