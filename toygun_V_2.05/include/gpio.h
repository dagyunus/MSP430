#ifndef DRIVER_GPIO_H_
#define DRIVER_GPIO_H_

#include <msp430.h>
#include <stdint.h>
/* RS422 Control Pins */
#define RS422_RE_PIN    BIT6    // P9.6 -> RS422_RE#
#define RS422_DE_PIN    BIT7    // P9.7 -> RS422_DE

/* RS422 UART Pins */
#define RS422_TX_PIN    BIT4    // P9.4 -> UCA2TXD
#define RS422_RX_PIN    BIT5    // P9.5 -> UCA2RXD

/* I2C UCB0 Pins */
#define I2C_UCB0_SDA    BIT1    // P3.1
#define I2C_UCB0_SCL    BIT2    // P3.2

/* I2C UCB2 Pins */
#define I2C_UCB2_SDA    BIT1    // P9.1
#define I2C_UCB2_SCL    BIT2    // P9.2

/* Port 3 Pins */
#define FAN_POW_OFF_LED_PIN     BIT6    // P3.6 (Output)
#define FAN_SPEED_CTRL_PIN      BIT7    // P3.7 (Output)

/* Port 4 Pins */
#define TMP101_ALERT_ISO_PIN    BIT0    // P4.0 (Input)
#define FAN_SENSOR_PIN          BIT1    // P4.1 (Input)
#define ADS1015_ALERT_RDY_PIN   BIT2    // P4.2 (Input)
#define FAN_PWR_CTRL_PIN        BIT3    // P4.3 (Output)
#define VOUT_PWR_CTRL_PIN       BIT4    // P4.4 (Output)
#define EXT_ON_OFF_CTRL_PIN     BIT5    // P4.5 (Input)
#define EXT_FAULT_FLAG_PIN      BIT6    // P4.6 (Output)
#define FORCE_LTC4364_UV_PIN    BIT7    // P4.7 (Output)

/* Port 7 Pins */
#define FORCE_LTC4364_SHDN_PIN  BIT3    // P7.3 (Output)
#define ON_MODE_LED_PIN         BIT4    // P7.4 (Output)
#define VOUT_OFF_LED_PIN        BIT5    // P7.5 (Output)
#define FUNC_MODE_LED_PIN       BIT6    // P7.6 (Output)

/* Port 8 Pins */
#define LTC4364_ENOUT_CTRL_PIN  BIT1    // P8.1 (Input)
#define FORCE_LTC4364_OV_PIN    BIT2    // P8.2 (Output)
#define ETR_ALARM_PIN           BIT4    // P8.4 (Input)
#define TMP101_ALERT_PIN        BIT5    // P8.5 (Input)
#define LTC4364_FLT_PIN         BIT6    // P8.6 (Input)

/* Internal ADC Pins (P6.0 - P6.4) */
#define ADC_28V_FLTRD_VMON      BIT0
#define ADC_28V_PROTECTED_IMON  BIT1
#define ADC_28V_PROTECTED_VMON  BIT2
#define ADC_24V_FAN_IMON        BIT3
#define ADC_24V_FAN_VMON        BIT4

/* 
 * RS422 Mode Control Macros
 * Full-duplex RS422:
 * DE  = 1 -> TX driver active
 * RE# = 0 -> RX receiver active
 */
#define RS422_TX_RX_ENABLE()   do {                  \
                                  P9OUT |=  RS422_DE_PIN;   \
                                  P9OUT &= ~RS422_RE_PIN;   \
                              } while(0)

#define RS422_TX_ENABLE()      (P9OUT |=  RS422_DE_PIN)
#define RS422_TX_DISABLE()     (P9OUT &= ~RS422_DE_PIN)
#define RS422_RX_ENABLE()      (P9OUT &= ~RS422_RE_PIN)
#define RS422_RX_DISABLE()     (P9OUT |=  RS422_RE_PIN)

/**
 * @brief Initializes all GPIO pins to their default states and modes.
 */
void gpio_init(void);

#endif /* DRIVER_GPIO_H_ */
