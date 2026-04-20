#ifndef ADC_DRV_H
#define ADC_DRV_H

#include <msp430.h>
#include <stdint.h>

typedef enum
{
    ADC_CH_28V_FLTRD_VMON = 0,   /* A0 / P6.0 */
    ADC_CH_28V_PROT_IMON  = 1,   /* A1 / P6.1 */
    ADC_CH_28V_PROT_VMON  = 2,   /* A2 / P6.2 */
    ADC_CH_24V_FAN_IMON   = 3,   /* A3 / P6.3 */
    ADC_CH_24V_FAN_VMON   = 4    /* A4 / P6.4 */
} adc_channel_t;

void adc_init(void);
uint16_t adc_read_channel(adc_channel_t ch);
float adc_read_voltage(adc_channel_t ch);

#endif