#include "../include/adc_drv.h"

#define ADC_VREF_VOLTS   (3.3f)
#define ADC_MAX_COUNTS   (4095.0f)

static uint8_t adc_channel_to_inch(adc_channel_t ch)
{
    switch (ch)
    {
        case ADC_CH_28V_FLTRD_VMON: return ADC12INCH_0;
        case ADC_CH_28V_PROT_IMON:  return ADC12INCH_1;
        case ADC_CH_28V_PROT_VMON:  return ADC12INCH_2;
        case ADC_CH_24V_FAN_IMON:   return ADC12INCH_3;
        case ADC_CH_24V_FAN_VMON:   return ADC12INCH_4;
        default:                    return ADC12INCH_0;
    }
}

void adc_init(void)
{
    /* P6.0..P6.4 analog input are configured in gpio_init() */

    ADC12CTL0 = 0;
    ADC12CTL1 = 0;
    ADC12CTL2 = 0;

    /* ADC on + sample hold time */
    ADC12CTL0 = ADC12ON | ADC12SHT0_8;

    /* single channel, single conversion */
    ADC12CTL1 = ADC12SHP;

    /* 12-bit resolution */
    ADC12CTL2 = ADC12RES_2;

    ADC12MCTL0 = ADC12INCH_0;

    ADC12CTL0 |= ADC12ENC;
}

uint16_t adc_read_channel(adc_channel_t ch)
{
    uint8_t inch;

    ADC12CTL0 &= ~ADC12ENC;
    inch = adc_channel_to_inch(ch);
    ADC12MCTL0 = inch;
    ADC12CTL0 |= ADC12ENC;
    ADC12CTL0 |= ADC12SC;

    /* Polling with timeout and NOP to satisfy compiler */
    volatile uint16_t timeout = 10000U;
    while ((ADC12CTL1 & ADC12BUSY) && (--timeout > 0))
    {
        __no_operation();
    }

    return ADC12MEM0;
}

float adc_read_voltage(adc_channel_t ch)
{
    uint16_t raw;

    raw = adc_read_channel(ch);
    /* 3.3 / 4095.0 = 0.00080586f approx */
    return (float)raw * 0.00080586f;
}