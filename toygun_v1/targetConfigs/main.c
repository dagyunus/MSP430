#include <msp430.h>
#include <stdint.h>
#include "i2c_drv.h"
#include "tmp101.h"
#include "ads1015.h"
#include "ds1682.h"
#include "adc_drv.h"

/* I2C addresses */
#define TMP101_INPUT_ADDR   0x48
#define TMP101_ISO_ADDR     0x4A
#define ADS1015_ADDR        0x48
#define DS1682_ADDR         0x6B

/* Debug globals */
volatile float g_temp_input_c = 0.0f;
volatile float g_temp_iso_c   = 0.0f;

volatile uint16_t g_ads_ain0_raw = 0;
volatile uint16_t g_ads_ain1_raw = 0;
volatile float    g_ads_ain0_v   = 0.0f;
volatile float    g_ads_ain1_v   = 0.0f;

volatile uint32_t g_qsec    = 0;
volatile uint32_t g_seconds = 0;

/* Internal ADC voltages only */
volatile float g_v_28v_fltrd_vmon = 0.0f;
volatile float g_v_28v_prot_imon  = 0.0f;
volatile float g_v_28v_prot_vmon  = 0.0f;
volatile float g_v_24v_fan_imon   = 0.0f;
volatile float g_v_24v_fan_vmon   = 0.0f;

/* Error flags */
volatile i2c_status_t g_err_tmp_input;
volatile i2c_status_t g_err_tmp_iso;
volatile i2c_status_t g_err_ads_ain0;
volatile i2c_status_t g_err_ads_ain1;
volatile i2c_status_t g_err_ds1682;

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    i2c_init_ucb0();
    i2c_init_ucb2();
    adc_init();

    g_err_tmp_input = tmp101_init(&g_i2c_ucb2, TMP101_INPUT_ADDR);
    g_err_tmp_iso   = tmp101_init(&g_i2c_ucb0, TMP101_ISO_ADDR);

    while (1)
    {
        /* TMP101 */
        g_err_tmp_input = tmp101_read_temp(&g_i2c_ucb2, TMP101_INPUT_ADDR, &g_temp_input_c);
        g_err_tmp_iso   = tmp101_read_temp(&g_i2c_ucb0, TMP101_ISO_ADDR, &g_temp_iso_c);

        /* ADS1015 */
        g_err_ads_ain0 = ads1015_read_channel(&g_i2c_ucb0, ADS1015_ADDR, 0, &g_ads_ain0_raw, &g_ads_ain0_v);
        g_err_ads_ain1 = ads1015_read_channel(&g_i2c_ucb0, ADS1015_ADDR, 1, &g_ads_ain1_raw, &g_ads_ain1_v);

        /* DS1682 */
        g_err_ds1682 = ds1682_read_etc(&g_i2c_ucb2, DS1682_ADDR, &g_qsec);
        if (g_err_ds1682 == I2C_OK)
        {
            g_seconds = g_qsec / 4UL;
        }

        /* Internal ADC - direct voltage read */
        g_v_28v_fltrd_vmon = adc_read_voltage(ADC_CH_28V_FLTRD_VMON);
        g_v_28v_prot_imon  = adc_read_voltage(ADC_CH_28V_PROT_IMON);
        g_v_28v_prot_vmon  = adc_read_voltage(ADC_CH_28V_PROT_VMON);
        g_v_24v_fan_imon   = adc_read_voltage(ADC_CH_24V_FAN_IMON);
        g_v_24v_fan_vmon   = adc_read_voltage(ADC_CH_24V_FAN_VMON);

        __no_operation();
        __delay_cycles(50000);
    }
}