#include <msp430.h>
#include "include/hal_system.h"
#include "include/hal_rs422.h"
#include "include/mb_rs422.h"
#include "include/i2c_drv.h"
#include "include/adc_drv.h"
#include "include/gpio.h"

volatile uint32_t g_update_timer = 0;

int main(void)
{
    /* Watchdog'u Kapatmak yerine 1 Saniyelik Koruma Modunda (ACLK üzerinden) Açıyoruz */
    /* WDTPW: Parola, WDTCNTCL: Sayacı sıfırla, WDTSSEL_1: ACLK, WDTIS_4: 1 saniye */
    WDTCTL = WDTPW | WDTCNTCL | WDTSSEL_1 | WDTIS_4;

    HAL_System_ClockInit();
    HAL_System_TimerInit();
    gpio_init();
    HAL_RS422_Init();
    i2c_init_ucb0();
    i2c_init_ucb2();
    adc_init();
    
    MB_RS422_Init();

    /* Warm-up sensors (Configuration) */
    i2c_write_reg8(&g_i2c_ucb2, TMP101_INPUT_ADDR, TMP101_REG_CONFIG, 0x60);
    i2c_write_reg8(&g_i2c_ucb0, TMP101_ISO_ADDR,   TMP101_REG_CONFIG, 0x60);

    __bis_SR_register(GIE);

    while (1)
    {
        /* Watchdog'u sürekli besle (Eğer kod buralara gelemezse cihaz kendini resetler) */
        WDTCTL = WDTPW | WDTCNTCL | WDTSSEL_1 | WDTIS_4;

        /* Modbus Process - Her zaman öncelikli */
        MB_RS422_Process();

        /* Sensör durum makinesi artık kendi içinde bekleme yapıyor, sürekli çağrılabilir */
        MB_RS422_UpdateTable(); 

        /* I2C Sürücüsünün olası kilitlenmelere karşı Timeout sayacını işlet */
        i2c_service_tick();
    }
}
