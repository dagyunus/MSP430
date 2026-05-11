#include <msp430.h>
#include "include/hal_system.h"
#include "include/hal_rs422.h"
#include "include/mb_rs422.h"
#include "include/i2c_drv.h"
#include "include/adc_drv.h"
#include "include/gpio.h"

volatile uint32_t g_update_timer = 0;

/* --- SISTEM DURUM MAKINESI (STATE MACHINE) --- */
typedef enum {
    STATE_INIT = 0,
    STATE_STANDBY,
    STATE_STARTING,
    STATE_NORMAL,
    STATE_OVERLOAD_1000W,
    STATE_FAULT,
    STATE_LOCKED
} SystemState_t;

SystemState_t g_sys_state = STATE_INIT;

/* Histeresis, Latch ve Zamanlayıcı Değişkenleri */
uint16_t g_overload_ms = 0;
uint8_t  g_fault_strikes = 0;

/* Dijital Filtre (Moving Average) Değişkenleri */
#define FILTER_SIZE 8
float g_vin_buffer[FILTER_SIZE] = {0};
uint8_t g_vin_idx = 0;
float g_vin_sum = 0;

/* Eşik Değerleri (Temsili, Kalibre Edilecek) */
#define LIMIT_UVLO_V          16.0f   // 16V Altı Kapat  // 0.762v
#define LIMIT_UVLO_REC_V      18.0f   // 18V Üstü Kurtar // 0,857v
#define LIMIT_OVP_V           32.0f   // 32V Üstü Kapat // 1,524v
#define LIMIT_OVP_REC_V       29.0f   // 29V Altı Kurtar // 1,381v
#define LIMIT_POWER_1000W     1000.0f
#define LIMIT_POWER_2000W     2000.0f
#define LIMIT_TEMP_OTP        85.0f
#define LIMIT_TEMP_REC        75.0f

void Protection_Task(void)
{
    static uint32_t last_tick = 0;
    float v_in, current, power, temp;
    
    /* Bu task 1 ms'de bir koşacak (g_update_timer 0.5ms'de bir artar) */
    if ((g_update_timer - last_tick) < 2) return;
    last_tick = g_update_timer;

    /* 1. DONANIMSAL KESMELER (SIFIR TOLERANS) */
    if (g_sys_state != STATE_LOCKED) {
        if (((P8IN & LTC4364_FLT_PIN) == 0) || 
            ((P4IN & ADS1015_ALERT_RDY_PIN) == 0) || 
            ((P4IN & TMP101_ALERT_ISO_PIN) == 0)) 
        {
            if (g_sys_state != STATE_FAULT) {
                g_fault_strikes++;
                g_sys_state = (g_fault_strikes >= 3) ? STATE_LOCKED : STATE_FAULT;
            }
        }
    }

    /* 2. DİJİTAL FİLTRE (Moving Average - Sadece V_IN için) */
    /* Opamp transfer fonksiyonu: Vout = Vin * (11k / 231k) -> Vin = Vout * 21 */
    float raw_vin = adc_read_voltage(ADC_CH_28V_FLTRD_VMON) * 21.0f;
    g_vin_sum -= g_vin_buffer[g_vin_idx];
    g_vin_buffer[g_vin_idx] = raw_vin;
    g_vin_sum += raw_vin;
    g_vin_idx = (g_vin_idx + 1) & (FILTER_SIZE - 1);
    v_in = g_vin_sum / (float)FILTER_SIZE;

    /* 3. MODBUS TABLOSUNDAN 270V GÜÇ HESABI İÇİN VERİ ÇEKİMİ */
    /* Modbus'ta IEEE-754 Big-Endian olarak tutulan 2 registerı float'a çeviriyoruz */
    uint8_t *p_i = (uint8_t*)&current;
    p_i[3] = MB_RS422_Table.Regs[14] >> 8; p_i[2] = MB_RS422_Table.Regs[14] & 0xFF;
    p_i[1] = MB_RS422_Table.Regs[15] >> 8; p_i[0] = MB_RS422_Table.Regs[15] & 0xFF;

    float vout_270;
    uint8_t *p_v = (uint8_t*)&vout_270;
    p_v[3] = MB_RS422_Table.Regs[16] >> 8; p_v[2] = MB_RS422_Table.Regs[16] & 0xFF;
    p_v[1] = MB_RS422_Table.Regs[17] >> 8; p_v[0] = MB_RS422_Table.Regs[17] & 0xFF;

    /* DİKKAT: Burada akım ve voltaj sensör çarpanları (Kalibrasyon) eklenecektir */
    float gercek_cikis_akimi = current * 1.0f;   // Çarpanı buraya giriniz
    float gercek_cikis_voltaji = vout_270 * 1.0f; // Çarpanı buraya giriniz

    power = gercek_cikis_voltaji * gercek_cikis_akimi;

    /* Sıcaklık (Modbus tablosundan son okunan değer alınır) */
    uint8_t *p_t = (uint8_t*)&temp;
    p_t[3] = MB_RS422_Table.Regs[10] >> 8; p_t[2] = MB_RS422_Table.Regs[10] & 0xFF;
    p_t[1] = MB_RS422_Table.Regs[11] >> 8; p_t[0] = MB_RS422_Table.Regs[11] & 0xFF;


    /* 4. STATE MACHINE MANTIĞI */
    switch (g_sys_state)
    {
        case STATE_INIT:
            /* Başlangıçta tüm çıkışlar kapalı */
            P4OUT &= ~VOUT_PWR_CTRL_PIN;
            P4OUT &= ~EXT_FAULT_FLAG_PIN;
            P7OUT &= ~ON_MODE_LED_PIN;
            P7OUT |= VOUT_OFF_LED_PIN;
            
            g_sys_state = STATE_STANDBY;
            break;

        case STATE_STANDBY:
            /* Dışarıdan ON/OFF kontrolü: EXT_ON_OFF_CTRL_PIN HIGH ise aç */
            if (P4IN & EXT_ON_OFF_CTRL_PIN) {
                g_sys_state = STATE_STARTING;
            }
            break;

        case STATE_STARTING:
            /* Voltajlar ve limitler uygunsa normal çalışmaya geç */
            if (v_in > LIMIT_UVLO_REC_V && v_in < LIMIT_OVP_REC_V && temp < LIMIT_TEMP_REC) {
                P4OUT |= VOUT_PWR_CTRL_PIN;   // Gücü Aç
                P7OUT |= ON_MODE_LED_PIN;     // LED'i Yak
                P7OUT &= ~VOUT_OFF_LED_PIN;
                P4OUT &= ~EXT_FAULT_FLAG_PIN; // Hata bayrağını indir
                g_sys_state = STATE_NORMAL;
            }
            break;

        case STATE_NORMAL:
            /* Dışarıdan kapatma emri geldi mi? */
            if ((P4IN & EXT_ON_OFF_CTRL_PIN) == 0) {
                g_sys_state = STATE_INIT;
                break;
            }

            /* Yazılımsal Limit Kontrolleri */
            if (v_in < LIMIT_UVLO_V || v_in > LIMIT_OVP_V || temp > LIMIT_TEMP_OTP) {
                g_fault_strikes++;
                g_sys_state = (g_fault_strikes >= 3) ? STATE_LOCKED : STATE_FAULT;
            }
            else if (power > LIMIT_POWER_2000W) {
                g_fault_strikes++;
                g_sys_state = (g_fault_strikes >= 3) ? STATE_LOCKED : STATE_FAULT; /* Anında Kes */
            }
            else if (power > LIMIT_POWER_1000W) {
                g_sys_state = STATE_OVERLOAD_1000W;
                g_overload_ms = 0;
            }
            else {
                /* Her şey yolunda çalışmaya devam ediyorsa hata hafızasını temizle */
                static uint16_t clear_timer = 0;
                if (g_fault_strikes > 0) {
                    if (++clear_timer >= 60000) { // Her 1ms'de bir artar (60 saniye)
                        g_fault_strikes = 0;
                        clear_timer = 0;
                    }
                } else {
                    clear_timer = 0;
                }
            }
            break;

        case STATE_OVERLOAD_1000W:
            g_overload_ms++;
            if (power < LIMIT_POWER_1000W) {
                g_sys_state = STATE_NORMAL; /* İyileşti */
            }
            else if (power > LIMIT_POWER_2000W || g_overload_ms > 1000) {
                g_fault_strikes++;
                g_sys_state = (g_fault_strikes >= 3) ? STATE_LOCKED : STATE_FAULT; /* 1 Saniyeyi aştı veya 2000W aştı */
            }
            break;

        case STATE_FAULT:
            /* GÜCÜ KES VE ALARM VER */
            P4OUT &= ~VOUT_PWR_CTRL_PIN;
            P4OUT |= EXT_FAULT_FLAG_PIN;
            P7OUT &= ~ON_MODE_LED_PIN;
            P7OUT |= VOUT_OFF_LED_PIN;
            
            /* Histeresis Kurtarması: Değerler normale dönerse Starting moduna dön */
            if ((P4IN & EXT_ON_OFF_CTRL_PIN) != 0) {
                if (v_in > LIMIT_UVLO_REC_V && v_in < LIMIT_OVP_REC_V && temp < LIMIT_TEMP_REC) {
                    /* Not: Donanım pinleri (LTC4364) hala düşükse buraya girmemeli! */
                    if ((P8IN & LTC4364_FLT_PIN) && (P4IN & ADS1015_ALERT_RDY_PIN) && (P4IN & TMP101_ALERT_ISO_PIN)) {
                        g_sys_state = STATE_STARTING;
                    }
                }
            } else {
                g_fault_strikes = 0; // Kullanıcı bilerek kapattıysa arıza hafızasını sil
                g_sys_state = STATE_STANDBY; 
            }
            break;

        case STATE_LOCKED:
            /* 3 KERE HATA YAPTI - KALICI KİLİT! (Sadece Manuel Kapatıp Açılırsa Düzelir) */
            P4OUT &= ~VOUT_PWR_CTRL_PIN;
            P4OUT |= EXT_FAULT_FLAG_PIN;
            P7OUT &= ~ON_MODE_LED_PIN;
            P7OUT |= VOUT_OFF_LED_PIN;
            
            /* Sadece kullanıcı pini OFF konumuna getirirse veya Modbus reset gelirse açılır */
            if ((P4IN & EXT_ON_OFF_CTRL_PIN) == 0) {
                g_fault_strikes = 0;
                g_sys_state = STATE_INIT;
            }
            break;
    }
}

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

        /* Sistem Korumalarını ve State Machine'i İşlet (Deneme için geçici olarak kapatıldı) */
        Protection_Task();
    }
}
