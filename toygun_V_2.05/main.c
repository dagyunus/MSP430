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

static void vout_ctrl_high(void){P4OUT |= VOUT_PWR_CTRL_PIN;} // GÜCÜ AÇ (ON)
    
static void vout_ctrl_low(void){P4OUT &= ~VOUT_PWR_CTRL_PIN;} // GÜCÜ KES (OFF)

static void fault_flag_clear(void){P4OUT |= EXT_FAULT_FLAG_PIN;}

static void fault_flag_active(void){P4OUT &= ~EXT_FAULT_FLAG_PIN;}

static uint8_t ext_on_requested(void){return ((P4IN & EXT_ON_OFF_CTRL_PIN) != 0U);}

static void update_vout_leds(uint8_t on_mode_active)
{
    if (on_mode_active)
        P7OUT |= FUNC_MODE_LED_PIN;
    else
        P7OUT &= ~FUNC_MODE_LED_PIN;

    if (P4OUT & VOUT_PWR_CTRL_PIN)
        P7OUT &= ~VOUT_OFF_LED_PIN;
    else
        P7OUT |= VOUT_OFF_LED_PIN;
}

/* Eşik Değerleri (Temsili, Kalibre Edilecek) */
#define LIMIT_UVLO_V          16.0f   // 16V Altı Kapat  // 0.762v
#define LIMIT_UVLO_REC_V      18.0f   // 18V Üstü Kurtar // 0,857v
#define LIMIT_OVP_V           32.0f   // 32V Üstü Kapat // 1,524v
#define LIMIT_OVP_REC_V       29.0f   // 29V Altı Kurtar // 1,381v
#define LIMIT_POWER_1000W     1000.0f
#define LIMIT_POWER_2000W     2000.0f
#define LIMIT_TEMP_OTP        85.0f
#define LIMIT_TEMP_REC        75.0f
#define VIN_FILTERED_SCALE    (28.0f / 1.3401f)
#define VIN_PROTECTED_SCALE   (28.0f / 2.1001f)

void Protection_Task(void)
{
    static uint32_t last_tick = 0;
    float v_in, v_protected, current, power, temp;
    uint8_t hw_fault_active;
    uint8_t on_mode_active;
    P3OUT |= FAN_POW_OFF_LED_PIN;
    /* Bu task 1 ms'de bir koşacak (g_update_timer 0.5ms'de bir artar) */
    if ((g_update_timer - last_tick) < 2) 
        return;
    last_tick = g_update_timer;

    /* 1. DONANIMSAL KESMELER (SIFIR TOLERANS) */
    if (g_sys_state != STATE_LOCKED) {
        if (((P8IN & LTC4364_FLT_PIN) == 0) || ((P8IN & TMP101_ALERT_PIN) == 0) ||((P4IN & TMP101_ALERT_ISO_PIN) == 0)) {
            if (g_sys_state != STATE_FAULT) {
                g_fault_strikes++;
                g_sys_state = (g_fault_strikes >= 3) ? STATE_LOCKED : STATE_FAULT;
            }
        }
    }

    /* 2. DİJİTAL FİLTRE (Moving Average - Sadece V_IN için) */
    /* Opamp transfer fonksiyonu: Vout = Vin * (11k / 231k) -> Vin = Vout * 21 */
    float raw_vin = adc_read_voltage(ADC_CH_28V_FLTRD_VMON) * VIN_FILTERED_SCALE;
    g_vin_sum -= g_vin_buffer[g_vin_idx];
    g_vin_buffer[g_vin_idx] = raw_vin;
    g_vin_sum += raw_vin;
    g_vin_idx = (g_vin_idx + 1) & (FILTER_SIZE - 1);
    v_in = g_vin_sum / (float)FILTER_SIZE;
    v_protected = adc_read_voltage(ADC_CH_28V_PROT_VMON) * VIN_PROTECTED_SCALE;
    hw_fault_active = ((((P8IN & LTC4364_FLT_PIN) == 0U) ||
                        ((P8IN & TMP101_ALERT_PIN) == 0U) ||
                        ((P4IN & TMP101_ALERT_ISO_PIN) == 0U)) ? 1U : 0U);

    /* 3. MODBUS TABLOSUNDAN 270V GÜÇ HESABI İÇİN VERİ ÇEKİMİ */
    /* Modbus'ta IEEE-754 Big-Endian olarak tutulan 2 registerı float'a çeviriyoruz */
    uint8_t *p_i = (uint8_t*)&current;
    p_i[3] = MB_RS422_Table.Regs[10] >> 8; p_i[2] = MB_RS422_Table.Regs[10] & 0xFF;
    p_i[1] = MB_RS422_Table.Regs[11] >> 8; p_i[0] = MB_RS422_Table.Regs[11] & 0xFF;

    float vout_270;
    uint8_t *p_v = (uint8_t*)&vout_270;
    p_v[3] = MB_RS422_Table.Regs[12] >> 8; p_v[2] = MB_RS422_Table.Regs[12] & 0xFF;
    p_v[1] = MB_RS422_Table.Regs[13] >> 8; p_v[0] = MB_RS422_Table.Regs[13] & 0xFF;

    /* DİKKAT: Burada akım ve voltaj sensör çarpanları (Kalibrasyon) eklenecektir */
    /* Veriler mb_rs422.c içinde zaten ölçeklendiği için direkt alıyoruz */
    float gercek_cikis_akimi = current;
    float gercek_cikis_voltaji = vout_270;

    power = gercek_cikis_voltaji * gercek_cikis_akimi;

    /* Sıcaklık (Modbus tablosundan son okunan değer alınır) */
    uint8_t *p_t = (uint8_t*)&temp;
    p_t[3] = MB_RS422_Table.Regs[6] >> 8; p_t[2] = MB_RS422_Table.Regs[6] & 0xFF;
    p_t[1] = MB_RS422_Table.Regs[7] >> 8; p_t[0] = MB_RS422_Table.Regs[7] & 0xFF;


    /* 4. STATE MACHINE MANTIĞI */
    switch (g_sys_state)
    {
        case STATE_INIT:
            /* Başlangıçta tüm çıkışlar kapalı */
            vout_ctrl_low();
            fault_flag_clear();
            g_sys_state = STATE_STANDBY;
            break;

        case STATE_STANDBY:
            vout_ctrl_low();

            /* Enable geldiyse girişler sağlıklı olduğunda start et */
            if (ext_on_requested() &&
                (v_in > LIMIT_UVLO_REC_V) && (v_in < LIMIT_OVP_REC_V) &&
                (v_protected > LIMIT_UVLO_REC_V) && (v_protected < LIMIT_OVP_REC_V) &&
                !hw_fault_active)
                g_sys_state = STATE_STARTING; 
            break;

        case STATE_STARTING:
            if (!ext_on_requested()) {
                vout_ctrl_low();
                g_sys_state = STATE_INIT;
                break;
            }

            /* Voltajlar ve limitler uygunsa normal çalışmaya geç */
            if (v_in > LIMIT_UVLO_REC_V && v_in < LIMIT_OVP_REC_V &&  v_protected > LIMIT_UVLO_REC_V && v_protected < LIMIT_OVP_REC_V && temp < LIMIT_TEMP_REC) {
                vout_ctrl_high();
                fault_flag_clear();
                g_sys_state = STATE_NORMAL;
            }
            break;

        case STATE_NORMAL:
            /* Dışarıdan kapatma emri geldi mi? */
            if (!ext_on_requested()) {
                vout_ctrl_low();
                g_sys_state = STATE_INIT;
                break;
            }
            vout_ctrl_high();

            /* Yazılımsal Limit Kontrolleri */
            if (v_in < LIMIT_UVLO_V || v_in > LIMIT_OVP_V || v_protected < LIMIT_UVLO_V || v_protected > LIMIT_OVP_V || temp > LIMIT_TEMP_OTP) {
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
                } 
                else 
                    clear_timer = 0;
            }
            break;

        case STATE_OVERLOAD_1000W:
            g_overload_ms++;
            if (power < LIMIT_POWER_1000W) 
                g_sys_state = STATE_NORMAL; /* İyileşti */
            else if (power > LIMIT_POWER_2000W || g_overload_ms > 1000) {
                g_fault_strikes++;
                g_sys_state = (g_fault_strikes >= 3) ? STATE_LOCKED : STATE_FAULT; /* 1 Saniyeyi aştı veya 2000W aştı */
            }
            break;

        case STATE_FAULT:
            /* GÜCÜ KES VE ALARM VER */
            vout_ctrl_low();
            fault_flag_active();
            
            if (ext_on_requested()) { /* Histeresis Kurtarması: Değerler normale dönerse Starting moduna dön */
                if (v_in > LIMIT_UVLO_REC_V && v_in < LIMIT_OVP_REC_V &&v_protected > LIMIT_UVLO_REC_V && v_protected < LIMIT_OVP_REC_V &&temp < LIMIT_TEMP_REC) {
                    /* Not: Donanım pinleri (LTC4364) hala düşükse buraya girmemeli! */
                    if ((P8IN & LTC4364_FLT_PIN) && (P8IN & TMP101_ALERT_PIN) && (P4IN & TMP101_ALERT_ISO_PIN)) 
                        g_sys_state = STATE_STARTING;    
                }
            } else {
                g_fault_strikes = 0; // Kullanıcı bilerek kapattıysa arıza hafızasını sil
                g_sys_state = STATE_STANDBY; 
            }
            break;

        case STATE_LOCKED:
            /* 3 KERE HATA YAPTI - KALICI KİLİT! (Sadece Manuel Kapatıp Açılırsa Düzelir) */
            vout_ctrl_low();
            fault_flag_active();
            
            /* Sadece kullanıcı pini OFF konumuna getirirse veya Modbus reset gelirse açılır */
            if (!ext_on_requested()) {
                g_fault_strikes = 0;
                g_sys_state = STATE_INIT;
            }
            break;
    }

    if ((g_sys_state == STATE_FAULT) || (g_sys_state == STATE_LOCKED))
    {
        vout_ctrl_low();
        fault_flag_active();
    }
    else
    {
        fault_flag_clear();
        if (ext_on_requested() &&
            ((g_sys_state == STATE_NORMAL) || (g_sys_state == STATE_OVERLOAD_1000W)))
            vout_ctrl_high();
        else
            vout_ctrl_low();
    }

    on_mode_active = ((((g_sys_state != STATE_FAULT) && (g_sys_state != STATE_LOCKED)) &&
                       (v_in > LIMIT_UVLO_REC_V) && (v_in < LIMIT_OVP_REC_V) &&
                       (v_protected > LIMIT_UVLO_REC_V) && (v_protected < LIMIT_OVP_REC_V) &&
                       (temp < LIMIT_TEMP_REC) &&
                       !hw_fault_active) ? 1U : 0U);

    update_vout_leds(on_mode_active);
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
        /* Watchdog'u besle */
        WDTCTL = WDTPW | WDTCNTCL | WDTSSEL_1 | WDTIS_4;
        /* Modbus ve Sensör İşlemleri */
        MB_RS422_Process();
        MB_RS422_UpdateTable(); 
        /* Sistem Korumalarını İşlet */
        Protection_Task();
    }
}
