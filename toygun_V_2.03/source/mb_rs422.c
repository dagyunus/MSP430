#include <msp430.h>
#include <stdint.h>
#include "../include/mb_rs422.h"
#include "../include/hal_rs422.h"
#include "../include/hal_system.h"
#include "../include/i2c_drv.h"
#include "../include/adc_drv.h"
#include "../include/gpio.h"

extern const I2C_Bus g_i2c_ucb0;
extern const I2C_Bus g_i2c_ucb2;

MB_RS422_Table_t MB_RS422_Table;

static uint16_t get_CRC16(const uint8_t *ptr, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i = len, j;
    while(i--) {
        crc ^= (uint16_t)(*ptr++);
        j = 8;
        while(j--) {
            if (crc & 0x0001U) { crc >>= 1; crc ^= 0xA001U; }
            else { crc >>= 1; }
        }
    }
    return crc;
}

void MB_RS422_Init(void)
{
    uint16_t i = MB_RS422_REG_COUNT;
    while (i--) { MB_RS422_Table.Regs[i] = 0; }
}

/* I2C Okuma State Machine Durumları */
typedef enum {
    S_ADC_READ = 0,
    S_TMP_IN_REQ, S_TMP_IN_WAIT,
    S_TMP_ISO_REQ, S_TMP_ISO_WAIT,
    S_ADS1_CFG_REQ, S_ADS1_CFG_WAIT, S_ADS1_RD_REQ, S_ADS1_RD_WAIT,
    S_ADS2_CFG_REQ, S_ADS2_CFG_WAIT, S_ADS2_RD_REQ, S_ADS2_RD_WAIT,
    S_DS1682_REQ, S_DS1682_WAIT,
    S_DONE
} SensorState_t;

extern volatile uint32_t g_update_timer;

/* Bu fonksiyon artık main loop içerisinde sürekli çağrılacak (engelleyici değil) */
void MB_RS422_UpdateTable(void)
{
    static SensorState_t g_sns_state = S_ADC_READ;
    static uint32_t g_sns_timer = 0;

    uint8_t buf[4];
    uint8_t rx_len;
    float f_val;
    uint8_t *p_f = (uint8_t*)&f_val;
    uint16_t idx, cfg;
    uint16_t adc_vals[5];
    int16_t raw_s;
    i2c_status_t st;

    /* Status Register Sürekli Güncellenir */
    MB_RS422_Table.Data.Status.All = 0;
    if (P4OUT & FORCE_LTC4364_UV_PIN)  MB_RS422_Table.Data.Status.Bits.Force_UV  = 1;
    if (P8OUT & FORCE_LTC4364_OV_PIN)  MB_RS422_Table.Data.Status.Bits.Force_OV  = 1;
    if (P4OUT & VOUT_PWR_CTRL_PIN)     MB_RS422_Table.Data.Status.Bits.VOut_Ctrl = 1;
    if (P4OUT & EXT_FAULT_FLAG_PIN)    MB_RS422_Table.Data.Status.Bits.Ext_Fault = 1;

    switch(g_sns_state) {
        case S_ADC_READ:
            if (g_update_timer >= 2000) { /* 1 Saniyede bir sensör döngüsünü başlat (0.5ms * 2000) */
                g_update_timer = 0;
                MB_RS422_Table.Data.Diag.All = 0; /* Hataları sıfırla */
                
                adc_vals[0] = adc_read_channel(ADC_CH_28V_FLTRD_VMON);
                adc_vals[1] = adc_read_channel(ADC_CH_28V_PROT_IMON);
                adc_vals[2] = adc_read_channel(ADC_CH_28V_PROT_VMON);
                adc_vals[3] = adc_read_channel(ADC_CH_24V_FAN_IMON);
                adc_vals[4] = adc_read_channel(ADC_CH_24V_FAN_VMON);

                idx = 5;
                while(idx--) {
                    f_val = (float)adc_vals[idx] * 0.00080586f;
                    MB_RS422_Table.Regs[idx*2] = ((uint16_t)p_f[3] << 8) | p_f[2];
                    MB_RS422_Table.Regs[idx*2+1] = ((uint16_t)p_f[1] << 8) | p_f[0];
                }
                g_sns_state = S_TMP_IN_REQ;
            }
            break;

        case S_TMP_IN_REQ:
            if (i2c_async_read_bytes(&g_i2c_ucb2, TMP101_INPUT_ADDR, TMP101_REG_TEMP, 2) == I2C_OK)
                g_sns_state = S_TMP_IN_WAIT;
            else { MB_RS422_Table.Data.Diag.Bits.TMP_In_Fail = 1; g_sns_state = S_TMP_ISO_REQ; }
            break;

        case S_TMP_IN_WAIT:
            st = i2c_async_poll(&g_i2c_ucb2, buf, &rx_len);
            if (st == I2C_OK && rx_len == 2) {
                raw_s = (int16_t)((((uint16_t)buf[0] << 8) | buf[1]) >> 4);
                if (raw_s & 0x0800) raw_s |= (int16_t)0xF000;
                f_val = (float)raw_s * 0.0625f;
                MB_RS422_Table.Regs[10] = ((uint16_t)p_f[3] << 8) | p_f[2];
                MB_RS422_Table.Regs[11] = ((uint16_t)p_f[1] << 8) | p_f[0];
                g_sns_state = S_TMP_ISO_REQ;
            } else if (st != I2C_ERR_BUSY) {
                MB_RS422_Table.Data.Diag.Bits.TMP_In_Fail = 1; g_sns_state = S_TMP_ISO_REQ;
            }
            break;

        case S_TMP_ISO_REQ:
            if (i2c_async_read_bytes(&g_i2c_ucb0, TMP101_ISO_ADDR, TMP101_REG_TEMP, 2) == I2C_OK)
                g_sns_state = S_TMP_ISO_WAIT;
            else { MB_RS422_Table.Data.Diag.Bits.TMP_Iso_Fail = 1; g_sns_state = S_ADS1_CFG_REQ; }
            break;

        case S_TMP_ISO_WAIT:
            st = i2c_async_poll(&g_i2c_ucb0, buf, &rx_len);
            if (st == I2C_OK && rx_len == 2) {
                raw_s = (int16_t)((((uint16_t)buf[0] << 8) | buf[1]) >> 4);
                if (raw_s & 0x0800) raw_s |= (int16_t)0xF000;
                f_val = (float)raw_s * 0.0625f;
                MB_RS422_Table.Regs[12] = ((uint16_t)p_f[3] << 8) | p_f[2];
                MB_RS422_Table.Regs[13] = ((uint16_t)p_f[1] << 8) | p_f[0];
                g_sns_state = S_ADS1_CFG_REQ;
            } else if (st != I2C_ERR_BUSY) {
                MB_RS422_Table.Data.Diag.Bits.TMP_Iso_Fail = 1; g_sns_state = S_ADS1_CFG_REQ;
            }
            break;

        case S_ADS1_CFG_REQ:
            cfg = 0xC383; buf[0] = (uint8_t)(cfg >> 8); buf[1] = (uint8_t)(cfg & 0xFF);
            if (i2c_async_write_reg_multi(&g_i2c_ucb0, ADS1015_ADDR, ADS1015_REG_CONFIG, buf, 2) == I2C_OK)
                g_sns_state = S_ADS1_CFG_WAIT;
            else { MB_RS422_Table.Data.Diag.Bits.ADS_Fail = 1; g_sns_state = S_ADS2_CFG_REQ; }
            break;

        case S_ADS1_CFG_WAIT:
            st = i2c_async_poll(&g_i2c_ucb0, 0, &rx_len);
            if (st == I2C_OK) { g_sns_timer = 15; g_sns_state = S_ADS1_RD_REQ; } // Dönüşüm bekleme
            else if (st != I2C_ERR_BUSY) { MB_RS422_Table.Data.Diag.Bits.ADS_Fail = 1; g_sns_state = S_ADS2_CFG_REQ; }
            break;

        case S_ADS1_RD_REQ:
            if (g_sns_timer) { g_sns_timer--; break; } // Bloklamadan 15 döngü (yaklaşık 7.5ms) bekle
            if (i2c_async_read_bytes(&g_i2c_ucb0, ADS1015_ADDR, ADS1015_REG_CONV, 2) == I2C_OK)
                g_sns_state = S_ADS1_RD_WAIT;
            else { MB_RS422_Table.Data.Diag.Bits.ADS_Fail = 1; g_sns_state = S_ADS2_CFG_REQ; }
            break;

        case S_ADS1_RD_WAIT:
            st = i2c_async_poll(&g_i2c_ucb0, buf, &rx_len);
            if (st == I2C_OK && rx_len == 2) {
                raw_s = ((((uint16_t)buf[0] << 8) | buf[1]) >> 4) & 0x0FFF;
                f_val = (float)raw_s * 0.002f;
                MB_RS422_Table.Regs[14] = ((uint16_t)p_f[3] << 8) | p_f[2];
                MB_RS422_Table.Regs[15] = ((uint16_t)p_f[1] << 8) | p_f[0];
                g_sns_state = S_ADS2_CFG_REQ;
            } else if (st != I2C_ERR_BUSY) {
                MB_RS422_Table.Data.Diag.Bits.ADS_Fail = 1; g_sns_state = S_ADS2_CFG_REQ;
            }
            break;

        case S_ADS2_CFG_REQ:
            cfg = 0xD383; buf[0] = (uint8_t)(cfg >> 8); buf[1] = (uint8_t)(cfg & 0xFF);
            if (i2c_async_write_reg_multi(&g_i2c_ucb0, ADS1015_ADDR, ADS1015_REG_CONFIG, buf, 2) == I2C_OK)
                g_sns_state = S_ADS2_CFG_WAIT;
            else { MB_RS422_Table.Data.Diag.Bits.ADS_Fail = 1; g_sns_state = S_DS1682_REQ; }
            break;

        case S_ADS2_CFG_WAIT:
            st = i2c_async_poll(&g_i2c_ucb0, 0, &rx_len);
            if (st == I2C_OK) { g_sns_timer = 15; g_sns_state = S_ADS2_RD_REQ; }
            else if (st != I2C_ERR_BUSY) { MB_RS422_Table.Data.Diag.Bits.ADS_Fail = 1; g_sns_state = S_DS1682_REQ; }
            break;

        case S_ADS2_RD_REQ:
            if (g_sns_timer) { g_sns_timer--; break; }
            if (i2c_async_read_bytes(&g_i2c_ucb0, ADS1015_ADDR, ADS1015_REG_CONV, 2) == I2C_OK)
                g_sns_state = S_ADS2_RD_WAIT;
            else { MB_RS422_Table.Data.Diag.Bits.ADS_Fail = 1; g_sns_state = S_DS1682_REQ; }
            break;

        case S_ADS2_RD_WAIT:
            st = i2c_async_poll(&g_i2c_ucb0, buf, &rx_len);
            if (st == I2C_OK && rx_len == 2) {
                raw_s = ((((uint16_t)buf[0] << 8) | buf[1]) >> 4) & 0x0FFF;
                f_val = (float)raw_s * 0.002f;
                MB_RS422_Table.Regs[16] = ((uint16_t)p_f[3] << 8) | p_f[2];
                MB_RS422_Table.Regs[17] = ((uint16_t)p_f[1] << 8) | p_f[0];
                g_sns_state = S_DS1682_REQ;
            } else if (st != I2C_ERR_BUSY) {
                MB_RS422_Table.Data.Diag.Bits.ADS_Fail = 1; g_sns_state = S_DS1682_REQ;
            }
            break;

        case S_DS1682_REQ:
            if (i2c_async_read_bytes(&g_i2c_ucb2, DS1682_ADDR, 0x01, 4) == I2C_OK) // 0x01 = DS1682_REG_ETC
                g_sns_state = S_DS1682_WAIT;
            else { MB_RS422_Table.Data.Diag.Bits.DS1682_Fail = 1; g_sns_state = S_ADC_READ; }
            break;

        case S_DS1682_WAIT:
            st = i2c_async_poll(&g_i2c_ucb2, buf, &rx_len);
            if (st == I2C_OK && rx_len == 4) {
                MB_RS422_Table.Regs[20] = ((uint16_t)buf[3] << 8) | buf[2];
                MB_RS422_Table.Regs[21] = ((uint16_t)buf[1] << 8) | buf[0];
                g_sns_state = S_ADC_READ; /* Döngüyü başa sar */
            } else if (st != I2C_ERR_BUSY) {
                MB_RS422_Table.Data.Diag.Bits.DS1682_Fail = 1; g_sns_state = S_ADC_READ;
            }
            break;
            
        default:
            g_sns_state = S_ADC_READ;
            break;
    }
}

void MB_RS422_Process(void)
{
    uint8_t rx_buf[RS422_BUFFER_SIZE], out_idx;
    uint16_t rx_len = 0, addr, qty, val, crc_calc, crc_rx, i;

    /* Timeout eşiğini 4 yaptık. Timer her 0.5ms'de artıyor. 
     * 4 * 0.5ms = 2.0ms (115200 baud Modbus standardı T3.5 ~1.75ms'dir, 2.0ms mükemmeldir) */
    if (HAL_RS422_TickRxIdle(rx_buf, &rx_len, RS422_BUFFER_SIZE, 4) == 0) {
        if (rx_len < 8 || rx_buf[0] != MB_RS422_DEVICE_ID) return;
        crc_calc = get_CRC16(rx_buf, rx_len - 2);
        crc_rx   = ((uint16_t)rx_buf[rx_len - 1] << 8) | rx_buf[rx_len - 2];
        if (crc_calc != crc_rx) return;

        if (rx_buf[1] == 0x03) {
            addr = ((uint16_t)rx_buf[2] << 8) | rx_buf[3];
            qty  = ((uint16_t)rx_buf[4] << 8) | rx_buf[5];
            if ((addr + qty) > MB_RS422_REG_COUNT) return;

            RS422_TX_ENABLE();
            RS422.SentData.Data[0] = MB_RS422_DEVICE_ID;
            RS422.SentData.Data[1] = 0x03;
            RS422.SentData.Data[2] = (uint8_t)(qty * 2);
            out_idx = 3;
            i = 0;
            while(i < qty) {
                uint16_t r_v = MB_RS422_Table.Regs[addr + i];
                RS422.SentData.Data[out_idx++] = (uint8_t)(r_v >> 8);
                RS422.SentData.Data[out_idx++] = (uint8_t)(r_v & 0xFF);
                i++;
            }
            crc_calc = get_CRC16(RS422.SentData.Data, out_idx);
            RS422.SentData.Data[out_idx++] = (uint8_t)(crc_calc & 0xFF);
            RS422.SentData.Data[out_idx++] = (uint8_t)(crc_calc >> 8);
            RS422.SentData.Length = out_idx;
            HAL_RS422_SendPacket();
        }
        else if (rx_buf[1] == 0x06) {
            addr = ((uint16_t)rx_buf[2] << 8) | rx_buf[3];
            val  = ((uint16_t)rx_buf[4] << 8) | rx_buf[5];
            if (addr == 19) {
                MB_RS422_Table.Data.Command.All = val;
                if (val & 0x01) P4OUT |=  FORCE_LTC4364_UV_PIN;  else P4OUT &= ~FORCE_LTC4364_UV_PIN;
                if (val & 0x02) P8OUT |=  FORCE_LTC4364_OV_PIN;  else P8OUT &= ~FORCE_LTC4364_OV_PIN;
                if (val & 0x04) P4OUT |=  VOUT_PWR_CTRL_PIN;     else P4OUT &= ~VOUT_PWR_CTRL_PIN;
                if (val & 0x08) P4OUT |=  EXT_FAULT_FLAG_PIN;    else P4OUT &= ~EXT_FAULT_FLAG_PIN;
                RS422.SentData.Length = rx_len;
                i = 0;
                while(i < rx_len) { RS422.SentData.Data[i] = rx_buf[i]; i++; }
                HAL_RS422_SendPacket();
            }
        }
    }
}
