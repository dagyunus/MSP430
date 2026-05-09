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

void MB_RS422_UpdateTable(void)
{
    uint8_t buf[4];
    float f_val;
    uint8_t *p_f = (uint8_t*)&f_val;
    uint16_t idx, cfg;
    uint16_t adc_vals[5];
    int16_t raw_s;

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

    if(i2c_read_bytes(&g_i2c_ucb2, TMP101_INPUT_ADDR, TMP101_REG_TEMP, buf, 2) == I2C_OK) {
        raw_s = (int16_t)((((uint16_t)buf[0] << 8) | buf[1]) >> 4);
        if (raw_s & 0x0800) raw_s |= (int16_t)0xF000;
        f_val = (float)raw_s * 0.0625f;
        MB_RS422_Table.Regs[10] = ((uint16_t)p_f[3] << 8) | p_f[2];
        MB_RS422_Table.Regs[11] = ((uint16_t)p_f[1] << 8) | p_f[0];
    }

    if(i2c_read_bytes(&g_i2c_ucb0, TMP101_ISO_ADDR, TMP101_REG_TEMP, buf, 2) == I2C_OK) {
        raw_s = (int16_t)((((uint16_t)buf[0] << 8) | buf[1]) >> 4);
        if (raw_s & 0x0800) raw_s |= (int16_t)0xF000;
        f_val = (float)raw_s * 0.0625f;
        MB_RS422_Table.Regs[12] = ((uint16_t)p_f[3] << 8) | p_f[2];
        MB_RS422_Table.Regs[13] = ((uint16_t)p_f[1] << 8) | p_f[0];
    }

    cfg = 0xC383; buf[0] = (uint8_t)(cfg >> 8); buf[1] = (uint8_t)(cfg & 0xFF);
    if(i2c_write_reg_multi(&g_i2c_ucb0, ADS1015_ADDR, ADS1015_REG_CONFIG, buf, 2) == I2C_OK) {
        HAL_System_SleepMs(5);
        if(i2c_read_bytes(&g_i2c_ucb0, ADS1015_ADDR, ADS1015_REG_CONV, buf, 2) == I2C_OK) {
            raw_s = ((((uint16_t)buf[0] << 8) | buf[1]) >> 4) & 0x0FFF;
            f_val = (float)raw_s * 0.002f;
            MB_RS422_Table.Regs[14] = ((uint16_t)p_f[3] << 8) | p_f[2];
            MB_RS422_Table.Regs[15] = ((uint16_t)p_f[1] << 8) | p_f[0];
        }
    }

    cfg = 0xD383; buf[0] = (uint8_t)(cfg >> 8); buf[1] = (uint8_t)(cfg & 0xFF);
    if(i2c_write_reg_multi(&g_i2c_ucb0, ADS1015_ADDR, ADS1015_REG_CONFIG, buf, 2) == I2C_OK) {
        HAL_System_SleepMs(5);
        if(i2c_read_bytes(&g_i2c_ucb0, ADS1015_ADDR, ADS1015_REG_CONV, buf, 2) == I2C_OK) {
            raw_s = ((((uint16_t)buf[0] << 8) | buf[1]) >> 4) & 0x0FFF;
            f_val = (float)raw_s * 0.002f;
            MB_RS422_Table.Regs[16] = ((uint16_t)p_f[3] << 8) | p_f[2];
            MB_RS422_Table.Regs[17] = ((uint16_t)p_f[1] << 8) | p_f[0];
        }
    }

    MB_RS422_Table.Data.Status.All = 0;
    if (P4OUT & FORCE_LTC4364_UV_PIN)  MB_RS422_Table.Data.Status.Bits.Force_UV  = 1;
    if (P8OUT & FORCE_LTC4364_OV_PIN)  MB_RS422_Table.Data.Status.Bits.Force_OV  = 1;
    if (P4OUT & VOUT_PWR_CTRL_PIN)     MB_RS422_Table.Data.Status.Bits.VOut_Ctrl = 1;
    if (P4OUT & EXT_FAULT_FLAG_PIN)    MB_RS422_Table.Data.Status.Bits.Ext_Fault = 1;
}

void MB_RS422_Process(void)
{
    uint8_t rx_buf[RS422_BUFFER_SIZE], out_idx;
    uint16_t rx_len = 0, addr, qty, val, crc_calc, crc_rx, i;

    if (HAL_RS422_TickRxIdle(rx_buf, &rx_len, RS422_BUFFER_SIZE, 5) == 0) {
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
