#ifndef MB_RS422_H_
#define MB_RS422_H_

#include <stdint.h>

#define MB_RS422_DEVICE_ID      0x01U
#define MB_RS422_REG_COUNT      16U

#include "i2c_drv.h"

/* Sensor I2C Addresses */
#define TMP101_INPUT_ADDR       0x48
#define TMP101_ISO_ADDR         0x4A
#define ADS1015_ADDR            0x48

typedef union {
    uint16_t All;
    struct {
        uint16_t Force_UV   : 1; /* Bit 0 */
        uint16_t Force_OV   : 1; /* Bit 1 */
        uint16_t VOut_Ctrl  : 1; /* Bit 2 */
        uint16_t Ext_Fault  : 1; /* Bit 3 */
        uint16_t Reserved   : 12;
    } Bits;
} MB_RS422_Ctrl_t;

typedef struct {
    float V_28V_Fltrd_Vmon; /* Reg 0-1 */
    float V_28V_Prot_Imon;  /* Reg 2-3 */
    float V_28V_Prot_Vmon;  /* Reg 4-5 */
    float Temp_Input_C;     /* Reg 6-7 */
    float Temp_Iso_C;       /* Reg 8-9 */
    float VOUT_270_Imon;    /* Reg 10-11 */
    float VOUT_270_Vmon;    /* Reg 12-13 */
    MB_RS422_Ctrl_t Status; /* Reg 14 */
    MB_RS422_Ctrl_t Command;/* Reg 15 */
} MB_RS422_Data_t;

typedef union {
    uint16_t        Regs[MB_RS422_REG_COUNT];
    MB_RS422_Data_t Data;
} MB_RS422_Table_t;

extern MB_RS422_Table_t MB_RS422_Table;

void MB_RS422_Init(void);
void MB_RS422_UpdateTable(void);
void MB_RS422_Process(void);

#endif
