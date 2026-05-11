#ifndef HAL_SYSTEM_H_
#define HAL_SYSTEM_H_

#include <stdint.h>

void HAL_System_ClockInit(void);
void HAL_System_TimerInit(void);
void HAL_System_SleepMs(uint16_t ms);

#endif
