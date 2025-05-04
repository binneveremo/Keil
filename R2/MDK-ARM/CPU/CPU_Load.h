#ifndef _CPU_UTILS_H__
#define _CPU_UTILS_H__

extern long long CPU_Tick;
#include "main.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "tim.h"


extern float CPU_USAGE_PERCENT;
void CPU_Usage_Init(void);
void CPU_Usage_Test(void);

#endif 

