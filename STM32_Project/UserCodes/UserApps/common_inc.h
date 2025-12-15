#ifndef __COMMON_INC_H__
#define __COMMON_INC_H__

#ifdef __cplusplus

extern "C" {
#endif
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define _PI 3.1415926f
#define _GRAVITY 9.8f
/*---------------------------- C Scope ---------------------------*/
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "string.h"
#include "task.h"

#include "fdcan.h"
#include "main.h"
#include "spi.h"
#include "stdio.h"
#include "stm32h7xx_hal.h"
#include "tim.h"
#include "usart.h"
void Main();

#ifdef __cplusplus
}

#endif
#endif  // __COMMON_INC_H__