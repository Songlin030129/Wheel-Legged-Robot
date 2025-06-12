#ifndef __COMMON_INC_H__
#define __COMMON_INC_H__

#ifdef __cplusplus

extern "C"
{
#endif

    /*---------------------------- C Scope ---------------------------*/
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "freertos_inc.h"
#include "semphr.h"
#include "task.h"
#include "string.h"

#include "fdcan.h"
#include "main.h"
#include "stdio.h"
#include "stm32h7xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "spi.h"
    void Main();

#ifdef __cplusplus
}

#endif
#endif // __COMMON_INC_H__