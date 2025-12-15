/**
 ******************************************************************************
 * @file	bsp_dwt.h
 * @author  Wang Hongxi
 * @version V1.1.0
 * @date    2022/3/8
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _BSP_DWT_H
#define _BSP_DWT_H

#include "common_inc.h"
#include "stdint.h"

typedef struct {
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;

void bsp_dwt_init(uint32_t CPU_Freq_mHz);
float bsp_dwt_get_dt(uint32_t* cnt_last);
double bsp_dwt_get_dt64(uint32_t* cnt_last);
float bsp_dwt_get_time_line_s(void);
float bsp_dwt_get_time_line_ms(void);
uint64_t bsp_dwt_get_time_line_us(void);
void bsp_dwt_delay(float Delay);
void bsp_dwt_delay_us(uint32_t us);
void bsp_dwt_systime_update(void);
float bsp_get_dt(uint32_t* cnt_last);

extern DWT_Time_t SysTime;

#endif /* BSP_DWT_H_ */
