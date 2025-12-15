#ifndef _BSP_CAN_H
#define _BSP_CAN_H

#include "common_inc.h"

typedef FDCAN_HandleTypeDef hcan_t;

extern void bsp_fdcan1_init(void);
extern void bsp_fdcan2_init(void);
extern uint8_t bsp_canx_send_data(FDCAN_HandleTypeDef* hcan, uint16_t id, uint8_t* data,
                                  uint32_t len);

#endif
