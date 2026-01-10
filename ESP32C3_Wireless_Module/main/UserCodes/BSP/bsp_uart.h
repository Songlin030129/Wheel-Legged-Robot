#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "common_inc.h"
esp_err_t bsp_uart_init(void);

void bsp_uart_send(const uint8_t *data, size_t len);

#endif