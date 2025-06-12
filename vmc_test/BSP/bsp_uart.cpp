#include "bsp_uart.h"
#include "VOFA_Debug.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    debug.UartReceive_IDLE_DMA_Callback(huart, Size);
}
