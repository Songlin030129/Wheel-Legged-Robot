#include "bsp_uart.h"
#include "VOFA_Debug.h"
#include "XboxController.h"
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    debug.UartReceive_IDLE_DMA_Callback(huart, Size);
    xbox.UartReceive_IDLE_DMA_Callback(huart, Size);
}
