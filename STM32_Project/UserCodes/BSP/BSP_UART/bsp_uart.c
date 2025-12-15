#include "bsp_uart.h"
#include "VOFA_Debug.h"
#include "XboxController.h"
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    vofa_debug_uart_receive_idle_dma_callback(&debug, huart, Size);
    xbox_uart_receive_idle_dma_callback(&xbox, huart, Size);
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    vofa_debug_uart_err_callback(&debug, huart);
    xbox_uart_err_callback(&xbox, huart);
}