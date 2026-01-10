#include "Comm_Task.h"
#include "GAP.h"
#include "GATT.h"
#include "bsp_uart.h"

char uart_send_buffer[512];

void COMM_Task()
{
    while (1) {
        sprintf(uart_send_buffer, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", gap_is_connected() && gatt_is_connected(),
                g_output.btnA, g_output.btnB, g_output.btnX, g_output.btnY, g_output.btnLB, g_output.btnRB, g_output.btnSelect, g_output.btnStart,
                g_output.btnXbox, g_output.btnShare, g_output.btnLS, g_output.btnRS, g_output.dirUp, g_output.dirDown, g_output.dirLeft,
                g_output.dirRight, (int)(g_output.joyLHori - 32768), ((int)g_output.joyLVert - 32768), ((int)g_output.joyRHori - 32768),
                ((int)g_output.joyRVert - 32768), g_output.trigLT, g_output.trigRT);
        bsp_uart_send((uint8_t*)uart_send_buffer, strlen(uart_send_buffer));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}