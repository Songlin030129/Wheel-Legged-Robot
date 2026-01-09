#include "Comm_Task.h"

void COMM_Task()
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}