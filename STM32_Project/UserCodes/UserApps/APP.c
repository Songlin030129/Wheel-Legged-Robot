#include "BMI088driver.h"
#include "Control_Task.h"
#include "Debug_Task.h"
#include "INS_Task.h"
#include "Remote_Control.h"
#include "VOFA_Debug.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "common_inc.h"

static TaskHandle_t INS_Task_Handle;
static TaskHandle_t DBG_Task_Handle;
static TaskHandle_t Control_Task_Handle;
static TaskHandle_t Remote_Task_Handle;

void INSTask(void* argument)
{
    INS_task();
}
void DBGTask(void* argument)
{
    Debug_Task();
}
void ControlTask(void* argument)
{
    Control_Task();
}
void RemoteControlTask(void* argument)
{
    Remote_Control_Task();
}
// /*--------------------------主函数创建线程-------------------------*/
void Main()
{
    bsp_dwt_init(480);
    bsp_fdcan1_init();
    bsp_fdcan2_init();

    BaseType_t xReturn = pdTRUE;
    xReturn = xTaskCreate(INSTask, "INSTask", 512, NULL, osPriorityNormal, &INS_Task_Handle);
    if (xReturn == pdTRUE)
        printf("INS Task Create Success!\r\n");
    else
        printf("INS Task Create Fail\r\n");
    xReturn = xTaskCreate(DBGTask, "DBGTask", 512, NULL, osPriorityNormal, &DBG_Task_Handle);
    if (xReturn == pdTRUE)
        printf("DBG Task Create Success!\r\n");
    else
        printf("DBG Task Create Fail\r\n");
    xReturn = xTaskCreate(ControlTask, "ControlTask", 512, NULL, osPriorityNormal, &Control_Task_Handle);
    if (xReturn == pdTRUE)
        printf("Control Task Create Success!\r\n");
    else
        printf("Control Task Create Fail\r\n");
    xReturn = xTaskCreate(RemoteControlTask, "RemoteControlTask", 512, NULL, osPriorityNormal, &Remote_Task_Handle);
    if (xReturn == pdTRUE)
        printf("Remote Task Task Create Success!\r\n");
    else
        printf("Remote Task Task Create Fail\r\n");
}
