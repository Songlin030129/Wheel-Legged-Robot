#include "common_inc.h"
#include "bsp_dwt.h"
#include "BMI088driver.h"
#include "bsp_can.h"
#include "INS_Task.h"
#include "Control_Task.h"
#include "VOFA_Debug.h"
static TaskHandle_t INS_Task_Handle;
static TaskHandle_t DBG_Task_Handle;
static TaskHandle_t Control_Task_Handle;
void INSTask(void* argument)
{
    while (1)
    {
        INS_task();
    }
}
void DBGTask(void* argument)
{
    debug.Init(&huart1);
    debug.Add_ValueCommander("E", &control.Control_Enable);
    debug.Add_ValueCommander("L", &control.Tar_L0_L);
    debug.Add_ValueCommander("T", &control.vmc_left.Tp);

    while (1)
    {
        debug.Run_Debug();
        vTaskDelay(10);

    }
}
void ControlTask(void* argument)
{
    while (1)
    {
        control.Task();
    }
}
// /*--------------------------主函数创建线程-------------------------*/
void Main()
{
    DWT_Init(480);

    /* BMI088初始化 */
    while (BMI088_init(&hspi2, 0) != BMI088_NO_ERROR)
    {
        ;
    }
    FDCAN1_Config();//can过滤器初始化
    FDCAN2_Config();

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



}
