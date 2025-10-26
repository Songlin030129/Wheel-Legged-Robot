#include "BLE_Task.h"
#include "Control_Task.h"
#include "Debug_Task.h"
#include "INA_Task.h"
#include "LCD_Task.h"
#include "bsp_i2c.h"
#include "bsp_nvs.h"
#include "bsp_pwm.h"
#include "bsp_spi.h"
#include "common_inc.h"

static const char *TAG = "APP";

void DBGTask(void *argument)
{
    Debug_Task();
}
void LCDTask(void *argument)
{
    LCD_Task();
}
void CTRLTask(void *argument)
{
    Control_Task();
}
void BLETask(void *argument)
{
    BLE_Task();
}
void INATask(void *argument)
{
    INA_Task();
}
void Main()
{
    esp_err_t err = bsp_nvs_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bsp_nvs_init failed: %d", (int)err);
    }

    spi_bus_config_t buscfg = {
        .mosi_io_num = 0,
        .miso_io_num = -1,
        .sclk_io_num = 1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    err = bsp_spi_bus_init(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bsp_spi_bus_init failed: %d", (int)err);
    }

    err = bsp_i2c_bus_init(I2C_NUM_0, 6, 5, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bsp_i2c_bus_init failed: %d", (int)err);
    }

    err = bsp_pwm_timer_init(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, LEDC_TIMER_14_BIT, 2000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "timer_init failed: %d", (int)err);
    }

    BaseType_t xReturn = pdTRUE;
    xReturn = xTaskCreate(DBGTask, "DBGTask", 2048, NULL, 23, NULL);
    if (xReturn == pdTRUE)
        printf("DBG Task Create Success!\r\n");
    else
        printf("DBG Task Create Fail\r\n");

    xReturn = xTaskCreate(LCDTask, "LCDTask", 2048, NULL, 23, NULL);
    if (xReturn == pdTRUE)
        printf("LCD Task Create Success!\r\n");
    else
        printf("LCD Task Create Fail\r\n");

    xReturn = xTaskCreate(CTRLTask, "CTRLTask", 2048, NULL, 23, NULL);
    if (xReturn == pdTRUE)
        printf("CTRL Task Create Success!\r\n");
    else
        printf("CTRL Task Create Fail\r\n");

    xReturn = xTaskCreate(INATask, "INATask", 2048, NULL, 23, NULL);
    if (xReturn == pdTRUE)
        printf("INA Task Create Success!\r\n");
    else
        printf("INA Task Create Fail\r\n");

    xReturn = xTaskCreate(BLETask, "BLETask", 2048, NULL, 23, NULL);
    if (xReturn == pdTRUE)
        printf("BLE Task Create Success!\r\n");
    else
        printf("BLE Task Create Fail\r\n");
}
