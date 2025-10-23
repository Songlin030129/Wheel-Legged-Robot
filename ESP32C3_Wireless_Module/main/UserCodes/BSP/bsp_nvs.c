#include "bsp_nvs.h"
#include "nvs_flash.h"

/**
 * @brief 初始化 NVS 存储分区
 */
esp_err_t bsp_nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI("MAIN", "NVS Flash initialized");
    return ret;
}
