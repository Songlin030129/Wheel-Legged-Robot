#ifndef __BSP_NVS_H__
#define __BSP_NVS_H__

#include "common_inc.h"

/**
 * @brief 初始化 NVS 存储分区
 *
 * @return ESP_OK 表示初始化成功；否则返回底层错误码
 */
esp_err_t bsp_nvs_init(void);

#endif
