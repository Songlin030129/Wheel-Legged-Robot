// main.c  — ESP-IDF 5.3.4 + NimBLE (pure C)
// 功能：作为 BLE Central 连接 Xbox Series X 控制器（HID），订阅 16B 输入，写入 8B 输出

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "GAP.h"
#include "GATT.h"

#include "esp_err.h"
#include "esp_log.h"

#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs_adv.h"
#include "host/ble_sm.h"
#include "host/ble_store.h"
#include "host/ble_uuid.h"
#include "store/config/ble_store_config.h"
static const char *TAG = "BLE_Task";

// ---------- 前置声明 ----------
static void on_stack_sync(void);
static void on_stack_reset(int reason);
static void host_task(void *param);
void ble_store_config_init(void);
// ---------- NimBLE 同步回调 ----------
static void on_stack_sync(void)
{
    scan_init();
}
static void on_stack_reset(int reason)
{
    /* On reset, print reset reason to console */
    ESP_LOGI(TAG, "nimble stack reset, reset reason: %d", reason);
}
// ---------- NimBLE Host 任务 ----------
static void host_task(void *param)
{
    nimble_port_run();              // This function never returns
    nimble_port_freertos_deinit();  // Just in case
}

// ---------- app_main ----------
void BLE_Task(void)
{
    esp_err_t err;
    int ret;
    err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to initalize nimble stack, err code: %d ", err);
        return;
    }
    ret = gap_init();
    if (ret != 0) {
        ESP_LOGE(TAG, "failed to initalize GAP service, err code: %d", ret);
        return;
    }
    ret = gatt_init();
    if (ret != 0) {
        ESP_LOGE(TAG, "failed to initalize GATT service, err code: %d", ret);
        return;
    }

    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_sec_lvl = 2;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;

    ble_hs_cfg.sm_our_key_dist = BLE_HS_KEY_DIST_ENC_KEY | BLE_HS_KEY_DIST_ID_KEY;
    ble_hs_cfg.sm_their_key_dist = BLE_HS_KEY_DIST_ENC_KEY | BLE_HS_KEY_DIST_ID_KEY;
    ble_store_config_init();

    // 启动 NimBLE Host 任务
    nimble_port_freertos_init(host_task);
    while (1) {
        vTaskDelay(1000);
    }
}
