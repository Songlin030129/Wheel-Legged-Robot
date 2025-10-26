#include "GATT.h"
static const char *TAG = "BLE_Task_GATT";

// 解析数据消息
xbox_input_t g_output = {0};
uint8_t g_battery = 0;

// 我们运行期收集到的服务/特征句柄
static uint16_t hid_val_handle_notify = 0;  // 输入报告（Notify）value handle
static uint16_t hid_ccc_handle_notify = 0;  // 其 CCCD（简化：val+1）
static uint16_t hid_val_handle_write = 0;   // 输出报告（Write）value handle

static uint16_t bat_val_handle_notify = 0;  // 0x2A19 battery level
static uint16_t bat_ccc_handle_notify = 0;

static int kick_next_chr_disc(uint16_t conn_handle);
static int on_disc_chrs(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg);

#define MAX_SVC_QUEUE 4  // 最多排几个服务
typedef struct {
    uint16_t uuid16;
    uint16_t start_handle;
    uint16_t end_handle;
} svc_info_t;

static svc_info_t g_svc_queue[MAX_SVC_QUEUE];
static int g_svc_count = 0;
static bool g_chr_disc_busy = false;

// 启动下一个服务的特征发现
static int kick_next_chr_disc(uint16_t conn_handle)
{
    if (g_chr_disc_busy || g_svc_count == 0)
        return 0;

    svc_info_t svc = g_svc_queue[0];
    // 左移队列
    for (int i = 1; i < g_svc_count; i++)
        g_svc_queue[i - 1] = g_svc_queue[i];
    g_svc_count--;

    g_chr_disc_busy = true;
    int rc = ble_gattc_disc_all_chrs(conn_handle, svc.start_handle, svc.end_handle, on_disc_chrs, (void *)(uintptr_t)svc.uuid16);
    if (rc) {
        g_chr_disc_busy = false;
        ESP_LOGW(TAG, "disc_all_chrs uuid=0x%04X rc=%d", svc.uuid16, rc);
    } else {
        ESP_LOGI(TAG, "Start chr discovery: svc=0x%04X [%u..%u]", svc.uuid16, svc.start_handle, svc.end_handle);
    }
    return rc;
}
// ---------- 服务发现回调 ----------
int on_disc_svc(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *svc, void *arg)
{
    if (error->status == BLE_HS_EDONE || error->status == BLE_ATT_ERR_ATTR_NOT_FOUND) {
        ESP_LOGI(TAG, "service discovery done (%d found)", g_svc_count);
        // 启动第一个待处理的特征发现
        kick_next_chr_disc(conn_handle);
        return 0;
    }

    if (error->status != 0) {
        ESP_LOGW(TAG, "service discovery err=%d", error->status);
        return error->status;
    }

    // 按需筛选服务：只保存需要的 UUID
    if (svc->uuid.u.type == BLE_UUID_TYPE_16) {
        uint16_t uuid = svc->uuid.u16.value;
        if (uuid == UUID16_HID || uuid == UUID16_BAT) {
            if (g_svc_count < MAX_SVC_QUEUE) {
                g_svc_queue[g_svc_count++] = (svc_info_t){
                    .uuid16 = uuid,
                    .start_handle = svc->start_handle,
                    .end_handle = svc->end_handle,
                };
                ESP_LOGI(TAG, "Queued svc 0x%04X [%u..%u]", uuid, svc->start_handle, svc->end_handle);
            }
        }
    }

    return 0;
}
// ---------- 特征发现回调 ----------
static int on_disc_chrs(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        ESP_LOGI(TAG, "chr discovery done: status=%d", error->status);
        g_chr_disc_busy = false;
        kick_next_chr_disc(conn_handle);  // 启动下一个服务
        if (hid_val_handle_notify) {
            hid_ccc_handle_notify = hid_val_handle_notify + 1;
            uint8_t v[2] = {0x01, 0x00};
            int rc = ble_gattc_write_flat(conn_handle, hid_ccc_handle_notify, v, 2, NULL, NULL);
            if (rc)
                ESP_LOGW(TAG, "HID notify enable write rc=%d", rc);
            else
                ESP_LOGI(TAG, "HID notify enable CCCD write success!");
        }
        if (bat_val_handle_notify) {
            bat_ccc_handle_notify = bat_val_handle_notify + 1;
            uint8_t v2[2] = {0x01, 0x00};
            int rc2 = ble_gattc_write_flat(conn_handle, bat_ccc_handle_notify, v2, 2, NULL, NULL);
            if (rc2)
                ESP_LOGW(TAG, "BAT notify enable write rc=%d", rc2);
            else
                ESP_LOGI(TAG, "BAT notify enable CCCD write success!");
        }
        return 0;
    } else if (error->status != 0) {
        volatile uint16_t svc_uuid16 = (uint16_t)(uintptr_t)arg;
        uint16_t chr_uuid16 = chr->uuid.u16.value;
        ESP_LOGW(TAG, "chrs discovery error, err code:%d, service uuid:0x%04X, chr uuid:0x%04X", error->status, svc_uuid16, chr_uuid16);
        return error->status;
    } else {
        bool can_notify = (chr->properties & BLE_GATT_CHR_F_NOTIFY) != 0;
        bool can_write = (chr->properties & (BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP)) != 0;

        uint16_t svc_uuid16 = (uint16_t)(uintptr_t)arg;

        if (svc_uuid16 == UUID16_HID) {
            if (can_notify && !hid_val_handle_notify) {
                hid_val_handle_notify = chr->val_handle;
                ESP_LOGI(TAG, "hid_val_handle_notify configure success: %d", hid_val_handle_notify);
            }
            if (can_write && !hid_val_handle_write) {
                hid_val_handle_write = chr->val_handle;
                ESP_LOGI(TAG, "hid_val_handle_write configure success: %d", hid_val_handle_write);
            }
        } else if (svc_uuid16 == UUID16_BAT) {
            if (can_notify && !bat_val_handle_notify) {
                bat_val_handle_notify = chr->val_handle;
                ESP_LOGI(TAG, "bat_val_handle_notify configure success: %d", bat_val_handle_notify);
            }
        }
        return 0;
    }
}

void gatt_handle_notify(uint16_t conn_handle, uint16_t attr_handle, const uint8_t *data, size_t len)
{
    if (attr_handle == hid_val_handle_notify && len == XBOX_INPUT_REPORT_LEN) {
        if (xbox_parse_input_report(data, len, &g_output) == 0) {
            // 示例：按 A 键时震动 0.5s（左右 60）
            // if (g_output.btnA && hid_val_handle_write) {
            //     xbox_out_t out;
            //     xbox_out_init_all_off(&out);
            //     out.select_bits |= (1u << 3) | (1u << 2);  // left + right
            //     out.power_left = 60;
            //     out.power_right = 60;
            //     out.time_active = 50;  // 0.50s
            //     uint8_t bytes[XBOX_OUT_REPORT_LEN];
            //     xbox_out_to_bytes(&out, bytes);
            //     ble_gattc_write_flat(conn_handle, hid_val_handle_write, bytes, XBOX_OUT_REPORT_LEN, NULL, NULL);
            // }
        }
    } else if (attr_handle == bat_val_handle_notify && len >= 1) {
        g_battery = data[0];
        ESP_LOGI(TAG, "Battery data:%d", g_battery);
        ESP_LOGI(TAG, "hid_val_handle_notify:%d", hid_val_handle_notify);
        ESP_LOGI(TAG, "hid_val_handle_write:%d", hid_val_handle_write);
        ESP_LOGI(TAG, "bat_val_handle_notify:%d", bat_val_handle_notify);
    }
}
int gatt_init()
{
    /* Local variables */
    int rc = 0;

    /* 1. GATT service initialization */
    ble_svc_gatt_init();

    return rc;
}