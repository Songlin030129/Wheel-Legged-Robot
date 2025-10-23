// main.c  — ESP-IDF 5.3.4 + NimBLE (pure C)
// 功能：作为 BLE Central 连接 Xbox Series X 控制器（HID），订阅 16B 输入，写入 8B 输出

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"

#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs_adv.h"
#include "host/ble_uuid.h"

#include "xbox_hid_report.h"
#include "xbox_parser.h"

static const char *TAG = "NimBLE-Xbox-C";

// ---------- 前置声明 ----------
static void start_scan(void);
static int on_gap_event(struct ble_gap_event *event, void *arg);
static void host_task(void *param);
static void on_sync(void);

// ---------- 运行期全局状态 ----------
static uint16_t g_conn_handle = 0xFFFF;

// 目标过滤
static const uint16_t APPEARANCE_CONTROLLER = 964;  // gamepad/controller
static const uint16_t UUID16_HID = 0x1812;
static const uint16_t UUID16_BAT = 0x180F;

// 识别的厂商数据（hex）："060000" 或 "0600030080"
static bool match_manufacturer_hex(const uint8_t *data, uint8_t len)
{
    char hex[2 * 31 + 1] = {0};  // adv 最多 31 字节
    int pos = 0;
    for (int i = 0; i < len && (pos + 2) < (int)sizeof(hex); ++i) {
        pos += snprintf(hex + pos, sizeof(hex) - pos, "%02x", data[i]);
    }
    return (strcmp(hex, "060000") == 0) || (strcmp(hex, "0600030080") == 0);
}

// 我们运行期收集到的服务/特征句柄
static uint16_t hid_val_handle_notify = 0;  // 输入报告（Notify）value handle
static uint16_t hid_ccc_handle_notify = 0;  // 其 CCCD（简化：val+1）
static uint16_t hid_val_handle_write = 0;   // 输出报告（Write）value handle

static uint16_t bat_val_handle_notify = 0;  // 0x2A19 battery level
static uint16_t bat_ccc_handle_notify = 0;

// 最近一次解析
static xbox_input_t g_last_input = {0};
static uint8_t g_battery = 0;

// 扫描时长（秒）
static int g_scan_time = 4;

// ---------- 小工具：打印地址 ----------
static void addr_to_str(const ble_addr_t *addr, char *dst, size_t dst_len)
{
    if (!addr || dst_len < 18) {
        if (dst)
            dst[0] = '\0';
        return;
    }
    // 注意 NimBLE 的 addr.val[0] 是 LSB；打印需要倒序
    snprintf(dst, dst_len, "%02X:%02X:%02X:%02X:%02X:%02X", addr->val[5], addr->val[4], addr->val[3], addr->val[2], addr->val[1], addr->val[0]);
}

// ---------- 特征发现回调 ----------
static int on_disc_chrs(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg)
{
    if (error->status != 0) {
        ESP_LOGI(TAG, "chr discovery done: status=%d", error->status);

        if (hid_val_handle_notify) {
            hid_ccc_handle_notify = hid_val_handle_notify + 1;
            uint8_t v[2] = {0x01, 0x00};
            int rc = ble_gattc_write_flat(g_conn_handle, hid_ccc_handle_notify, v, 2, NULL, NULL);
            if (rc)
                ESP_LOGW(TAG, "HID notify enable write rc=%d", rc);
        }
        if (bat_val_handle_notify) {
            bat_ccc_handle_notify = bat_val_handle_notify + 1;
            uint8_t v2[2] = {0x01, 0x00};
            int rc2 = ble_gattc_write_flat(g_conn_handle, bat_ccc_handle_notify, v2, 2, NULL, NULL);
            if (rc2)
                ESP_LOGW(TAG, "BAT notify enable write rc=%d", rc2);
        }
        return 0;
    }

    bool can_notify = (chr->properties & BLE_GATT_CHR_F_NOTIFY) != 0;
    bool can_write = (chr->properties & (BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP)) != 0;

    uint16_t svc_uuid16 = (uint16_t)(uintptr_t)arg;

    if (svc_uuid16 == UUID16_HID) {
        if (can_notify && !hid_val_handle_notify)
            hid_val_handle_notify = chr->val_handle;
        if (can_write && !hid_val_handle_write)
            hid_val_handle_write = chr->val_handle;
    } else if (svc_uuid16 == UUID16_BAT) {
        if (can_notify && !bat_val_handle_notify)
            bat_val_handle_notify = chr->val_handle;
    }
    return 0;
}

// ---------- 服务发现回调（只关心 HID / BAT） ----------
static int on_disc_svc(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *svc, void *arg)
{
    if (error->status != 0) {
        ESP_LOGI(TAG, "svc discovery finished: status=%d", error->status);
        return 0;
    }

    // 正确访问 UUID：先看类型，再取 16-bit 值
    if (svc->uuid.u.type != BLE_UUID_TYPE_16) {
        return 0;
    }
    uint16_t uuid16 = svc->uuid.u16.value;

    if (uuid16 == UUID16_HID || uuid16 == UUID16_BAT) {
        // 别传局部变量地址！把 16-bit 值打包进指针传下去
        void *pass = (void *)(uintptr_t)uuid16;

        int rc = ble_gattc_disc_all_chrs(conn_handle, svc->start_handle, svc->end_handle, on_disc_chrs, pass);
        if (rc) {
            ESP_LOGW(TAG, "disc_all_chrs rc=%d (uuid=0x%04X)", rc, uuid16);
        }
    }
    return 0;
}

// ---------- 广告过滤，识别 Xbox ----------
static bool is_xbox_from_adv(const struct ble_hs_adv_fields *f)
{
    // Appearance + HID uuid16 + （可选）厂商数据
    if (f->appearance_is_present && f->appearance == APPEARANCE_CONTROLLER) {
        bool has_hid = false;
        for (int i = 0; i < f->num_uuids16; ++i) {
            if (f->uuids16[i].value == UUID16_HID) {
                has_hid = true;
                break;
            }
        }
        bool mf_ok = true;
        if (f->mfg_data_len > 0)
            mf_ok = match_manufacturer_hex(f->mfg_data, f->mfg_data_len);
        return has_hid && mf_ok;
    }
    return false;
}

// ---------- 扫描 ----------
static void start_scan(void)
{
    struct ble_gap_disc_params p = {0};
    p.passive = 0;      // active scan
    p.itvl = 0x0061;    // ≈ 97.6 ms
    p.window = 0x0061;  // 同上
    p.filter_policy = 0;
    p.limited = 0;

    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, g_scan_time * 1000 /* ms */, &p, on_gap_event, NULL);
    if (rc) {
        ESP_LOGE(TAG, "ble_gap_disc rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "scan start");
    }
}

// ---------- GAP 事件 ----------
static int on_gap_event(struct ble_gap_event *e, void *arg)
{
    switch (e->type) {
        case BLE_GAP_EVENT_DISC: {
            struct ble_hs_adv_fields f;
            memset(&f, 0, sizeof(f));
            ble_hs_adv_parse_fields(&f, e->disc.data, e->disc.length_data);

            char addr_str[18];
            addr_to_str(&e->disc.addr, addr_str, sizeof(addr_str));

            if (is_xbox_from_adv(&f)) {
                ESP_LOGI(TAG, "found Xbox: %s rssi=%d", addr_str, e->disc.rssi);
                // 先停扫再连接
                ble_gap_disc_cancel();
                int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &e->disc.addr, 30000 /* ms */, NULL, on_gap_event, NULL);
                if (rc)
                    ESP_LOGE(TAG, "connect rc=%d", rc);
            }
            return 0;
        }

        case BLE_GAP_EVENT_CONNECT:
            if (e->connect.status == 0) {
                g_conn_handle = e->connect.conn_handle;

                // 通过连接句柄获取连接描述符，再拿对端地址
                struct ble_gap_conn_desc desc;
                if (ble_gap_conn_find(g_conn_handle, &desc) == 0) {
                    char addr_str[18];
                    addr_to_str(&desc.peer_id_addr, addr_str, sizeof(addr_str));
                    ESP_LOGI(TAG, "connected; handle=%u peer=%s", g_conn_handle, addr_str);
                }

                // 发现所有服务（回调里只处理 HID/BAT）
                ble_gattc_disc_all_svcs(g_conn_handle, on_disc_svc, NULL);
            } else {
                ESP_LOGW(TAG, "connect failed; status=%d", e->connect.status);
                start_scan();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "disconnect; reason=%d", e->disconnect.reason);
            g_conn_handle = 0xFFFF;
            start_scan();
            return 0;

        case BLE_GAP_EVENT_NOTIFY_RX: {
            // 收到 Notify：根据 val_handle 区分 HID/Battery
            uint16_t hv = e->notify_rx.attr_handle;
            if (hv == hid_val_handle_notify) {
                if (e->notify_rx.om->om_len == XBOX_INPUT_REPORT_LEN) {
                    uint8_t buf[XBOX_INPUT_REPORT_LEN];
                    os_mbuf_copydata(e->notify_rx.om, 0, XBOX_INPUT_REPORT_LEN, buf);

                    if (xbox_parse_input_report(buf, XBOX_INPUT_REPORT_LEN, &g_last_input) == 0) {
                        // 示例：按 A 键时震动 0.5s（左右 60）
                        if (g_last_input.btnA && hid_val_handle_write) {
                            xbox_out_t out;
                            xbox_out_init_all_off(&out);
                            out.select_bits |= (1u << 3) | (1u << 2);  // left + right
                            out.power_left = 60;
                            out.power_right = 60;
                            out.time_active = 50;  // 0.50s
                            uint8_t bytes[XBOX_OUT_REPORT_LEN];
                            xbox_out_to_bytes(&out, bytes);
                            ble_gattc_write_flat(g_conn_handle, hid_val_handle_write, bytes, XBOX_OUT_REPORT_LEN, NULL, NULL);
                        }
                    }
                }
            } else if (hv == bat_val_handle_notify) {
                if (e->notify_rx.om->om_len >= 1) {
                    uint8_t lv = 0;
                    os_mbuf_copydata(e->notify_rx.om, 0, 1, &lv);
                    g_battery = lv;
                    ESP_LOGI(TAG, "battery=%u%%", g_battery);
                }
            }
            return 0;
        }

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "mtu updated: %d", e->mtu.value);
            return 0;

        case BLE_GAP_EVENT_DISC_COMPLETE:
            ESP_LOGI(TAG, "scan complete");
            return 0;

        default:
            return 0;
    }
}

// ---------- NimBLE 同步回调 ----------
static void on_sync(void)
{
    uint8_t addr_val[6] = {0};
    ble_hs_id_infer_auto(0, &addr_val[0]);

    ble_addr_t addr;
    ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, addr.val, NULL);
    char s[18];
    addr_to_str(&addr, s, sizeof(s));
    ESP_LOGI(TAG, "addr %s", s);

    // 设备名（广播里可见）
    ble_svc_gap_device_name_set("ESP32-Xbox-C");

    start_scan();
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
    // 初始化 NimBLE 和控制器
    ESP_ERROR_CHECK(esp_nimble_hci_init());  // 如果还需要初始化 HCI 接口
    nimble_port_init();

    // GAP/GATT 通用服务（设备名、Appearance 等）
    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_hs_cfg.sync_cb = on_sync;

    // 启动 NimBLE Host 任务
    nimble_port_freertos_init(host_task);
    while (1) {
        vTaskDelay(1000);
    }
}
