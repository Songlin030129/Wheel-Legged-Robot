#include "GAP.h"
static const char *TAG = "BLE_Task_GAP";
static TimerHandle_t scan_retry_timer;
// ---------- 运行期全局状态 ----------
static uint16_t g_conn_handle = 0xFFFF;
static bool g_tar_device_found = false;
static void addr_to_str(const ble_addr_t *addr, char *dst, size_t dst_len);
static bool match_manufacturer_hex(const uint8_t *data, uint8_t len);
static bool is_xbox_from_adv(const struct ble_hs_adv_fields *f);
static void start_scan(void);
static int gap_event_handler(struct ble_gap_event *e, void *arg);
static void scan_retry_timer_callback();

// 扫描时长（秒）
static int g_scan_time = 5;
static uint8_t g_own_addr_type = BLE_OWN_ADDR_PUBLIC;  // 默认值，占位
static void scan_retry_timer_callback()
{
    ESP_LOGI(TAG, "scan retry...");
    start_scan();
}
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

// ---------- 广播过滤，识别 Xbox ----------
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

// ---------- GAP 事件 ----------
static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_DISC: {
            struct ble_hs_adv_fields f;
            memset(&f, 0, sizeof(f));
            ble_hs_adv_parse_fields(&f, event->disc.data, event->disc.length_data);

            char addr_str[18];
            addr_to_str(&event->disc.addr, addr_str, sizeof(addr_str));

            if (is_xbox_from_adv(&f)) {
                ESP_LOGI(TAG, "found Xbox: %s rssi=%d", addr_str, event->disc.rssi);
                g_tar_device_found = true;
                // 先停扫再连接
                ble_gap_disc_cancel();
                int rc = ble_gap_connect(g_own_addr_type, &event->disc.addr, 30000 /* ms */, NULL, gap_event_handler, NULL);
                if (rc)
                    ESP_LOGE(TAG, "connect rc=%d", rc);
            }
            return 0;
        }

        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                g_conn_handle = event->connect.conn_handle;

                // 通过连接句柄获取连接描述符，再拿对端地址
                struct ble_gap_conn_desc desc;
                if (ble_gap_conn_find(g_conn_handle, &desc) == 0) {
                    char addr_str[18];
                    addr_to_str(&desc.peer_id_addr, addr_str, sizeof(addr_str));
                    ESP_LOGI(TAG, "connected; handle=%u peer=%s", g_conn_handle, addr_str);

                    if (desc.sec_state.encrypted) {
                        ESP_LOGI(TAG, "link already encrypted, start service discovery");
                        ble_gattc_disc_all_svcs(g_conn_handle, on_disc_svc, NULL);
                    } else {
                        struct ble_gap_upd_params up = {
                            .itvl_min = 24,  // 30 ms
                            .itvl_max = 40,  // 50 ms
                            .latency = 0,
                            .supervision_timeout = 600,  // 6.0 s  (单位 10 ms)
                        };
                        ble_gap_update_params(g_conn_handle, &up);
                        ESP_LOGI(TAG, "ble_gap_security_initiate");
                        int rc = ble_gap_security_initiate(g_conn_handle);
                        if (rc != 0) {
                            ESP_LOGE(TAG, "ble_gap_security_initiate rc=%d, disconnect", rc);
                            ble_gap_terminate(g_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                        }
                    }
                } else {
                    ESP_LOGW(TAG, "conn desc fetch failed, disconnect");
                    ble_gap_terminate(g_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                }
            } else {
                ESP_LOGW(TAG, "connect failed; status=%d", event->connect.status);
                start_scan();
            }
            return 0;
        case BLE_GAP_EVENT_PASSKEY_ACTION: {
            struct ble_sm_io io = {0};
            io.action = event->passkey.params.action;
            int rc = 0;
            switch (event->passkey.params.action) {
                case BLE_SM_IOACT_DISP:
                    ESP_LOGI(TAG, "Passkey display request: %06" PRIu32, event->passkey.params.numcmp);
                    io.passkey = event->passkey.params.numcmp;
                    break;

                case BLE_SM_IOACT_NUMCMP:
                    ESP_LOGI(TAG, "Numeric comparison %06" PRIu32 ", auto-accept", event->passkey.params.numcmp);
                    io.numcmp_accept = 1;
                    break;

                case BLE_SM_IOACT_INPUT:
                default:
                    ESP_LOGI(TAG, "Passkey input requested; respond with 000000");
                    io.passkey = 0;
                    break;
            }
            rc = ble_sm_inject_io(event->passkey.conn_handle, &io);
            if (rc != 0) {
                ESP_LOGW(TAG, "ble_sm_inject_io rc=%d", rc);
            }
            return 0;
        }
        case BLE_GAP_EVENT_REPEAT_PAIRING:
            return BLE_GAP_REPEAT_PAIRING_RETRY;
        case BLE_GAP_EVENT_ENC_CHANGE: {
            struct ble_gap_conn_desc desc;
            int ret = ble_gap_conn_find(g_conn_handle, &desc);
            if (event->enc_change.status == 0 && ret == 0 && desc.sec_state.encrypted) {
                ESP_LOGI(TAG, "encryption complete, start service discovery");
                ble_gattc_disc_all_svcs(event->enc_change.conn_handle, on_disc_svc, NULL);
            } else {
                ESP_LOGW(TAG, "encryption failed; status=%d", event->enc_change.status);
                ble_gap_terminate(event->enc_change.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            }
            return 0;
        }
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "disconnect; reason=%d", event->disconnect.reason);
            g_conn_handle = 0xFFFF;
            start_scan();
            return 0;

        case BLE_GAP_EVENT_NOTIFY_RX: {
            uint16_t hv = event->notify_rx.attr_handle;
            /* 拷贝数据到栈/堆后分发给 GATT 模块 */
            size_t len = os_mbuf_len(event->notify_rx.om);  // 或者确定长度
            uint8_t buf[XBOX_INPUT_REPORT_LEN];             // 若最大已知较小；否则 heap 分配
            os_mbuf_copydata(event->notify_rx.om, 0, len, buf);
            gatt_handle_notify(g_conn_handle, hv, buf, len);
            return 0;
        }

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "mtu updated: %d", event->mtu.value);
            return 0;

        case BLE_GAP_EVENT_DISC_COMPLETE:
            if (g_tar_device_found == false) {
                ESP_LOGI(TAG, "scan complete, target no found...");
                xTimerStart(scan_retry_timer, 0);
            }
            return 0;

        default:
            return 0;
    }
}

// ---------- 扫描 ----------
static void start_scan(void)
{
    struct ble_gap_disc_params p = {0};
    p.passive = 0;  // active scan
    p.itvl = 0x0061;
    p.window = 0x0061;
    p.filter_policy = 0;
    p.limited = 0;
    g_tar_device_found = false;

    int rc = ble_gap_disc(g_own_addr_type, g_scan_time * 1000 /* ms */, &p, gap_event_handler, NULL);
    if (rc) {
        ESP_LOGE(TAG, "ble_gap_disc rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "scan start");
    }
}

void scan_init(void)
{
    // 1) 推断“地址类型”（不是地址值！）
    int rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto rc=%d", rc);
        g_own_addr_type = BLE_OWN_ADDR_PUBLIC;  // 兜底
    }

    // 2) 正确按类型复制并打印“本机地址”
    ble_addr_t addr = {0};
    if (g_own_addr_type == BLE_OWN_ADDR_PUBLIC) {
        ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, addr.val, NULL);
    } else {
        // RANDOM / RPA 场景
        ble_hs_id_copy_addr(BLE_ADDR_RANDOM, addr.val, NULL);
    }
    char s[18];
    addr_to_str(&addr, s, sizeof(s));
    ESP_LOGI(TAG, "own_addr_type=%u, local_addr=%s", (unsigned)g_own_addr_type, s);

    // 3) 继续你原来的定时器与开扫
    scan_retry_timer = xTimerCreate("scan_retry", pdMS_TO_TICKS(5000), pdFALSE, NULL, scan_retry_timer_callback);
    start_scan();
}

int gap_init()
{
    /* Local variables */
    int rc = 0;

    /* Call NimBLE GAP initialization API */
    ble_svc_gap_init();

    /* Set GAP device name */
    rc = ble_svc_gap_device_name_set("ESP32-Xbox-C");
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set device name to %s, error code: %d", "ESP32-Xbox-C", rc);
        return rc;
    }
    return rc;
}
