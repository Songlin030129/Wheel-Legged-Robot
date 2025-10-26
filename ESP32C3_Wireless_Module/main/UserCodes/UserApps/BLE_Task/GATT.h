#ifndef __GATT_H__
#define __GATT_H__
#include "esp_err.h"
#include "esp_log.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "services/gatt/ble_svc_gatt.h"
#include "xbox_hid_report.h"
#include "xbox_parser.h"

int gatt_init();
void gatt_handle_notify(uint16_t conn_handle, uint16_t attr_handle, const uint8_t *data, size_t len);
int on_disc_svc(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *svc, void *arg);
extern xbox_input_t g_output;
extern uint8_t g_battery;

#endif