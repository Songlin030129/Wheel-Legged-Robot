#ifndef __GAP_H__
#define __GAP_H__
#include "GATT.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/timers.h"
#include "host/ble_gap.h"
#include "host/ble_sm.h"
#include "services/gap/ble_svc_gap.h"
#include "xbox_hid_report.h"
#include "xbox_parser.h"


int gap_init();
void scan_init(void);

#endif