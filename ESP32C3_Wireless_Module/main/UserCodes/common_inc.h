#ifndef __COMMON_INC_H__
#define __COMMON_INC_H__
#include "driver/gpio.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "stdio.h"
#include "string.h"

#ifdef __cplusplus
extern "C" {
#endif

void Main();

#ifdef __cplusplus
}
#endif
#endif  // __COMMON_INC_H__