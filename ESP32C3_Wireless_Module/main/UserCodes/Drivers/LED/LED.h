#ifndef __LED_H__
#define __LED_H__
#include <stdint.h>
#include "UserCodes/common_inc.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t pin;
    uint8_t state;
} LED_t;

extern LED_t led;

void LED_init(LED_t *led, uint8_t _pin);
void LED_on(LED_t *led);
void LED_off(LED_t *led);
void LED_toggle(LED_t *led);

#ifdef __cplusplus
}
#endif

#endif