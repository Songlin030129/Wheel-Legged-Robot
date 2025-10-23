#include "LED.h"
#include "driver/gpio.h"

LED_t led;

void LED_init(LED_t *led, uint8_t _pin)
{
    led->pin = _pin;
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << led->pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    LED_off(led);
    led->state = 0;
}

void LED_on(LED_t *led)
{
    gpio_set_level((gpio_num_t)led->pin, 0);
    led->state = 1;
}

void LED_off(LED_t *led)
{
    gpio_set_level((gpio_num_t)led->pin, 1);
    led->state = 0;
}

void LED_toggle(LED_t *led)
{
    gpio_set_level((gpio_num_t)led->pin, !led->state);
    led->state = !led->state;
}
