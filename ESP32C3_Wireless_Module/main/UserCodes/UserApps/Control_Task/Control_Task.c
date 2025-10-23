#include "Control_Task.h"
static const char *TAG = "Control_Task";
bool output_state = false;
void Control_Task()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << 7),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_config_t io_conf1 = {
        .pin_bit_mask = (1ULL << 8),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf1);

    while (1) {
        uint8_t sw_level = gpio_get_level(8);
        gpio_set_level(7, sw_level);
        output_state = sw_level;
        vTaskDelay(10);
    }
}