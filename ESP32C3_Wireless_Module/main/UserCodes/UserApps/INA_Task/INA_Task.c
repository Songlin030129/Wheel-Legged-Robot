#include "INA_Task.h"
#include "ina226.h"
static const char *TAG = "INA_Task";
float vsh = 0, vbus = 0, cur = 0, power = 0;

void INA_Task()
{
    ina226_config_t config = {.avg = INA226_AVG_4, .vbus_ct = INA226_CT_588US, .vsh_ct = INA226_CT_588US, .mode = INA226_MODE_SHUNT_BUS_CONT};
    ina226_init(&ina226, I2C_NUM_0, 0x40, (uint16_t)400000, 0.0005f, 100.0f, &config);
    while (1) {
        if (ina226_read_shunt_voltage(&ina226, &vsh) == ESP_OK && ina226_read_bus_voltage(&ina226, &vbus) == ESP_OK &&
            ina226_read_current(&ina226, &cur) == ESP_OK && ina226_read_power(&ina226, &power) == ESP_OK) {
            ESP_LOGI(TAG, "Vsh=%.6f V, Vbus=%.3f V, I=%.3f A, P=%.3f W", vsh, vbus, cur, power);
        }
        vTaskDelay(100);
    }
}