#include "bsp_i2c.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "bsp_i2c";
static bool s_i2c_installed[I2C_NUM_MAX] = {0};

esp_err_t bsp_i2c_bus_init(i2c_port_t port, gpio_num_t scl_pin, gpio_num_t sda_pin, bool internal_pullup)
{
    if (port < 0 || port >= I2C_NUM_MAX)
        return ESP_ERR_INVALID_ARG;

    if (s_i2c_installed[port]) {
        ESP_LOGW(TAG, "I2C port %d already installed", port);
        return ESP_OK;
    }

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = port,
        .scl_io_num = scl_pin,
        .sda_io_num = sda_pin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = internal_pullup,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    s_i2c_installed[port] = true;
    return ESP_OK;
}

esp_err_t bsp_i2c_bus_deinit(i2c_port_t port)
{
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(port, &bus_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    return ESP_OK;
}

esp_err_t bsp_i2c_add_device(i2c_port_t port, i2c_device_config_t *config, i2c_master_dev_handle_t *i2c_dev_handle)
{
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(port, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, config, i2c_dev_handle));
    return ESP_OK;
}

esp_err_t bsp_i2c_remove_device(i2c_master_dev_handle_t handle)
{
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(handle));
    return ESP_OK;
}

esp_err_t bsp_i2c_master_write_read(i2c_master_dev_handle_t dev, const uint8_t *writebuf, size_t write_len, uint8_t *readbuf, size_t read_len,
                                    TickType_t timeout)
{
    return i2c_master_transmit_receive(dev, writebuf, write_len, readbuf, read_len, timeout);
}

esp_err_t bsp_i2c_master_write(i2c_master_dev_handle_t dev, const uint8_t *data, size_t len, TickType_t timeout)
{
    return i2c_master_transmit(dev, data, len, timeout);
}

esp_err_t bsp_i2c_master_read(i2c_master_dev_handle_t dev, uint8_t *data, size_t len, TickType_t timeout)
{
    return i2c_master_receive(dev, data, len, timeout);
}
