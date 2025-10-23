#include "bsp_spi.h"
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "bsp_spi";

static bool s_bus_inited[SPI_HOST_MAX] = {0};

esp_err_t bsp_spi_bus_init(spi_host_device_t host, const spi_bus_config_t *bus_config, spi_dma_chan_t dma_chan)
{
    if (!bus_config)
        return ESP_ERR_INVALID_ARG;
    if ((int)host < 0 || host >= SPI_HOST_MAX)
        return ESP_ERR_INVALID_ARG;

    if (s_bus_inited[host]) {
        ESP_LOGW(TAG, "SPI host %d already initialized", host);
        return ESP_OK;
    }

    esp_err_t err = spi_bus_initialize(host, bus_config, dma_chan);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SPI host %d already initialized by other module", host);
        err = ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "spi_bus_initialize failed");
    s_bus_inited[host] = true;
    return ESP_OK;
}

esp_err_t bsp_spi_bus_free(spi_host_device_t host)
{
    if ((int)host < 0 || host >= SPI_HOST_MAX)
        return ESP_ERR_INVALID_ARG;
    if (!s_bus_inited[host])
        return ESP_ERR_INVALID_STATE;

    esp_err_t err = spi_bus_free(host);
    ESP_RETURN_ON_ERROR(err, TAG, "spi_bus_free failed");
    s_bus_inited[host] = false;
    return ESP_OK;
}

esp_err_t bsp_spi_device_add(spi_host_device_t host, const spi_device_interface_config_t *dev_config, spi_device_handle_t *out_handle)
{
    if (!dev_config || !out_handle)
        return ESP_ERR_INVALID_ARG;
    if ((int)host < 0 || host >= SPI_HOST_MAX)
        return ESP_ERR_INVALID_ARG;
    if (!s_bus_inited[host])
        return ESP_ERR_INVALID_STATE;

    esp_err_t err = spi_bus_add_device(host, dev_config, out_handle);
    ESP_RETURN_ON_ERROR(err, TAG, "spi_bus_add_device failed");
    return ESP_OK;
}

esp_err_t bsp_spi_device_remove(spi_device_handle_t device)
{
    if (!device)
        return ESP_ERR_INVALID_ARG;
    esp_err_t err = spi_bus_remove_device(device);
    ESP_RETURN_ON_ERROR(err, TAG, "spi_bus_remove_device failed");
    return ESP_OK;
}

esp_err_t bsp_spi_device_tx(spi_device_handle_t device, const void *data, size_t length)
{
    if (!device)
        return ESP_ERR_INVALID_ARG;
    if (length == 0)
        return ESP_OK;
    if (!data)
        return ESP_ERR_INVALID_ARG;

    spi_transaction_t trans = {
        .length = length * 8,
    };
    if (length <= sizeof(trans.tx_data)) {
        trans.flags = SPI_TRANS_USE_TXDATA;
        memcpy(trans.tx_data, data, length);
    } else {
        trans.tx_buffer = data;
    }
    return spi_device_polling_transmit(device, &trans);
}

esp_err_t bsp_spi_device_rx(spi_device_handle_t device, void *out_data, size_t length)
{
    if (!device)
        return ESP_ERR_INVALID_ARG;
    if (length == 0)
        return ESP_OK;
    if (!out_data)
        return ESP_ERR_INVALID_ARG;

    spi_transaction_t trans = {
        .length = length * 8,
        .rxlength = length * 8,
    };
    if (length <= sizeof(trans.rx_data)) {
        trans.flags = SPI_TRANS_USE_RXDATA;
    } else {
        trans.rx_buffer = out_data;
    }
    esp_err_t err = spi_device_polling_transmit(device, &trans);
    if (err != ESP_OK)
        return err;

    if (trans.flags & SPI_TRANS_USE_RXDATA) {
        memcpy(out_data, trans.rx_data, length);
    }
    return ESP_OK;
}
