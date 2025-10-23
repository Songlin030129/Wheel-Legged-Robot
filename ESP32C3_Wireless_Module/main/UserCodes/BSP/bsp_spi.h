#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__

#include "common_inc.h"
#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化指定 SPI 总线
 *
 * @param host       SPI 主机编号（如 SPI2_HOST）
 * @param bus_config 总线配置指针，不可为 NULL
 * @param dma_chan   DMA 通道选择（可用 SPI_DMA_CH_AUTO）
 * @return ESP_OK 表示初始化成功；重复初始化会返回 ESP_OK；其他错误返回底层错误码
 */
esp_err_t bsp_spi_bus_init(spi_host_device_t host, const spi_bus_config_t *bus_config, spi_dma_chan_t dma_chan);

/**
 * @brief 释放指定 SPI 总线
 *
 * @param host SPI 主机编号
 * @return ESP_OK 表示释放成功；若总线尚未初始化则返回 ESP_ERR_INVALID_STATE
 */
esp_err_t bsp_spi_bus_free(spi_host_device_t host);

/**
 * @brief 将外设挂载到指定 SPI 总线上
 *
 * @param host        SPI 主机编号
 * @param dev_config  设备接口配置指针，不可为 NULL
 * @param out_handle  返回的设备句柄指针，不可为 NULL
 * @return ESP_OK 表示挂载成功；参数或状态不合法时返回相应错误码
 */
esp_err_t bsp_spi_device_add(spi_host_device_t host, const spi_device_interface_config_t *dev_config, spi_device_handle_t *out_handle);

/**
 * @brief 从 SPI 总线上移除指定外设
 *
 * @param device SPI 设备句柄
 * @return ESP_OK 表示移除成功；否则返回底层错误码
 */
esp_err_t bsp_spi_device_remove(spi_device_handle_t device);

/**
 * @brief 以轮询方式发送一帧 SPI 数据
 *
 * @param device SPI 设备句柄
 * @param data   待发送的数据指针（若 length<=4 支持使用内部寄存器发送）
 * @param length 字节长度，可为 0
 * @return ESP_OK 表示发送成功；否则返回底层错误码
 */
esp_err_t bsp_spi_device_tx(spi_device_handle_t device, const void *data, size_t length);

/**
 * @brief 以轮询方式读取 SPI 数据
 *
 * @param device   SPI 设备句柄
 * @param out_data 输出缓冲区指针
 * @param length   待读取的字节数
 * @return ESP_OK 表示读取成功；否则返回底层错误码
 */
esp_err_t bsp_spi_device_rx(spi_device_handle_t device, void *out_data, size_t length);

#ifdef __cplusplus
}
#endif

#endif  // __BSP_SPI_H__
