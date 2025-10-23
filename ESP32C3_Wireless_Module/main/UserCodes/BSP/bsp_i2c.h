#ifndef __BSP_I2C_H__
#define __BSP_I2C_H__

#include "common_inc.h"
#include "driver/i2c_master.h"
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 默认 I2C 端口编号
 */
#ifndef BSP_I2C_MASTER_PORT
#define BSP_I2C_MASTER_PORT I2C_NUM_0
#endif

/**
 * @brief 默认 SDA 引脚编号
 */
#ifndef BSP_I2C_MASTER_SDA_PIN
#define BSP_I2C_MASTER_SDA_PIN 6
#endif

/**
 * @brief 默认 SCL 引脚编号
 */
#ifndef BSP_I2C_MASTER_SCL_PIN
#define BSP_I2C_MASTER_SCL_PIN 7
#endif

/**
 * @brief 默认 I2C 时钟频率（Hz）
 */
#ifndef BSP_I2C_MASTER_FREQ_HZ
#define BSP_I2C_MASTER_FREQ_HZ (400 * 1000)
#endif

/**
 * @brief 初始化 I2C 总线
 *
 * @param port      I2C 主机编号
 * @param scl_pin   SCL 引脚
 * @param sda_pin   SDA 引脚
 * @return ESP_OK 表示初始化成功；否则返回底层错误码
 */
esp_err_t bsp_i2c_bus_init(i2c_port_t port, gpio_num_t scl_pin, gpio_num_t sda_pin, bool internal_pullup);

esp_err_t bsp_i2c_bus_deinit(i2c_port_t port);

esp_err_t bsp_i2c_add_device(i2c_port_t port, i2c_device_config_t *config, i2c_master_dev_handle_t *i2c_dev_handle);

esp_err_t bsp_i2c_remove_device(i2c_master_dev_handle_t handle);

/**
 * @brief I2C 主机写-读（先写寄存器地址，再读取数据）
 *
 * @param dev    I2C 设备句柄
 * @param writebuf 待写入的缓冲区指针
 * @param write_len 写入字节数
 * @param readbuf  待读取的缓冲区指针
 * @param read_len 读取字节数
 * @param timeout  超时时间（Tick）
 * @return ESP_OK 表示通讯成功；否则返回底层错误码
 */
esp_err_t bsp_i2c_master_write_read(i2c_master_dev_handle_t dev, const uint8_t *writebuf, size_t write_len, uint8_t *readbuf, size_t read_len,
                                    TickType_t timeout);

/**
 * @brief I2C 主机写设备
 *
 * @param dev    I2C 设备句柄
 * @param data    待写入的数据指针
 * @param len     写入字节数
 * @param timeout 超时时间（Tick）
 * @return ESP_OK 表示通讯成功；否则返回底层错误码
 */
esp_err_t bsp_i2c_master_write(i2c_master_dev_handle_t dev, const uint8_t *data, size_t len, TickType_t timeout);

/**
 * @brief I2C 主机读设备
 *
 * @param dev    I2C 设备句柄
 * @param data    读取缓冲区指针
 * @param len     读取字节数
 * @param timeout 超时时间（Tick）
 * @return ESP_OK 表示通讯成功；否则返回底层错误码
 */
esp_err_t bsp_i2c_master_read(i2c_master_dev_handle_t dev, uint8_t *data, size_t len, TickType_t timeout);

#ifdef __cplusplus
}
#endif

#endif  // __BSP_I2C_H__
