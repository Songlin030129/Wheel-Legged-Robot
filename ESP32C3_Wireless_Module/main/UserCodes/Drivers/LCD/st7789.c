#include "st7789.h"
#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

static const char *TAG = "ST7789";

st7789_t st7789;

/**
 * @brief 毫秒级延时封装
 *
 * @param ms 延时时间（毫秒）
 */
static inline void st7789_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/**
 * @brief 设置 LCD 控制引脚电平
 */
static inline void st7789_dc(st7789_t *dev, uint32_t level)
{
    if (dev->dc_pin >= 0)
        gpio_set_level((gpio_num_t)dev->dc_pin, level);
}
static inline void st7789_rst(st7789_t *dev, uint32_t level)
{
    if (dev->rst_pin >= 0)
        gpio_set_level((gpio_num_t)dev->rst_pin, level);
}
static inline void st7789_blk(st7789_t *dev, uint32_t level)
{
    if (dev->blk_pin >= 0)
        gpio_set_level((gpio_num_t)dev->blk_pin, level);
}

/**
 * @brief SPI 发送辅助函数
 *
 * @param data 待发送的数据指针
 * @param len_bytes 数据长度（字节）
 * @return esp_err_t ESP_OK 表示成功，否则返回错误码
 */
static inline esp_err_t st7789_spi_tx(st7789_t *dev, const void *data, size_t len_bytes)
{
    return bsp_spi_device_tx(dev->spi_handle, data, len_bytes);
}

/**
 * @brief 向 LCD 写入 8 位数据
 *
 * @param dat 数据值
 */
static void st7789_write_data8(st7789_t *dev, uint8_t dat)
{
    st7789_dc(dev, 1);
    esp_err_t err = st7789_spi_tx(dev, &dat, 1);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "write data8 failed: %d", (int)err);
}

/**
 * @brief 向 LCD 写入 16 位数据
 *
 * @param dat 16 位数据值
 */
static void st7789_write_data16(st7789_t *dev, uint16_t dat)
{
    uint8_t temp[2] = {(uint8_t)(dat >> 8), (uint8_t)dat};
    st7789_dc(dev, 1);
    esp_err_t err = st7789_spi_tx(dev, temp, sizeof(temp));
    if (err != ESP_OK)
        ESP_LOGE(TAG, "write data16 failed: %d", (int)err);
}

/**
 * @brief 写入 LCD 命令寄存器
 *
 * @param dat 命令值
 */
static void st7789_write_reg(st7789_t *dev, uint8_t dat)
{
    st7789_dc(dev, 0);
    esp_err_t err = st7789_spi_tx(dev, &dat, 1);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "write command failed: %d", (int)err);
    st7789_dc(dev, 1);
}

/**
 * @brief 连续写入 LCD 数据缓冲区
 *
 * @param dat  数据指针
 * @param size 字节长度
 */
void st7789_write_data(st7789_t *dev, uint8_t *dat, uint32_t size)
{
    if (!dat || size == 0)
        return;
    st7789_dc(dev, 1);
    while (size) {
        uint32_t chunk_limit = (LCD_SPI_MAX_BYTES > 0) ? LCD_SPI_MAX_BYTES : size;
        uint32_t chunk = size > chunk_limit ? chunk_limit : size;
        esp_err_t err = st7789_spi_tx(dev, dat, chunk);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "write buffer failed: %d", (int)err);
            break;
        }
        dat += chunk;
        size -= chunk;
    }
}

/**
 * @brief 设置 LCD 绘制窗口
 *
 * @param x1 左上角 X 坐标
 * @param y1 左上角 Y 坐标
 * @param x2 右下角 X 坐标
 * @param y2 右下角 Y 坐标
 */
void st7789_set_address(st7789_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    if (USE_HORIZONTAL == 0) {
        st7789_write_reg(dev, 0x2a);
        st7789_write_data16(dev, x1 + 52);
        st7789_write_data16(dev, x2 + 52);
        st7789_write_reg(dev, 0x2b);
        st7789_write_data16(dev, y1 + 40);
        st7789_write_data16(dev, y2 + 40);
        st7789_write_reg(dev, 0x2c);
    } else if (USE_HORIZONTAL == 1) {
        st7789_write_reg(dev, 0x2a);
        st7789_write_data16(dev, x1 + 53);
        st7789_write_data16(dev, x2 + 53);
        st7789_write_reg(dev, 0x2b);
        st7789_write_data16(dev, y1 + 40);
        st7789_write_data16(dev, y2 + 40);
        st7789_write_reg(dev, 0x2c);
    } else if (USE_HORIZONTAL == 2) {
        st7789_write_reg(dev, 0x2a);
        st7789_write_data16(dev, x1 + 40);
        st7789_write_data16(dev, x2 + 40);
        st7789_write_reg(dev, 0x2b);
        st7789_write_data16(dev, y1 + 53);
        st7789_write_data16(dev, y2 + 53);
        st7789_write_reg(dev, 0x2c);
    } else {
        st7789_write_reg(dev, 0x2a);
        st7789_write_data16(dev, x1 + 40);
        st7789_write_data16(dev, x2 + 40);
        st7789_write_reg(dev, 0x2b);
        st7789_write_data16(dev, y1 + 52);
        st7789_write_data16(dev, y2 + 52);
        st7789_write_reg(dev, 0x2c);
    }
}

/**
 * @brief 初始化 ST7789 控制器
 */
void st7789_init(st7789_t *dev, spi_host_device_t spi_host, int spi_speed, int cs_pin, int dc_pin, int rst_pin, int blk_pin)
{
    dev->spi_host = spi_host;
    dev->spi_clock_speed = spi_speed;
    dev->cs_pin = cs_pin;
    dev->dc_pin = dc_pin;
    dev->rst_pin = rst_pin;
    dev->blk_pin = blk_pin;

    gpio_config_t io = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (dev->dc_pin >= 0)
        io.pin_bit_mask |= (1ULL << dev->dc_pin);
    if (dev->rst_pin >= 0)
        io.pin_bit_mask |= (1ULL << dev->rst_pin);
    if (dev->blk_pin >= 0)
        io.pin_bit_mask |= (1ULL << dev->blk_pin);

    gpio_config(&io);

    st7789_blk(dev, 0);

    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = dev->spi_clock_speed,
        .spics_io_num = dev->cs_pin,
        .queue_size = 4,
        .flags = SPI_DEVICE_NO_DUMMY,
    };
    esp_err_t err = bsp_spi_device_add(dev->spi_host, &devcfg, &(dev->spi_handle));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bsp_spi_device_add failed: %d", (int)err);
        return;
    }

    // 复位序列
    st7789_rst(dev, 0);
    st7789_delay_ms(100);
    st7789_rst(dev, 1);
    st7789_delay_ms(100);
    st7789_blk(dev, 1);
    st7789_delay_ms(100);

    // 控制器初始化寄存器写入
    st7789_write_reg(dev, 0x36);
    if (USE_HORIZONTAL == 0)
        st7789_write_data8(dev, 0x00);
    else if (USE_HORIZONTAL == 1)
        st7789_write_data8(dev, 0xC0);
    else if (USE_HORIZONTAL == 2)
        st7789_write_data8(dev, 0x70);
    else
        st7789_write_data8(dev, 0xA0);

    st7789_write_reg(dev, 0x3A);
    st7789_write_data8(dev, 0x05);

    st7789_write_reg(dev, 0xB2);
    st7789_write_data8(dev, 0x0C);
    st7789_write_data8(dev, 0x0C);
    st7789_write_data8(dev, 0x00);
    st7789_write_data8(dev, 0x33);
    st7789_write_data8(dev, 0x33);

    st7789_write_reg(dev, 0xB7);
    st7789_write_data8(dev, 0x35);

    st7789_write_reg(dev, 0xBB);
    st7789_write_data8(dev, 0x19);

    st7789_write_reg(dev, 0xC0);
    st7789_write_data8(dev, 0x2C);

    st7789_write_reg(dev, 0xC2);
    st7789_write_data8(dev, 0x01);

    st7789_write_reg(dev, 0xC3);
    st7789_write_data8(dev, 0x12);

    st7789_write_reg(dev, 0xC4);
    st7789_write_data8(dev, 0x20);

    st7789_write_reg(dev, 0xC6);
    st7789_write_data8(dev, 0x0F);

    st7789_write_reg(dev, 0xD0);
    st7789_write_data8(dev, 0xA4);
    st7789_write_data8(dev, 0xA1);

    st7789_write_reg(dev, 0xE0);
    st7789_write_data8(dev, 0xD0);
    st7789_write_data8(dev, 0x04);
    st7789_write_data8(dev, 0x0D);
    st7789_write_data8(dev, 0x11);
    st7789_write_data8(dev, 0x13);
    st7789_write_data8(dev, 0x2B);
    st7789_write_data8(dev, 0x3F);
    st7789_write_data8(dev, 0x54);
    st7789_write_data8(dev, 0x4C);
    st7789_write_data8(dev, 0x18);
    st7789_write_data8(dev, 0x0D);
    st7789_write_data8(dev, 0x0B);
    st7789_write_data8(dev, 0x1F);
    st7789_write_data8(dev, 0x23);

    st7789_write_reg(dev, 0xE1);
    st7789_write_data8(dev, 0xD0);
    st7789_write_data8(dev, 0x04);
    st7789_write_data8(dev, 0x0C);
    st7789_write_data8(dev, 0x11);
    st7789_write_data8(dev, 0x13);
    st7789_write_data8(dev, 0x2C);
    st7789_write_data8(dev, 0x3F);
    st7789_write_data8(dev, 0x44);
    st7789_write_data8(dev, 0x51);
    st7789_write_data8(dev, 0x2F);
    st7789_write_data8(dev, 0x1F);
    st7789_write_data8(dev, 0x1F);
    st7789_write_data8(dev, 0x20);
    st7789_write_data8(dev, 0x23);

    st7789_write_reg(dev, 0x21);

    st7789_write_reg(dev, 0x11);
    st7789_delay_ms(100);

    st7789_write_reg(dev, 0x29);
}
