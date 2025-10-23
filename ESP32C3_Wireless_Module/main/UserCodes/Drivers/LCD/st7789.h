#ifndef _ST7789_H_
#define _ST7789_H_

#include "bsp_spi.h"
#include "common_inc.h"

/**
 * @brief 设置横屏或竖屏显示：0/1 竖屏，2/3 横屏
 */
#ifndef USE_HORIZONTAL
#define USE_HORIZONTAL 2
#endif

#if USE_HORIZONTAL == 0 || USE_HORIZONTAL == 1
#define LCD_W 135
#define LCD_H 240
#else
#define LCD_W 240
#define LCD_H 135
#endif

#ifndef LCD_SPI_MAX_BYTES
#define LCD_SPI_MAX_BYTES 4092
#endif

typedef struct {
    spi_host_device_t spi_host;
    int cs_pin, dc_pin, rst_pin, blk_pin;
    int spi_clock_speed;
    spi_device_handle_t spi_handle;
} st7789_t;

/**
 * @brief 初始化 LCD 控制器
 */
void st7789_init(st7789_t *dev, spi_host_device_t spi_host, int spi_speed, int cs_pin, int dc_pin, int rst_pin, int blk_pin);

/**
 * @brief 向 LCD 连续写入数据
 *
 * @param dat  待写入的数据指针
 * @param size 数据长度（字节）
 */
void st7789_write_data(st7789_t *dev, uint8_t *dat, uint32_t size);

/**
 * @brief 设置 LCD 绘制窗口
 *
 * @param x1 左上角 X 坐标
 * @param y1 左上角 Y 坐标
 * @param x2 右下角 X 坐标
 * @param y2 右下角 Y 坐标
 */
void st7789_set_address(st7789_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
extern st7789_t st7789;

#endif
