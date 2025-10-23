#ifndef _LCD_HPP_
#define _LCD_HPP_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "st7789.h"

#ifdef __cplusplus
}
#endif

/**
 * @brief LCD颜色常量（RGB565）
 */
typedef enum {
    LCD_COLOR_WHITE = 0xFFFF,      /**< 白色 */
    LCD_COLOR_BLACK = 0x0000,      /**< 黑色 */
    LCD_COLOR_BLUE = 0x001F,       /**< 蓝色 */
    LCD_COLOR_BRED = 0xF81F,       /**< 品红色 */
    LCD_COLOR_GRED = 0xFFE0,       /**< 黄绿色 */
    LCD_COLOR_GBLUE = 0x07FF,      /**< 天蓝色 */
    LCD_COLOR_RED = 0xF800,        /**< 红色 */
    LCD_COLOR_MAGENTA = 0xF81F,    /**< 洋红色 */
    LCD_COLOR_GREEN = 0x07E0,      /**< 绿色 */
    LCD_COLOR_CYAN = 0x7FFF,       /**< 青色 */
    LCD_COLOR_YELLOW = 0xFFE0,     /**< 黄色 */
    LCD_COLOR_BROWN = 0xBC40,      /**< 棕色 */
    LCD_COLOR_BRRED = 0xFC07,      /**< 棕红色 */
    LCD_COLOR_GRAY = 0x8430,       /**< 灰色 */
    LCD_COLOR_DARKBLUE = 0x01CF,   /**< 深蓝色 */
    LCD_COLOR_LIGHTBLUE = 0x7D7C,  /**< 浅蓝色 */
    LCD_COLOR_GRAYBLUE = 0x5458,   /**< 灰蓝色 */
    LCD_COLOR_LIGHTGREEN = 0x841F, /**< 浅绿色 */
    LCD_COLOR_LGRAY = 0xC618,      /**< 浅灰色 */
    LCD_COLOR_LGRAYBLUE = 0xA651,  /**< 浅灰蓝色 */
    LCD_COLOR_LBBLUE = 0x2B12      /**< 浅棕蓝色 */
} LCD_ColorType_e;

/**
 * @brief 字体字号定义
 */
typedef enum { LCD_FONT_SIZE_12 = 12, LCD_FONT_SIZE_16 = 16, LCD_FONT_SIZE_24 = 24, LCD_FONT_SIZE_32 = 32 } LCD_FontSize_e;

/**
 * @brief 圆弧绘制区域标志
 */
typedef enum {
    LCD_CIRCLE_UPPER_RIGHT = 0x01,
    LCD_CIRCLE_UPPER_LEFT = 0x02,
    LCD_CIRCLE_LOWER_LEFT = 0x04,
    LCD_CIRCLE_LOWER_RIGHT = 0x08,
    LCD_CIRCLE_DRAW_ALL = (LCD_CIRCLE_UPPER_RIGHT | LCD_CIRCLE_UPPER_LEFT | LCD_CIRCLE_LOWER_LEFT | LCD_CIRCLE_LOWER_RIGHT)
} LCD_CircleType_e;

/**
 * @brief 绘图颜色模式
 */
typedef enum { LCD_COLOR_MODE_NORMAL = 0, LCD_COLOR_MODE_XOR = 1 } LCD_ColorMode_e;

/**
 * @brief LCD绘图上下文
 */
typedef struct {
    st7789_t *st7789_handle;
    LCD_ColorMode_e ColorMode; /**< 颜色叠加模式 */
    uint16_t backColor;        /**< 背景色 */
    uint16_t LCD_Buffer[LCD_H][LCD_W];

} LCD_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化LCD硬件和上下文
 * @param lcd LCD上下文指针
 */
void LCD_LinkST7789(LCD_t *lcd, st7789_t *st7789_handle);

/**
 * @brief 将缓冲区内容发送到屏幕
 * @param lcd LCD上下文指针
 */
void LCD_SendBuffer(LCD_t *lcd);

/**
 * @brief 清空屏幕缓冲区
 * @param lcd LCD上下文指针
 */
void LCD_ClearBuffer(LCD_t *lcd);

/**
 * @brief 在指定位置绘制一个像素
 * @param lcd LCD上下文指针
 * @param x 像素X坐标
 * @param y 像素Y坐标
 * @param color 像素颜色
 */
void LCD_DrawPoint(LCD_t *lcd, int16_t x, int16_t y, uint16_t color);

/**
 * @brief 绘制一条直线
 * @param lcd LCD上下文指针
 * @param x1 起点X坐标
 * @param y1 起点Y坐标
 * @param x2 终点X坐标
 * @param y2 终点Y坐标
 * @param color 线条颜色
 */
void LCD_DrawLine(LCD_t *lcd, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);

/**
 * @brief 绘制带行列的表格
 * @param lcd LCD上下文指针
 * @param x0 左上角X坐标
 * @param y0 左上角Y坐标
 * @param width 表格宽度
 * @param height 表格高度
 * @param row 行数
 * @param col 列数
 * @param color 线条颜色
 */
void LCD_DrawTab(LCD_t *lcd, int16_t x0, int16_t y0, uint16_t width, uint16_t height, uint16_t row, uint16_t col, uint16_t color);

/**
 * @brief 绘制空心矩形
 * @param lcd LCD上下文指针
 * @param x 左上角X坐标
 * @param y 左上角Y坐标
 * @param width 矩形宽度
 * @param height 矩形高度
 * @param color 边框颜色
 */
void LCD_DrawFrame(LCD_t *lcd, int16_t x, int16_t y, uint16_t width, uint16_t height, uint16_t color);

/**
 * @brief 绘制实心矩形
 * @param lcd LCD上下文指针
 * @param x 左上角X坐标
 * @param y 左上角Y坐标
 * @param width 矩形宽度
 * @param height 矩形高度
 * @param color 填充颜色
 */
void LCD_DrawBox(LCD_t *lcd, int16_t x, int16_t y, uint16_t width, uint16_t height, uint16_t color);

/**
 * @brief 绘制圆角空心矩形
 * @param lcd LCD上下文指针
 * @param x 左上角X坐标
 * @param y 左上角Y坐标
 * @param width 矩形宽度
 * @param height 矩形高度
 * @param color 边框颜色
 * @param r 圆角半径
 */
void LCD_DrawRFrame(LCD_t *lcd, int16_t x, int16_t y, uint16_t width, uint16_t height, uint16_t color, uint8_t r);

/**
 * @brief 绘制圆角实心矩形
 * @param lcd LCD上下文指针
 * @param x 左上角X坐标
 * @param y 左上角Y坐标
 * @param width 矩形宽度
 * @param height 矩形高度
 * @param color 填充颜色
 * @param r 圆角半径
 */
void LCD_DrawRBox(LCD_t *lcd, int16_t x, int16_t y, uint16_t width, uint16_t height, uint16_t color, uint8_t r);

/**
 * @brief 以中心点为基准绘制空心矩形
 * @param lcd LCD上下文指针
 * @param x 中心X坐标
 * @param y 中心Y坐标
 * @param width 矩形宽度
 * @param height 矩形高度
 * @param color 边框颜色
 */
void LCD_DrawFrame_1(LCD_t *lcd, int16_t x, int16_t y, uint16_t width, uint16_t height, uint16_t color);

/**
 * @brief 以中心点为基准绘制实心矩形
 * @param lcd LCD上下文指针
 * @param x 中心X坐标
 * @param y 中心Y坐标
 * @param width 矩形宽度
 * @param height 矩形高度
 * @param color 填充颜色
 */
void LCD_DrawBox_1(LCD_t *lcd, int16_t x, int16_t y, uint16_t width, uint16_t height, uint16_t color);

/**
 * @brief 以中心点为基准绘制圆角空心矩形
 * @param lcd LCD上下文指针
 * @param x 中心X坐标
 * @param y 中心Y坐标
 * @param width 矩形宽度
 * @param height 矩形高度
 * @param color 边框颜色
 * @param r 圆角半径
 */
void LCD_DrawRFrame_1(LCD_t *lcd, int16_t x, int16_t y, uint16_t width, uint16_t height, uint16_t color, uint8_t r);

/**
 * @brief 以中心点为基准绘制圆角实心矩形
 * @param lcd LCD上下文指针
 * @param x 中心X坐标
 * @param y 中心Y坐标
 * @param width 矩形宽度
 * @param height 矩形高度
 * @param color 填充颜色
 * @param r 圆角半径
 */
void LCD_DrawRBox_1(LCD_t *lcd, int16_t x, int16_t y, uint16_t width, uint16_t height, uint16_t color, uint8_t r);

/**
 * @brief 绘制圆弧或完整圆
 * @param lcd LCD上下文指针
 * @param x 圆心X坐标
 * @param y 圆心Y坐标
 * @param r 半径
 * @param color 绘制颜色
 * @param section 绘制区域标志
 */
void LCD_DrawCircle(LCD_t *lcd, int16_t x, int16_t y, uint16_t r, uint16_t color, LCD_CircleType_e section);

/**
 * @brief 绘制实心圆弧或圆盘
 * @param lcd LCD上下文指针
 * @param x 圆心X坐标
 * @param y 圆心Y坐标
 * @param r 半径
 * @param color 填充颜色
 * @param section 绘制区域标志
 */
void LCD_DrawDisc(LCD_t *lcd, int16_t x, int16_t y, uint16_t r, uint16_t color, LCD_CircleType_e section);

/**
 * @brief 显示单个ASCII字符
 * @param lcd LCD上下文指针
 * @param x 左上角X坐标
 * @param y 左上角Y坐标
 * @param chr 待显示的字符
 * @param fc 前景色
 * @param sizey 字高
 */
void LCD_ShowChar(LCD_t *lcd, int16_t x, int16_t y, uint8_t chr, uint16_t fc, uint8_t sizey);

/**
 * @brief 显示ASCII字符串
 * @param lcd LCD上下文指针
 * @param x 左上角X坐标
 * @param y 左上角Y坐标
 * @param str 字符串指针
 * @param fc 前景色
 * @param sizey 字高
 */
void LCD_ShowString(LCD_t *lcd, int16_t x, int16_t y, const uint8_t *str, uint16_t fc, uint8_t sizey);

/**
 * @brief 以格式化方式输出字符串
 * @param lcd LCD上下文指针
 * @param x 左上角X坐标
 * @param y 左上角Y坐标
 * @param fc 前景色
 * @param sizey 字高
 * @param fmt printf风格格式化字符串
 * @param ... 可变参数
 */
void LCD_ShowText(LCD_t *lcd, int16_t x, int16_t y, uint16_t fc, uint8_t sizey, const char *fmt, ...);

/**
 * @brief 右对齐格式化输出字符串
 * @param lcd LCD上下文指针
 * @param x 对齐区域参考X坐标
 * @param y 左上角Y坐标
 * @param fc 前景色
 * @param sizey 字高
 * @param totalWidth 对齐区域宽度，<=0时按照右边界x处理
 * @param fmt printf风格格式化字符串
 * @param ... 可变参数
 */
void LCD_ShowTextAlignRight(LCD_t *lcd, int16_t x, int16_t y, uint16_t fc, uint8_t sizey, int totalWidth, const char *fmt, ...);

/**
 * @brief 居中格式化输出字符串
 * @param lcd LCD上下文指针
 * @param x 中心X坐标
 * @param y 左上角Y坐标
 * @param fc 前景色
 * @param sizey 字高
 * @param fmt printf风格格式化字符串
 * @param ... 可变参数
 */
void LCD_ShowTextCentered(LCD_t *lcd, int16_t x, int16_t y, uint16_t fc, uint8_t sizey, const char *fmt, ...);

/**
 * @brief 显示RGB565彩色图片
 * @param lcd LCD上下文指针
 * @param x 左上角X坐标
 * @param y 左上角Y坐标
 * @param width 图片宽度
 * @param height 图片高度
 * @param pic 图像数据指针
 */
void LCD_ShowPicture(LCD_t *lcd, int16_t x, int16_t y, uint16_t width, uint16_t height, const uint8_t pic[]);

/**
 * @brief 缩放并显示RGB565彩色图片
 * @param lcd LCD上下文指针
 * @param x 左上角X坐标
 * @param y 左上角Y坐标
 * @param srcWidth 源图宽度
 * @param srcHeight 源图高度
 * @param dstWidth 目标宽度
 * @param dstHeight 目标高度
 * @param pic 图像数据指针
 */
void LCD_ShowResizedPicture(LCD_t *lcd, int16_t x, int16_t y, uint16_t srcWidth, uint16_t srcHeight, uint16_t dstWidth, uint16_t dstHeight,
                            const uint8_t pic[]);

/**
 * @brief 显示单色位图
 * @param lcd LCD上下文指针
 * @param x 左上角X坐标
 * @param y 左上角Y坐标
 * @param width 图片宽度
 * @param height 图片高度
 * @param color 显示颜色
 * @param pic 图像数据指针
 */
void LCD_ShowBinaryPicture(LCD_t *lcd, int16_t x, int16_t y, uint16_t width, uint16_t height, uint16_t color, const uint8_t pic[]);

/**
 * @brief 缩放并显示单色位图
 * @param lcd LCD上下文指针
 * @param x 左上角X坐标
 * @param y 左上角Y坐标
 * @param srcWidth 源图宽度
 * @param srcHeight 源图高度
 * @param dstWidth 目标宽度
 * @param dstHeight 目标高度
 * @param color 显示颜色
 * @param pic 图像数据指针
 */
void LCD_ShowResizedBinaryPicture(LCD_t *lcd, int16_t x, int16_t y, uint16_t srcWidth, uint16_t srcHeight, uint16_t dstWidth, uint16_t dstHeight,
                                  uint16_t color, const uint8_t pic[]);

/**
 * @brief 全局LCD上下文实例
 */
extern LCD_t tft_lcd;

#ifdef __cplusplus
}
#endif

#endif
