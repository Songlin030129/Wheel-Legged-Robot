#include "lcd.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "user_lcd_font.h"
#include "user_lcd_pic.h"

LCD_t tft_lcd = {
    .ColorMode = LCD_COLOR_MODE_NORMAL,
};

void LCD_LinkST7789(LCD_t *lcd, st7789_t *st7789_handle)
{
    lcd->st7789_handle = st7789_handle;
}

void LCD_SendBuffer(LCD_t *lcd)
{
    st7789_set_address(lcd->st7789_handle, 0, 0, LCD_W - 1, LCD_H - 1);
    st7789_write_data(lcd->st7789_handle, (uint8_t *)lcd->LCD_Buffer, LCD_H * LCD_W * 2);
}

void LCD_ClearBuffer(LCD_t *lcd)
{
    memset(lcd->LCD_Buffer, 0, sizeof(lcd->LCD_Buffer));
}

void LCD_DrawPoint(LCD_t *lcd, int16_t x, int16_t y, uint16_t color)
{
    if (x < 0 || y < 0 || x >= LCD_W || y >= LCD_H) {
        return;
    }

    if (lcd->ColorMode == LCD_COLOR_MODE_XOR) {
        if (lcd->LCD_Buffer[y][x] == color) {
            lcd->LCD_Buffer[y][x] = lcd->backColor;
        } else {
            lcd->LCD_Buffer[y][x] = color;
        }
    } else {
        lcd->LCD_Buffer[y][x] = color;
    }
}

void LCD_DrawLine(LCD_t *lcd, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
    int delta_x = x2 - x1;
    int delta_y = y2 - y1;
    int incx = 0;
    int incy = 0;
    int distance;
    int xerr = 0;
    int yerr = 0;
    int uRow = x1;
    int uCol = y1;

    if (delta_x > 0) {
        incx = 1;
    } else if (delta_x < 0) {
        incx = -1;
        delta_x = -delta_x;
    }

    if (delta_y > 0) {
        incy = 1;
    } else if (delta_y < 0) {
        incy = -1;
        delta_y = -delta_y;
    }

    distance = (delta_x > delta_y) ? delta_x : delta_y;

    for (int t = 0; t <= distance; t++) {
        LCD_DrawPoint(lcd, uRow, uCol, color);
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance) {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance) {
            yerr -= distance;
            uCol += incy;
        }
    }
}

void LCD_DrawTab(LCD_t *lcd, int16_t x0, int16_t y0, uint16_t width, uint16_t height, uint16_t row, uint16_t col, uint16_t color)
{
    if (row == 0 || col == 0) {
        return;
    }
    for (uint16_t i = 0; i <= row; i++) {
        int16_t y = (int16_t)(y0 + (int32_t)height * i / row);
        LCD_DrawLine(lcd, x0, y, x0 + (int16_t)width, y, color);
    }
    for (uint16_t j = 0; j <= col; j++) {
        int16_t x = (int16_t)(x0 + (int32_t)width * j / col);
        LCD_DrawLine(lcd, x, y0, x, y0 + (int16_t)height, color);
    }
}

void LCD_DrawFrame(LCD_t *lcd, int16_t x, int16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    for (int16_t i = x; i < x + (int16_t)width; i++) {
        LCD_DrawPoint(lcd, i, y, color);
        LCD_DrawPoint(lcd, i, y + (int16_t)height - 1, color);
    }
    for (int16_t j = y; j < y + (int16_t)height; j++) {
        LCD_DrawPoint(lcd, x, j, color);
        LCD_DrawPoint(lcd, x + (int16_t)width - 1, j, color);
    }
}

void LCD_DrawBox(LCD_t *lcd, int16_t x, int16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    for (int16_t i = x; i < x + (int16_t)width; ++i) {
        for (int16_t j = y; j < y + (int16_t)height; ++j) {
            LCD_DrawPoint(lcd, i, j, color);
        }
    }
}

void LCD_DrawRFrame(LCD_t *lcd, int16_t x, int16_t y, uint16_t width, uint16_t height, uint16_t color, uint8_t r)
{
    for (int16_t i = x + r + 1; i < x + (int16_t)width - r - 1; i++) {
        LCD_DrawPoint(lcd, i, y, color);
        LCD_DrawPoint(lcd, i, y + (int16_t)height - 1, color);
    }
    for (int16_t j = y + r + 1; j < y + (int16_t)height - r - 1; j++) {
        LCD_DrawPoint(lcd, x, j, color);
        LCD_DrawPoint(lcd, x + (int16_t)width - 1, j, color);
    }

    LCD_DrawCircle(lcd, x + r, y + r, r, color, LCD_CIRCLE_UPPER_LEFT);
    LCD_DrawCircle(lcd, x + (int16_t)width - 1 - r, y + r, r, color, LCD_CIRCLE_UPPER_RIGHT);
    LCD_DrawCircle(lcd, x + r, y + (int16_t)height - 1 - r, r, color, LCD_CIRCLE_LOWER_LEFT);
    LCD_DrawCircle(lcd, x + (int16_t)width - 1 - r, y + (int16_t)height - 1 - r, r, color, LCD_CIRCLE_LOWER_RIGHT);
}

void LCD_DrawRBox(LCD_t *lcd, int16_t x, int16_t y, uint16_t width, uint16_t height, uint16_t color, uint8_t r)
{
    LCD_DrawDisc(lcd, x + r, y + r, r, color, LCD_CIRCLE_UPPER_LEFT);
    LCD_DrawDisc(lcd, x + (int16_t)width - 1 - r, y + r, r, color, LCD_CIRCLE_UPPER_RIGHT);
    LCD_DrawDisc(lcd, x + r, y + (int16_t)height - 1 - r, r, color, LCD_CIRCLE_LOWER_LEFT);
    LCD_DrawDisc(lcd, x + (int16_t)width - 1 - r, y + (int16_t)height - 1 - r, r, color, LCD_CIRCLE_LOWER_RIGHT);

    LCD_DrawBox(lcd, x + r + 1, y, width - 2 - 2 * r, r + 1, color);
    LCD_DrawBox(lcd, x, y + r + 1, width, height - 2 * r - 2, color);
    LCD_DrawBox(lcd, x + r + 1, y + (int16_t)height - 1 - r, width - 2 - 2 * r, r + 1, color);
}

void LCD_DrawFrame_1(LCD_t *lcd, int16_t x, int16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    x -= (int16_t)width / 2;
    y -= (int16_t)height / 2;
    for (int16_t i = x; i < x + (int16_t)width; i++) {
        LCD_DrawPoint(lcd, i, y, color);
        LCD_DrawPoint(lcd, i, y + (int16_t)height - 1, color);
    }
    for (int16_t j = y; j < y + (int16_t)height; j++) {
        LCD_DrawPoint(lcd, x, j, color);
        LCD_DrawPoint(lcd, x + (int16_t)width - 1, j, color);
    }
}

void LCD_DrawBox_1(LCD_t *lcd, int16_t x, int16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    x -= (int16_t)width / 2;
    y -= (int16_t)height / 2;
    for (int16_t i = x; i < x + (int16_t)width; ++i) {
        for (int16_t j = y; j < y + (int16_t)height; ++j) {
            LCD_DrawPoint(lcd, i, j, color);
        }
    }
}

void LCD_DrawRFrame_1(LCD_t *lcd, int16_t x, int16_t y, uint16_t width, uint16_t height, uint16_t color, uint8_t r)
{
    x -= (int16_t)width / 2;
    y -= (int16_t)height / 2;
    for (int16_t i = x + r + 1; i < x + (int16_t)width - r - 1; i++) {
        LCD_DrawPoint(lcd, i, y, color);
        LCD_DrawPoint(lcd, i, y + (int16_t)height - 1, color);
    }
    for (int16_t j = y + r + 1; j < y + (int16_t)height - r - 1; j++) {
        LCD_DrawPoint(lcd, x, j, color);
        LCD_DrawPoint(lcd, x + (int16_t)width - 1, j, color);
    }

    LCD_DrawCircle(lcd, x + r, y + r, r, color, LCD_CIRCLE_UPPER_LEFT);
    LCD_DrawCircle(lcd, x + (int16_t)width - 1 - r, y + r, r, color, LCD_CIRCLE_UPPER_RIGHT);
    LCD_DrawCircle(lcd, x + r, y + (int16_t)height - 1 - r, r, color, LCD_CIRCLE_LOWER_LEFT);
    LCD_DrawCircle(lcd, x + (int16_t)width - 1 - r, y + (int16_t)height - 1 - r, r, color, LCD_CIRCLE_LOWER_RIGHT);
}

void LCD_DrawRBox_1(LCD_t *lcd, int16_t x, int16_t y, uint16_t width, uint16_t height, uint16_t color, uint8_t r)
{
    x -= (int16_t)width / 2;
    y -= (int16_t)height / 2;
    LCD_DrawDisc(lcd, x + r, y + r, r, color, LCD_CIRCLE_UPPER_LEFT);
    LCD_DrawDisc(lcd, x + (int16_t)width - 1 - r, y + r, r, color, LCD_CIRCLE_UPPER_RIGHT);
    LCD_DrawDisc(lcd, x + r, y + (int16_t)height - 1 - r, r, color, LCD_CIRCLE_LOWER_LEFT);
    LCD_DrawDisc(lcd, x + (int16_t)width - 1 - r, y + (int16_t)height - 1 - r, r, color, LCD_CIRCLE_LOWER_RIGHT);

    LCD_DrawBox(lcd, x + r + 1, y, width - 2 - 2 * r, r + 1, color);
    LCD_DrawBox(lcd, x, y + r + 1, width, height - 2 * r - 2, color);
    LCD_DrawBox(lcd, x + r + 1, y + (int16_t)height - 1 - r, width - 2 - 2 * r, r + 1, color);
}

void LCD_DrawCircle(LCD_t *lcd, int16_t x, int16_t y, uint16_t r, uint16_t color, LCD_CircleType_e section)
{
    uint16_t x0 = (uint16_t)(r * cos(0.01745 * 45.0));
    uint16_t fx;

    for (int i = -((int)x0) + 1; i < 0; ++i) {
        double value = (double)r * (double)r - (double)i * (double)i;
        if (value < 0.0) {
            value = 0.0;
        }
        fx = (uint16_t)sqrt(value);
        if (section & LCD_CIRCLE_UPPER_RIGHT) {
            LCD_DrawPoint(lcd, x - i, y - (int16_t)fx, color);
            LCD_DrawPoint(lcd, x + (int16_t)fx, y + (int16_t)i, color);
        }
        if (section & LCD_CIRCLE_UPPER_LEFT) {
            LCD_DrawPoint(lcd, x + (int16_t)i, y - (int16_t)fx, color);
            LCD_DrawPoint(lcd, x - (int16_t)fx, y + (int16_t)i, color);
        }
        if (section & LCD_CIRCLE_LOWER_LEFT) {
            LCD_DrawPoint(lcd, x + (int16_t)i, y + (int16_t)fx, color);
            LCD_DrawPoint(lcd, x - (int16_t)fx, y - (int16_t)i, color);
        }
        if (section & LCD_CIRCLE_LOWER_RIGHT) {
            LCD_DrawPoint(lcd, x - (int16_t)i, y + (int16_t)fx, color);
            LCD_DrawPoint(lcd, x + (int16_t)fx, y - (int16_t)i, color);
        }
    }

    fx = (uint16_t)sqrt((double)r * (double)r - (double)x0 * (double)x0);
    if (section & LCD_CIRCLE_UPPER_RIGHT) {
        if (r > 1) {
            LCD_DrawPoint(lcd, x + (int16_t)r, y, color);
            LCD_DrawPoint(lcd, x, y - (int16_t)r, color);
        }
        if (x0 == fx) {
            LCD_DrawPoint(lcd, x + (int16_t)x0, y - (int16_t)x0, color);
        } else {
            LCD_DrawPoint(lcd, x + (int16_t)x0, y - (int16_t)fx, color);
            LCD_DrawPoint(lcd, x + (int16_t)fx, y - (int16_t)x0, color);
        }
    }
    if (section & LCD_CIRCLE_UPPER_LEFT) {
        if (r > 1) {
            LCD_DrawPoint(lcd, x - (int16_t)r, y, color);
            LCD_DrawPoint(lcd, x, y - (int16_t)r, color);
        }
        if (x0 == fx) {
            LCD_DrawPoint(lcd, x - (int16_t)x0, y - (int16_t)x0, color);
        } else {
            LCD_DrawPoint(lcd, x - (int16_t)x0, y - (int16_t)fx, color);
            LCD_DrawPoint(lcd, x - (int16_t)fx, y - (int16_t)x0, color);
        }
    }
    if (section & LCD_CIRCLE_LOWER_LEFT) {
        if (r > 1) {
            LCD_DrawPoint(lcd, x - (int16_t)r, y, color);
            LCD_DrawPoint(lcd, x, y + (int16_t)r, color);
        }
        if (x0 == fx) {
            LCD_DrawPoint(lcd, x - (int16_t)x0, y + (int16_t)x0, color);
        } else {
            LCD_DrawPoint(lcd, x - (int16_t)x0, y + (int16_t)fx, color);
            LCD_DrawPoint(lcd, x - (int16_t)fx, y + (int16_t)x0, color);
        }
    }
    if (section & LCD_CIRCLE_LOWER_RIGHT) {
        if (r > 1) {
            LCD_DrawPoint(lcd, x + (int16_t)r, y, color);
            LCD_DrawPoint(lcd, x, y + (int16_t)r, color);
        }
        if (x0 == fx) {
            LCD_DrawPoint(lcd, x + (int16_t)x0, y + (int16_t)x0, color);
        } else {
            LCD_DrawPoint(lcd, x + (int16_t)x0, y + (int16_t)fx, color);
            LCD_DrawPoint(lcd, x + (int16_t)fx, y + (int16_t)x0, color);
        }
    }
    if (section == LCD_CIRCLE_DRAW_ALL) {
        LCD_DrawPoint(lcd, x + (int16_t)r, y, color);
        LCD_DrawPoint(lcd, x - (int16_t)r, y, color);
        LCD_DrawPoint(lcd, x, y - (int16_t)r, color);
        LCD_DrawPoint(lcd, x, y + (int16_t)r, color);
    }
}

void LCD_DrawDisc(LCD_t *lcd, int16_t x, int16_t y, uint16_t r, uint16_t color, LCD_CircleType_e section)
{
    uint16_t x0 = (uint16_t)(r * cos(0.01745 * 45.0));
    uint16_t fx;

    for (int i = -((int)x0) + 1; i < 0; ++i) {
        double value = (double)r * (double)r - (double)i * (double)i;
        if (value < 0.0) {
            value = 0.0;
        }
        fx = (uint16_t)sqrt(value);
        if (section & LCD_CIRCLE_UPPER_RIGHT) {
            LCD_DrawLine(lcd, x - (int16_t)i, y - (int16_t)fx, x - (int16_t)i, y + (int16_t)i, color);
            LCD_DrawLine(lcd, x + (int16_t)fx, y + (int16_t)i, x - (int16_t)i, y + (int16_t)i, color);
        }
        if (section & LCD_CIRCLE_UPPER_LEFT) {
            LCD_DrawLine(lcd, x + (int16_t)i, y - (int16_t)fx, x + (int16_t)i, y + (int16_t)i, color);
            LCD_DrawLine(lcd, x - (int16_t)fx, y + (int16_t)i, x + (int16_t)i, y + (int16_t)i, color);
        }
        if (section & LCD_CIRCLE_LOWER_LEFT) {
            LCD_DrawLine(lcd, x + (int16_t)i, y + (int16_t)fx, x + (int16_t)i, y - (int16_t)i, color);
            LCD_DrawLine(lcd, x - (int16_t)fx, y - (int16_t)i, x + (int16_t)i, y - (int16_t)i, color);
        }
        if (section & LCD_CIRCLE_LOWER_RIGHT) {
            LCD_DrawLine(lcd, x - (int16_t)i, y + (int16_t)fx, x - (int16_t)i, y - (int16_t)i, color);
            LCD_DrawLine(lcd, x + (int16_t)fx, y - (int16_t)i, x - (int16_t)i, y - (int16_t)i, color);
        }
    }

    LCD_DrawPoint(lcd, x, y, color);
    if (r != 2) {
        LCD_DrawPoint(lcd, x, y, color);
    }

    fx = (uint16_t)sqrt((double)r * (double)r - (double)x0 * (double)x0);
    if (section & LCD_CIRCLE_UPPER_RIGHT) {
        if (r > 1) {
            LCD_DrawLine(lcd, x + (int16_t)r, y, x, y, color);
            LCD_DrawLine(lcd, x, y - (int16_t)r, x, y, color);
        }
        if (r > 2) {
            LCD_DrawLine(lcd, x, y, x + (int16_t)x0, y - (int16_t)x0, color);
        }
        if (x0 == fx) {
            LCD_DrawPoint(lcd, x + (int16_t)x0, y - (int16_t)x0, color);
        } else {
            LCD_DrawPoint(lcd, x + (int16_t)x0, y - (int16_t)fx, color);
            LCD_DrawPoint(lcd, x + (int16_t)fx, y - (int16_t)x0, color);
            LCD_DrawPoint(lcd, x + (int16_t)x0, y - (int16_t)x0, color);
        }
    }
    if (section & LCD_CIRCLE_UPPER_LEFT) {
        if (r > 1) {
            LCD_DrawLine(lcd, x - (int16_t)r, y, x, y, color);
            LCD_DrawLine(lcd, x, y - (int16_t)r, x, y, color);
        }
        if (r > 2) {
            LCD_DrawLine(lcd, x, y, x - (int16_t)x0, y - (int16_t)x0, color);
        }
        if (x0 == fx) {
            LCD_DrawPoint(lcd, x - (int16_t)x0, y - (int16_t)x0, color);
        } else {
            LCD_DrawPoint(lcd, x - (int16_t)x0, y - (int16_t)fx, color);
            LCD_DrawPoint(lcd, x - (int16_t)fx, y - (int16_t)x0, color);
            LCD_DrawPoint(lcd, x - (int16_t)x0, y - (int16_t)x0, color);
        }
    }
    if (section & LCD_CIRCLE_LOWER_LEFT) {
        if (r > 1) {
            LCD_DrawLine(lcd, x - (int16_t)r, y, x, y, color);
            LCD_DrawLine(lcd, x, y + (int16_t)r, x, y, color);
        }
        if (r > 2) {
            LCD_DrawLine(lcd, x, y, x - (int16_t)x0, y + (int16_t)x0, color);
        }
        if (x0 == fx) {
            LCD_DrawPoint(lcd, x - (int16_t)x0, y + (int16_t)x0, color);
        } else {
            LCD_DrawPoint(lcd, x - (int16_t)x0, y + (int16_t)fx, color);
            LCD_DrawPoint(lcd, x - (int16_t)fx, y + (int16_t)x0, color);
            LCD_DrawPoint(lcd, x - (int16_t)x0, y + (int16_t)x0, color);
        }
    }
    if (section & LCD_CIRCLE_LOWER_RIGHT) {
        if (r > 1) {
            LCD_DrawLine(lcd, x + (int16_t)r, y, x, y, color);
            LCD_DrawLine(lcd, x, y + (int16_t)r, x, y, color);
        }
        if (r > 2) {
            LCD_DrawLine(lcd, x, y, x + (int16_t)x0, y + (int16_t)x0, color);
        }
        if (x0 == fx) {
            LCD_DrawPoint(lcd, x + (int16_t)x0, y + (int16_t)x0, color);
        } else {
            LCD_DrawPoint(lcd, x + (int16_t)x0, y + (int16_t)fx, color);
            LCD_DrawPoint(lcd, x + (int16_t)fx, y + (int16_t)x0, color);
            LCD_DrawPoint(lcd, x + (int16_t)x0, y + (int16_t)x0, color);
        }
    }
    if (section == LCD_CIRCLE_DRAW_ALL) {
        LCD_DrawLine(lcd, x + (int16_t)r, y, x, y, color);
        LCD_DrawLine(lcd, x - (int16_t)r, y, x, y, color);
        LCD_DrawLine(lcd, x, y - (int16_t)r, x, y, color);
        LCD_DrawLine(lcd, x, y + (int16_t)r, x, y, color);
        LCD_DrawPoint(lcd, x, y, color);
    }
}

void LCD_ShowChar(LCD_t *lcd, int16_t x, int16_t y, uint8_t chr, uint16_t fc, uint8_t sizey)
{
    uint8_t temp;
    uint8_t sizex;
    uint8_t t;
    uint16_t i;
    uint16_t TypefaceNum;
    int16_t x0 = x;

    sizex = (uint8_t)(sizey / 2);
    TypefaceNum = (uint16_t)((sizex / 8 + ((sizex % 8) ? 1 : 0)) * sizey);
    chr = (uint8_t)(chr - ' ');
    for (i = 0; i < TypefaceNum; i++) {
        if (sizey == 12) {
            temp = ascii_1206[chr][i];
        } else if (sizey == 16) {
            temp = ascii_1608[chr][i];
        } else if (sizey == 24) {
            temp = ascii_2412[chr][i];
        } else if (sizey == 32) {
            temp = ascii_3216[chr][i];
        } else {
            return;
        }
        for (t = 0; t < 8; t++) {
            if (temp & (uint8_t)(0x01 << t)) {
                LCD_DrawPoint(lcd, x, y, fc);
            }
            x++;
            if ((x - x0) == sizex) {
                x = x0;
                y++;
                break;
            }
        }
    }
}

void LCD_ShowString(LCD_t *lcd, int16_t x, int16_t y, const uint8_t *str, uint16_t fc, uint8_t sizey)
{
    while (*str != '\0') {
        if (*str == '\r') {
            x = 0;
            str++;
            continue;
        } else if (*str == '\n') {
            y += sizey;
            str++;
            continue;
        }
        LCD_ShowChar(lcd, x, y, *str, fc, sizey);
        x += sizey / 2;
        str++;
    }
}

void LCD_ShowText(LCD_t *lcd, int16_t x, int16_t y, uint16_t fc, uint8_t sizey, const char *fmt, ...)
{
    char strFormatted[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(strFormatted, sizeof(strFormatted), fmt, args);
    va_end(args);
    LCD_ShowString(lcd, x, y, (const uint8_t *)strFormatted, fc, sizey);
}

void LCD_ShowTextAlignRight(LCD_t *lcd, int16_t x, int16_t y, uint16_t fc, uint8_t sizey, int totalWidth, const char *fmt, ...)
{
    char strFormatted[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(strFormatted, sizeof(strFormatted), fmt, args);
    va_end(args);
    int strWidth = (int)(sizey * (int)strlen(strFormatted));
    int16_t startX = (int16_t)(x - strWidth);
    if (totalWidth > 0) {
        startX = (int16_t)(x + totalWidth - strWidth);
    }
    LCD_ShowString(lcd, startX, y, (const uint8_t *)strFormatted, fc, sizey);
}

void LCD_ShowTextCentered(LCD_t *lcd, int16_t x, int16_t y, uint16_t fc, uint8_t sizey, const char *fmt, ...)
{
    char strFormatted[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(strFormatted, sizeof(strFormatted), fmt, args);
    va_end(args);
    int strWidth = (int)(sizey * (int)strlen(strFormatted));
    LCD_ShowString(lcd, (int16_t)(x - strWidth / 2), y, (const uint8_t *)strFormatted, fc, sizey);
}

void LCD_ShowPicture(LCD_t *lcd, int16_t x, int16_t y, uint16_t width, uint16_t height, const uint8_t pic[])
{
    uint32_t k = 0;
    for (uint16_t i = 0; i < height; i++) {
        for (uint16_t j = 0; j < width; j++) {
            uint16_t color = (uint16_t)((uint16_t)pic[k * 2] << 8) | pic[k * 2 + 1];
            LCD_DrawPoint(lcd, (int16_t)(x + j), (int16_t)(y + i), color);
            k++;
        }
    }
}

static uint16_t LCD_BilinearInterpolate(const uint8_t *pic, int srcWidth, int srcHeight, float x, float y)
{
    int x1 = (int)x;
    int y1 = (int)y;
    int x2 = (x1 + 1) < (srcWidth - 1) ? (x1 + 1) : (srcWidth - 1);
    int y2 = (y1 + 1) < (srcHeight - 1) ? (y1 + 1) : (srcHeight - 1);
    uint16_t Q11 = (uint16_t)((uint16_t)pic[(y1 * srcWidth + x1) * 2] << 8) | pic[(y1 * srcWidth + x1) * 2 + 1];
    uint16_t Q21 = (uint16_t)((uint16_t)pic[(y1 * srcWidth + x2) * 2] << 8) | pic[(y1 * srcWidth + x2) * 2 + 1];
    uint16_t Q12 = (uint16_t)((uint16_t)pic[(y2 * srcWidth + x1) * 2] << 8) | pic[(y2 * srcWidth + x1) * 2 + 1];
    uint16_t Q22 = (uint16_t)((uint16_t)pic[(y2 * srcWidth + x2) * 2] << 8) | pic[(y2 * srcWidth + x2) * 2 + 1];

    float x2_x = (float)x2 - x;
    float x_x1 = x - (float)x1;
    float y2_y = (float)y2 - y;
    float y_y1 = y - (float)y1;

    uint16_t R1 = (uint16_t)(Q11 * x2_x + Q21 * x_x1);
    uint16_t R2 = (uint16_t)(Q12 * x2_x + Q22 * x_x1);
    uint16_t P = (uint16_t)(R1 * y2_y + R2 * y_y1);

    return P;
}

void LCD_ShowResizedPicture(LCD_t *lcd, int16_t x, int16_t y, uint16_t srcWidth, uint16_t srcHeight, uint16_t dstWidth, uint16_t dstHeight,
                            const uint8_t pic[])
{
    float x_ratio = (float)srcWidth / (float)dstWidth;
    float y_ratio = (float)srcHeight / (float)dstHeight;

    for (uint16_t i = 0; i < dstHeight; i++) {
        for (uint16_t j = 0; j < dstWidth; j++) {
            float srcX = j * x_ratio;
            float srcY = i * y_ratio;
            uint16_t color = LCD_BilinearInterpolate(pic, (int)srcWidth, (int)srcHeight, srcX, srcY);
            LCD_DrawPoint(lcd, (int16_t)(x + j), (int16_t)(y + i), color);
        }
    }
}

void LCD_ShowBinaryPicture(LCD_t *lcd, int16_t x, int16_t y, uint16_t width, uint16_t height, uint16_t color, const uint8_t pic[])
{
    uint32_t byteIndex = 0;
    uint8_t bitIndex = 0;

    for (uint16_t i = 0; i < height; i++) {
        for (uint16_t j = 0; j < width; j++) {
            if (pic[byteIndex] & (uint8_t)(0x80 >> bitIndex)) {
                LCD_DrawPoint(lcd, (int16_t)(x + j), (int16_t)(y + i), color);
            }
            bitIndex++;
            if (bitIndex == 8) {
                bitIndex = 0;
                byteIndex++;
            }
        }
    }
}

static uint8_t LCD_NearestNeighborInterpolate(const uint8_t *pic, int srcWidth, int srcHeight, float x, float y)
{
    int nearestX = (int)(x + 0.5f);
    int nearestY = (int)(y + 0.5f);

    if (nearestX >= srcWidth) {
        nearestX = srcWidth - 1;
    }
    if (nearestY >= srcHeight) {
        nearestY = srcHeight - 1;
    }
    if (nearestX < 0) {
        nearestX = 0;
    }
    if (nearestY < 0) {
        nearestY = 0;
    }

    int byteIndex = (nearestY * srcWidth + nearestX) / 8;
    int bitIndex = 7 - (nearestX % 8);

    return (pic[byteIndex] & (uint8_t)(1 << bitIndex)) ? 1U : 0U;
}

void LCD_ShowResizedBinaryPicture(LCD_t *lcd, int16_t x, int16_t y, uint16_t srcWidth, uint16_t srcHeight, uint16_t dstWidth, uint16_t dstHeight,
                                  uint16_t color, const uint8_t pic[])
{
    float x_ratio = (float)srcWidth / (float)dstWidth;
    float y_ratio = (float)srcHeight / (float)dstHeight;

    for (uint16_t i = 0; i < dstHeight; i++) {
        for (uint16_t j = 0; j < dstWidth; j++) {
            float srcX = j * x_ratio;
            float srcY = i * y_ratio;
            uint8_t pixel = LCD_NearestNeighborInterpolate(pic, (int)srcWidth, (int)srcHeight, srcX, srcY);
            if (pixel) {
                LCD_DrawPoint(lcd, (int16_t)(x + j), (int16_t)(y + i), color);
            }
        }
    }
}
