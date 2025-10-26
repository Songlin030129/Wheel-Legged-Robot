#include "LCD_Task.h"
#include "Control_Task.h"
#include "GATT.h"
#include "INA_Task.h"
#include "lcd.h"

static const char *TAG = "LCD_Task";
void LCD_Task()
{
    st7789_init(&st7789, SPI2_HOST, 10 * 1000 * 1000, 10, 2, 3, -1);
    LCD_LinkST7789(&tft_lcd, &st7789);
    while (1) {
        LCD_ClearBuffer(&tft_lcd);
        LCD_ShowText(&tft_lcd, 0, 0, LCD_COLOR_WHITE, 16, "Vsh=%.6f V, Vbus=%.3f V", vsh, vbus);
        LCD_ShowText(&tft_lcd, 0, 16, LCD_COLOR_WHITE, 16, "I=%.3f A, P=%.3f W", cur, power);
        LCD_ShowText(&tft_lcd, 0, 32, LCD_COLOR_WHITE, 16, "Output:%d", output_state);
        LCD_ShowText(&tft_lcd, 0, 48, LCD_COLOR_WHITE, 16, "Xbox:%d, %d", g_output.joyLHori, g_output.joyLVert);

        LCD_SendBuffer(&tft_lcd);
        vTaskDelay(1);
    }
}