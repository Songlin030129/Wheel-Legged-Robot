/*
 * @LastEditors: qingmeijiupiao
 * @Description: 屏幕显示相关
 * @Author: qingmeijiupiao
 * @Date: 2024-10-13 17:24:35
 */
#ifndef SCREEN_HPP
#define SCREEN_HPP
#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <list> 


#include "static/HXC_NVS.hpp"
#include "static/POWERMETER.hpp"
#include "static/powerctrl.hpp"
#include "WIRELESSCTRL.hpp"
#include "static/TemperatureSensor.hpp"

#include "img/wifi_img.hpp"
#include "img/robocon_img.hpp"
#include "img/CQUPTHXC_img.hpp"
#include "img/JLC_LOGO.hpp"
extern TemperatureSensor_t Temperature_sensor;
extern POWERCTRL_t power_output;
// 格式化浮点数为固定长度的字符串
void sprintf_float(double value, char* buffer, int len) {
    int max = pow(10, len) - 1;               // 最大值，根据长度决定
    if (value > max) value = max;
    if (value < 0) value = 0;

    int count = 0;
    int temp = value;                            // 记录小数点前的位数
    while (temp > 10) {
        temp /= 10;
        count++;
    }

    // 构建格式化字符串
    String format = "%";
    if (count == len) {
        format += "d";
    }
    else {
        format += "." + String(len - count) + "f";
    }

    sprintf(buffer, format.c_str(), value);   // 格式化浮点数
}

// 屏幕显示相关
namespace SCREEN {
    constexpr int SCREEN_HZ = 60;                 // 屏幕刷新率
    uint8_t screen_rotation = 3;                         // 屏幕旋转方向,1 OR 3
    using page_func = std::function<void()>;
    // 页面列表
    std::list<page_func> page_list;                    // 页面列表
    std::list<page_func>::iterator now_page = page_list.begin(); // 当前页面


    TFT_eSPI tft = TFT_eSPI();                // 创建屏幕对象
    TFT_eSprite clk = TFT_eSprite(&tft);      // 创建缓冲区

    constexpr uint16_t border_color = TFT_YELLOW;    //边框颜色


    /*↓↓↓↓↓↓↓↓页面函数区域↓↓↓↓↓↓↓↓*/
    // 主页面绘制函数
    void mian_page() {
        constexpr uint16_t default_color = TFT_WHITE; // 保存默认颜色
        constexpr uint16_t voltage_color = TFT_RED;   // 电压颜色
        constexpr uint16_t current_color = TFT_GREEN;  // 电流颜色
        constexpr uint16_t power_color = TFT_BLUE;    // 功率颜色
        constexpr uint16_t line_color = border_color; // 分割线颜色

        clk.setTextColor(default_color); // 默认文本颜色

        // 电压部分
        auto draw_voltage = []() {
            clk.setTextColor(voltage_color);
            clk.setCursor(10, 10);
            char buffer[4];
            sprintf_float(POWERMETER::voltage, buffer, 3); // 格式化电压数据
            clk.setTextFont(6);
            clk.setTextSize(1);
            clk.print(buffer);
            clk.setTextFont(1);
            clk.setTextSize(5);
            clk.print("V");
            };draw_voltage(); // 调用电压绘制函数

        // 电流部分
        auto draw_current = []() {
            clk.setTextColor(current_color);
            char buffer[4];
            sprintf_float(POWERMETER::current, buffer, 3); // 格式化电流数据
            clk.setCursor(10, 60);
            clk.setTextFont(6);
            clk.setTextSize(1);
            clk.print(buffer);
            clk.setTextFont(1);
            clk.setTextSize(5);
            clk.print("A");
            };draw_current(); // 调用电流绘制函数

        // 功率部分
        auto draw_power = []() {
            clk.setTextColor(power_color);
            char buffer[4];
            sprintf_float(POWERMETER::voltage * POWERMETER::current, buffer, 3); // 格式化功率数据
            clk.setCursor(10, 110);
            clk.setTextFont(4);
            clk.setTextSize(1);
            clk.print(buffer);
            clk.print("W");
            };draw_power(); // 调用功率绘制函数

        clk.setTextColor(default_color);
        // 当前开关状态部分“<——”
        auto draw_arrow = []() {
            uint16_t color = power_output.getstate() ? TFT_GREEN : TFT_RED; // 设置颜色，关是红色，开是绿色
            clk.drawLine(104, 120 - 1, 120, 120 - 11, color); // 画箭头上
            clk.drawLine(104, 120, 120, 120 - 10, color);

            clk.drawLine(105, 119, 150, 119, color);
            clk.drawLine(105, 120, 150, 120, color); // 画箭头中心
            clk.drawLine(105, 121, 150, 121, color);

            clk.drawLine(104, 120, 120, 120 + 10, color); // 画箭头下
            clk.drawLine(104, 120 + 1, 120, 120 + 11, color);
            if (!power_output.getstate()) { // 如果是关的话
                // 画个X
                clk.drawLine(135 - 10, 120 + 11, 135 + 10, 120 - 11, color);
                clk.drawLine(135 - 10, 120 - 11, 135 + 10, 120 + 11, color);
            }
            };draw_arrow(); // 调用箭头绘制函数

        // 绘制温度
        auto draw_temperature = []() {
            clk.setCursor(170, 10);
            clk.setTextFont(4);
            clk.setTextSize(1);
            clk.print(int(Temperature_sensor.getTemperature())); // 显示温度
            clk.print("  C"); // 显示单位摄氏度
            clk.drawCircle(203, 14, 3, default_color); // 显示小圆圈作为摄氏度符号
            };
        draw_temperature(); // 调用温度绘制函数

        // 最大电压和电流显示
        auto draw_max_values = []() {
            char buffer[4];
            clk.setCursor(170, 30);
            clk.print("MAX:");

            clk.setCursor(170, 50);
            clk.setTextColor(TFT_RED);
            sprintf_float(POWERMETER::MAX_VOLTAGE, buffer, 2);
            clk.print(buffer);
            clk.setTextColor(default_color);
            clk.print("V");

            clk.setCursor(170, 70);
            clk.setTextColor(TFT_GREEN);
            sprintf_float(POWERMETER::MAX_CURRENT, buffer, 2);
            clk.print(buffer);
            clk.setTextColor(default_color);
            clk.print("A");
            };
        draw_max_values(); // 调用最大值绘制函数

        // 连接图标
        auto draw_connection_icon = []() {
            if (is_conect) {
                clk.pushImage(180, 90, __WIFI_WIDTH, __WIFI_HEIGHT, __wifi);
            }
            else {
                clk.pushImage(180, 90, __CLOSE_WIFI_WIDTH, __CLOSE_WIFI_HEIGHT, __close_wifi);
            }
            };
        draw_connection_icon(); // 调用连接图标绘制函数

        // 绘制分割线
        for (int i = 0; i < 3; i++) {
            clk.drawLine(162 + i, 0, 162 + i, 135, TFT_YELLOW); // 分割线用黄色
        }
        clk.setTextColor(TFT_WHITE);
    }

    // 累计电量页面绘制函数
    void amperage_page() {
        constexpr uint16_t default_color = TFT_WHITE; // 默认颜色
        constexpr uint16_t mah_color = TFT_RED;       // mAH颜色
        constexpr uint16_t mwh_color = TFT_GREEN;     // mWH颜色
        constexpr uint16_t time_color = TFT_BLUE;     // 时间颜色

        // 显示累计毫安时
        auto draw_output_mah = []() {
            char buffer[4];
            sprintf_float(POWERMETER::output_mah, buffer, 4); // 显示累计毫安时
            clk.setCursor(0, 15);
            clk.setTextFont(6);
            clk.setTextSize(1);
            clk.setTextColor(mah_color); // 设置颜色
            clk.print(buffer);
            clk.setTextFont(1);
            clk.setTextSize(5);
            clk.print("mAH");
            };draw_output_mah(); // 调用毫安时绘制函数

        // 显示累计毫瓦时
        auto draw_output_mwh = []() {
            char buffer[4];
            sprintf_float(POWERMETER::output_mwh, buffer, 4); // 显示累计毫瓦时
            clk.setCursor(0, 60);
            clk.setTextFont(6);
            clk.setTextSize(1);
            clk.setTextColor(mwh_color); // 设置颜色
            clk.print(buffer);
            clk.setTextFont(1);
            clk.setTextSize(5);
            clk.print("mWH");
            };draw_output_mwh(); // 调用毫瓦时绘制函数

        // 显示累计时间
        auto draw_runtime = []() {
            char buffer[4];
            int time_m = POWERMETER::last_time / 60000;
            int time_s = (POWERMETER::last_time % 60000) / 1000;
            int time_ms = POWERMETER::last_time % 100;
            clk.setCursor(20, 110);
            clk.setTextFont(4);
            clk.setTextSize(1);
            clk.setTextColor(time_color); // 设置颜色
            clk.print("Runtime:  ");
            sprintf(buffer, "%4d", time_m);
            clk.print(time_m);
            clk.print(":");
            sprintf(buffer, "%2d", time_s);
            clk.print(buffer);
            clk.print(":");
            sprintf(buffer, "%2d", time_ms);
            buffer[2] = '\0';
            clk.print(buffer);
            };draw_runtime(); // 调用运行时间绘制函数
    }


    // 曲线页面
    void curve_page() {
        // 曲线低点和高点高度
        constexpr float curve_maxi_height = 0.8;  // 0-1
        constexpr float curve_mini_height = 0.1; // 0-1
        constexpr int smooth_curve_level = 1;    // 平滑曲线的等级

        // 曲线点缓存
        static uint8_t power_curve_point[190];
        static uint8_t voltage_curve_point[190];
        static uint8_t current_curve_point[190];

        // 绘制背景
        auto draw_background = []() {
            for (int i = 50; i < 240; i += 10) {
                clk.drawLine(i, 0, i, 135, 0x3333);
            }
            for (int i = 0; i < 135; i += 10) {
                clk.drawLine(50, i, 240, i, 0x3333);
            }
            };draw_background(); // 调用背景绘制函数

        // MAX部分
        auto draw_max_values = []() {
            clk.setTextColor(TFT_WHITE);
            clk.setCursor(3, 0);
            clk.setTextFont(2);
            clk.setTextSize(1);
            clk.print("MAX->\n");

            clk.setTextColor(TFT_RED);
            clk.setCursor(3, 12);
            clk.print(POWERMETER::voltage_queue.get_max());
            clk.print("V\n");

            clk.setTextColor(TFT_GREEN);
            clk.setCursor(3, 24);
            clk.print(POWERMETER::current_queue.get_max());
            clk.print("A\n");

            clk.setTextColor(TFT_BLUE);
            clk.setCursor(3, 36);
            clk.print(POWERMETER::power_queue.get_max());
            clk.print("W\n");
            };draw_max_values(); // 调用最大值绘制函数

        // MIN部分
        auto draw_min_values = []() {
            clk.setTextColor(TFT_WHITE);
            clk.setCursor(3, 135 - 16);
            clk.setTextFont(2);
            clk.setTextSize(1);
            clk.print("MIN->\n");

            clk.setTextColor(TFT_RED);
            clk.setCursor(3, 70 + 12);
            clk.print(POWERMETER::voltage_queue.get_min());
            clk.print("V\n");

            clk.setTextColor(TFT_GREEN);
            clk.setCursor(3, 70 + 24);
            clk.print(POWERMETER::current_queue.get_min());
            clk.print("A\n");

            clk.setTextColor(TFT_BLUE);
            clk.setCursor(3, 70 + 36);
            clk.print(POWERMETER::power_queue.get_min());
            clk.print("W\n");
            };draw_min_values(); // 调用最小值绘制函数

        // 数据处理
        float step = POWERMETER::READ_HZ * POWERMETER::data_save_time / 190;
        memset(power_curve_point, 0, sizeof(power_curve_point));
        memset(voltage_curve_point, 0, sizeof(voltage_curve_point));
        memset(current_curve_point, 0, sizeof(current_curve_point));

        // 曲线缩放
        constexpr float curve_scale = 1 / (curve_maxi_height - curve_mini_height);

        // 保存为采样点
        auto process_curve_points = [&]() {
            for (int i = 0; i < 190; i++) {
                if (POWERMETER::power_queue.get_max() - POWERMETER::power_queue.get_min() != 0) {
                    float mini_var = POWERMETER::power_queue.get_min();
                    float data_range = POWERMETER::MAX_CURRENT * POWERMETER::MAX_VOLTAGE - mini_var;
                    power_curve_point[i] = 135 * (POWERMETER::power_queue.toArray()[int(i * step)] - mini_var) / (curve_scale * data_range);
                }

                if (POWERMETER::voltage_queue.get_max() - POWERMETER::voltage_queue.get_min() != 0) {
                    float mini_var = POWERMETER::voltage_queue.get_min();
                    float data_range = POWERMETER::MAX_VOLTAGE - mini_var;
                    voltage_curve_point[i] = 135 * (POWERMETER::voltage_queue.toArray()[int(i * step)] - mini_var) / (curve_scale * data_range);
                }

                if (POWERMETER::current_queue.get_max() - POWERMETER::current_queue.get_min() != 0) {
                    float mini_var = POWERMETER::current_queue.get_min();
                    float data_range = POWERMETER::MAX_CURRENT - mini_var;
                    current_curve_point[i] = 135 * (POWERMETER::current_queue.toArray()[int(i * step)] - mini_var) / (curve_scale * data_range);
                }
            }
            };process_curve_points(); // 处理曲线数据

        //平滑曲线
        auto smooth_curve_points = [&]() {
            for (int i = 0; i < smooth_curve_level; i++) {
                power_curve_point[0] = power_curve_point[1] + power_curve_point[0] / 2;
                voltage_curve_point[0] = voltage_curve_point[1] + voltage_curve_point[0] / 2;
                current_curve_point[0] = current_curve_point[1] + current_curve_point[0] / 2;
                for (int i = 1; i < 189; i++) {
                    power_curve_point[i] = (power_curve_point[i - 1] + power_curve_point[i] + power_curve_point[i + 1]) / 3;
                    voltage_curve_point[i] = (voltage_curve_point[i - 1] + voltage_curve_point[i] + voltage_curve_point[i + 1]) / 3;
                    current_curve_point[i] = (current_curve_point[i - 1] + current_curve_point[i] + current_curve_point[i + 1]) / 3;
                }
                power_curve_point[189] = power_curve_point[188] + power_curve_point[189] / 2;
                voltage_curve_point[189] = voltage_curve_point[188] + voltage_curve_point[189] / 2;
                current_curve_point[189] = current_curve_point[188] + current_curve_point[189] / 2;
            }
            };smooth_curve_points();



        // 调整曲线高度
        auto fix_curve_points = [&]() {
            // 调整曲线点位置
            for (int i = 0; i < 190; i++) {
                power_curve_point[i] += curve_mini_height * 135;
                voltage_curve_point[i] += curve_mini_height * 135;
                current_curve_point[i] += curve_mini_height * 135;
            }

            };fix_curve_points();


        // 绘制曲线
        auto draw_curves = []() {
            for (int i = 1; i < 190; i++) {
                clk.drawLine(i + 50, 133 - power_curve_point[i], i + 49, 133 - power_curve_point[i - 1], TFT_BLUE);// 功率曲线
                clk.drawLine(i + 50, 133 - voltage_curve_point[i], i + 49, 133 - voltage_curve_point[i - 1], TFT_RED);// 电压曲线
                clk.drawLine(i + 50, 133 - current_curve_point[i], i + 49, 133 - current_curve_point[i - 1], TFT_GREEN);// 电流曲线
            }
            };draw_curves(); // 调用曲线绘制函数

        clk.setTextColor(TFT_WHITE); // 重置文本颜色
    }


    void wireless_page() {
        clk.setCursor(10, 15);
        clk.setTextFont(4);
        clk.setTextSize(1);
        clk.print("self MAC:");
        clk.setCursor(10, 40);
        char macStr[18];
        sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", self_Macaddress[0], self_Macaddress[1], self_Macaddress[2], self_Macaddress[3], self_Macaddress[4], self_Macaddress[5]);
        clk.print(macStr);
        clk.setCursor(10, 70);

        MAC_t MAC = WIRELESSCTRL::pair_mac.read();

        if (MAC == broadcastMacAddress) {
            clk.setTextColor(TFT_RED);
            clk.print("broadcast mode");
        }
        else {
            clk.setTextColor(TFT_GREEN);
            clk.print("pair MAC:");
        }
        clk.setCursor(10, 95);

        sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", MAC[0], MAC[1], MAC[2], MAC[3], MAC[4], MAC[5]);
        clk.print(macStr);

        clk.setTextColor(TFT_WHITE);
    }

    // 模板页面
    // void temple_page() {
    //     clk.setCursor(10, 15);//设置坐标
    //     clk.setTextFont(4);//设置使用的默认字体序号
    //     clk.setTextSize(1);//设置字体大小
    //     clk.setTextColor(TFT_WHITE);
    //     clk.print("temple:");
    // }

    /*↑↑↑↑↑↑↑↑↑页面函数区域↑↑↑↑↑↑↑↑↑*/


    // 初始化
    void setup() {
        tft.init();                                                // 初始化屏幕
        tft.setRotation(screen_rotation);                          // 设置屏幕方向
        tft.fillScreen(TFT_BLACK);                                 // 清空屏幕

        /*下面添加页面函数，修改添加顺序可以修改显示顺序*/
        page_list.push_back(mian_page);                            // 添加主页面
        page_list.push_back(amperage_page);                        // 添加累计电量页面
        page_list.push_back(curve_page);                           // 添加曲线页面
        page_list.push_back(wireless_page);                        // 添加无线页面
        // page_list.push_back(temple_page);                         // 添加模板页面
        /*添加页面函数*/

        now_page = page_list.begin();                              // 初始化当前页面

        clk.createSprite(240, 135);                                // 创建缓冲区

        // 打印LOGO
        auto print_start_img = []() {
            //ROBOCON logo
            tft.startWrite();
            clk.fillSprite(TFT_BLACK);
            clk.pushImage(0, 0, ROBOCONIMG_WIDTH, ROBOCONIMG_HEIGHT, ROBOCONIMG);
            clk.pushSprite(0, 0);
            tft.startWrite();
            tft.endWrite();
            delay(750);
            // 重邮和HXC LOGO
            tft.startWrite();
            clk.fillSprite(TFT_BLACK);
            clk.pushImage(0, 0, CQUPTHXC_LOGO_WIDTH, CQUPTHXC_LOGO_HEIGHT, CQUPTHXC_LOGO);
            clk.pushSprite(0, 0);
            tft.endWrite();
            delay(750);

            // 嘉立创LOGO
            // tft.startWrite();
            // clk.fillSprite(TFT_BLACK);
            // clk.pushImage(0, 0, JLC_LOGO_WIDTH, JLC_LOGO_HEIGHT, JLC_LOGO);
            // clk.pushSprite(0, 0);
            // tft.endWrite();
            // delay(2000);

            };print_start_img();// 调用LOGO绘制函数
    }
    // 屏幕更新任务
    HXC::thread<void> updatescreen_thread([]() {
        SCREEN::setup();
        while (true) {
            tft.startWrite();
            clk.fillSprite(TFT_BLACK);                             // 清空缓冲区

            now_page->operator()();                                // 调用当前页面的绘制函数

            clk.drawRoundRect(0, 0, 240, 135, 0, border_color);      // 绘制矩形边框
            clk.pushSprite(0, 0);                                  // 将缓冲区内容推送到屏幕
            tft.endWrite();                                        // 结束写入
            delay(1000 / SCREEN_HZ);                               // 控制刷新率
        }
        });
}

#endif