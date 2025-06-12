/*
 * @LastEditors: qingmeijiupiao
 * @Description: 按键相关
 * @Author: qingmeijiupiao
 * @LastEditTime: 2024-11-28 12:24:01
 */

#ifndef BUTTONS_HPP
#define BUTTONS_HPP
#include "static/PINS.h"
#include "OneButton.h"
#include "SCREEN.hpp"
#include "static/HXCthread.hpp"
OneButton right_button(BUT1, true);           // 右按键
OneButton left_button(BUT2, true);            // 左按键


// 按键控制相关
namespace BUTTON {
    constexpr int BUTTON_tick_frc = 100; // 按键检测频率 HZ

    // 按键初始化
    void setup() {
        pinMode(BUT1, INPUT_PULLUP);                               // 设置按键为上拉输入模式
        pinMode(BUT2, INPUT_PULLUP);

        // 定义按下中间按键时的操作（关闭电源）
        auto pressed_func = []() {
            power_output.off();
            };

        // 定义长按中间按键时的操作（打开电源）
        auto longpress_func = []() {
            power_output.on();
            };

        // 定义左按键按下时的操作（切换到上一个页面）
        auto left_button_pressed_func = []() {
            if (SCREEN::now_page == SCREEN::page_list.begin()) {
                SCREEN::now_page = --SCREEN::page_list.end(); // 如果当前是第一个页面，跳到最后一个页面
            }
            else {
                --SCREEN::now_page;
            }
            };

        auto left_button_long_pressed_func = []() {// 左按键长按旋转屏幕
            if (SCREEN::screen_rotation == 3) {
                SCREEN::screen_rotation = 1;
            }
            else {
                SCREEN::screen_rotation = 3;
            }
            SCREEN::tft.setRotation(SCREEN::screen_rotation);
            };


        // 给按键绑定相应的功能
        right_button.attachPress(pressed_func);
        right_button.attachLongPressStart(longpress_func);
        right_button.setLongPressIntervalMs(1000); // 设置长按时间为1秒

        left_button.attachClick(left_button_pressed_func); // 左按键点击切换页面
        left_button.attachLongPressStart(left_button_long_pressed_func);// 左按键长按旋转屏幕
        left_button.setLongPressIntervalMs(1000);

    }
    // 按键任务
    HXC::thread<void> button_detect_thread([] {
        BUTTON::setup();
        while (true) {
            left_button.tick(); // 检测按键状态
            right_button.tick();
            delay(1000 / BUTTON_tick_frc); // 控制检测频率
        }
        });
}



#endif
