/*
 * @LastEditors: qingmeijiupiao
 * @Description: 输出控制相关
 * @Author: qingmeijiupiao
 * @LastEditTime: 2024-11-17 23:36:49
 */
#ifndef POWERCTRL_HPP
#define POWERCTRL_HPP
#include <Arduino.h>
#include "buzz.hpp"
extern BUZZ_t buzz;
// 电源控制类型
class POWERCTRL_t {
    public:
    void setup(bool state=false) {
        pinMode(pin, OUTPUT);
        power = state;
        digitalWrite(pin, power);
    }
    POWERCTRL_t(uint8_t pin) {
        this->pin = pin;
    }

    void on() {
        if (millis() - last_change_time < delaytime &&last_change_time !=0 ) {
            last_change_time = millis();
            return;
        }
        last_change_time = millis();
        power = true;
        digitalWrite(pin, power);
    }

    void off() {
        power = false;
        if (millis() - last_change_time < delaytime) {
            last_change_time = millis();
            return;
        }
        last_change_time = millis();
        power = false;
        digitalWrite(pin, power);
    }
    bool getstate() { return power; }
    private:
    uint8_t pin;
    int last_change_time = 0;
    bool power = false;
    int delaytime=300;
};
#endif