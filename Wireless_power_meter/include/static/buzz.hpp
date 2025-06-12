/*
 * @LastEditors: qingmeijiupiao
 * @Description: 蜂鸣器相关
 * @Author: qingmeijiupiao
 * @LastEditTime: 2024-11-17 23:37:55
 */

#ifndef BUUZZ_HPP
#define BUUZZ_HPP
#include <Arduino.h>
class BUZZ_t {
    public:
    BUZZ_t(uint8_t _pin) {
        this->pin = _pin;
    }
    
    //初始化
    void setup(int pwmfrc=5000,uint8_t _pwm_channel=0,uint8_t _resolution=8) {
        ledcSetup(_pwm_channel, pwmfrc, _resolution);
        ledcWrite(_pwm_channel, 0);
        ledcAttachPin(pin, _pwm_channel);
        pwm_channel = _pwm_channel;
        resolution = _resolution;
    }
    //指定占空比
    void buzz(float value) {// 0~1
        if (value < 0) value = 0;
        if (value > 1) value = 1;
        int max = pow(2, resolution) - 1;
        ledcWrite(pwm_channel, value * max);
    }
    private:
        uint8_t pin;
        uint8_t pwm_channel;
        uint8_t resolution;

};
#endif