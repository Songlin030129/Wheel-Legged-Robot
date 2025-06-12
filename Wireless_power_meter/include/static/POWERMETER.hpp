/*
 * @LastEditors: qingmeijiupiao
 * @Description: 功率表相关部分
 * @Author: qingmeijiupiao
 * @LastEditTime: 2024-12-10 16:34:12
 */
#ifndef POWERMETER_HPP
#define POWERMETER_HPP
#include "INA226.h"
#include "FixedSizeQueue.hpp"
#include "static/HXCthread.hpp"
#include "HXC_NVS.hpp"
// 输入电压电流读取相关
namespace POWERMETER {
    HXC::NVS_DATA<float> sample_resistance("resistance",2);// NVS数据，采样电阻的值，默认为2mΩ
    constexpr int READ_HZ = 100;                  // 读取电压电流的频率
    constexpr int data_save_time=5;              // 保存数据的时间 秒
    INA226 PowerSensor(0x40);                 // 创建INA226传感器对象
    float voltage = 0;                       // 电压
    float current = 0;                       // 电流
    float output_mah = 0;                    // 输出的毫安时
    float output_mwh = 0;                    // 输出的毫瓦时
    float MAX_CURRENT = 0;                    // 最大电流
    float MAX_VOLTAGE = 0;                    // 最大电压
    uint32_t last_time = millis();             // 上次读取的时间
    FixedSizeQueue<float,READ_HZ*data_save_time> voltage_queue;         // 电压队列
    FixedSizeQueue<float,READ_HZ*data_save_time> current_queue;         // 电流队列
    FixedSizeQueue<float,READ_HZ*data_save_time> power_queue;         // 功率队列

    // 初始化
    void setup(){
        Wire.setPins(5, 4);                   // 设置I2C引脚
        Wire.begin();                         // 初始化I2C
        if (!PowerSensor.begin()) {           // 检查INA226是否连接
            Serial.println("INA226 sensor not found!");
        }
    };

    // 更新电压电流的任务
    HXC::thread<void> updatePower_thread([]() {
        // 初始化
        POWERMETER::setup();
        while (true) {
            voltage = PowerSensor.getBusVoltage(); // 读取电压
            uint16_t row_data = PowerSensor.getRegister(1); // 读取寄存器中的电流数据
            last_time = millis();              // 更新上次读取时间

            // 处理电流数据
            int16_t data = *(int16_t*)&row_data;
            data = data > 0 ? data : -data;

            current = double(data) *sample_resistance* 5e-4;     // 转换电流数据
            // 更新最大电压和最大电流
            if (voltage > MAX_VOLTAGE) MAX_VOLTAGE = voltage;
            if (current > MAX_CURRENT) MAX_CURRENT = current;

            // 计算输出的毫安时和毫瓦时
            output_mah += current * (millis() - last_time) / 3600.0;
            output_mwh += voltage * current * (millis() - last_time) / 3600.0;

            // 更新电压队列
            voltage_queue.push(voltage);
            // 更新电流队列
            current_queue.push(current);
            // 更新功率队列
            power_queue.push(voltage * current);

            delay(1000 / READ_HZ);             // 延时，保持读取频率
        }
    });
}
#endif