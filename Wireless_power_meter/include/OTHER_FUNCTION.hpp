/*
 * @LastEditors: qingmeijiupiao
 * @Description: 其他功能，如串口打印，过流保护，过放保护
 * @Author: qingmeijiupiao
 * @Date: 2024-10-13 17:16:26
 */

#ifndef OTHER_FUNCTION_HPP
#define OTHER_FUNCTION_HPP
#include <Arduino.h>
#include "static/POWERMETER.hpp"
#include "static/powerctrl.hpp"
extern POWERCTRL_t power_output;
namespace OTHER_FUNCTION{
    /*#############串口打印######################*/
    // 串口打印数据
    TaskHandle_t serial_print_handle = nullptr;
    void serial_print_data_task(void* pvParameters) {
        while (true)
        {
        Serial.print("V&A:");// 输出电压:%f
        Serial.print(POWERMETER::voltage);
        Serial.print(",");
        // 输出电流:%f
        Serial.println(POWERMETER::current);
        delay(10);
        }
    }
    // 串口打印数据控制
    void serial_print_ctrl(bool state) {
        if (state&&serial_print_handle==nullptr) {
            xTaskCreate(serial_print_data_task, "serial_print_data", 2048, NULL, 5, &serial_print_handle);
        }else{
            if(serial_print_handle!=nullptr){
                vTaskDelete(serial_print_handle);
                serial_print_handle = nullptr;
            }
        }
    }
    /*#############串口打印######################*/

    /*#############电流保护######################*/
    float current_protect_value = 30;// 电流保护值,A
    TaskHandle_t current_protect_handle = nullptr;
    void current_protect_task(void* pvParameters) {
        while (true)
        {
            if(POWERMETER::current>current_protect_value){
                power_output.off();
            }
            delay(10);
        }
    }
    // 开关电流保护
    void current_protect_ctrl(bool state) {
        if (state&&current_protect_handle==nullptr) {
            xTaskCreate(current_protect_task, "current_protect", 2048, NULL, 5, &current_protect_handle);
        }else{
            if(current_protect_handle!=nullptr){
                vTaskDelete(current_protect_handle);
                current_protect_handle = nullptr;
            }
        }
    }
    /*#############电流保护######################*/


    /*#############低电压保护######################*/
    float voltage_protect_value = 12;// 电压保护值,V
    TaskHandle_t voltage_protect_task_handle= nullptr;
    void voltage_protect_task(void* p){
        while (true){
            if(POWERMETER::voltage_queue.get_average()<voltage_protect_value){//这里求平均防止突然的大负载压降关断
                power_output.off();
            }
            delay(10);
        }
    }
    // 开关低压保护
    void voltage_protect_ctrl(bool state) {
        if (state&&voltage_protect_task==nullptr) {
            xTaskCreate(voltage_protect_task, "voltage_protect", 2048, NULL, 5, &voltage_protect_task_handle);
        }else{
            if(voltage_protect_task!=nullptr){
                vTaskDelete(voltage_protect_task);
                voltage_protect_task_handle = nullptr;
            }
        }
    }
    /*#############低电压保护######################*/
    
}
#endif