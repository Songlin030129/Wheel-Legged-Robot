#include <Arduino.h>
#include "static/PINS.h"
#include "static/powerctrl.hpp"// 电源控制
#include "static/TemperatureSensor.hpp"// 温度传感器
#include "static/buzz.hpp"// 蜂鸣器
#include "BUTTONS.hpp"// 按键
#include "SCREEN.hpp"// 屏幕
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

// 初始化各种模块
POWERCTRL_t power_output(PowerPin);           // 电源控制
TemperatureSensor_t Temperature_sensor;       // 温度传感器
BUZZ_t buzz(BUZZER_PIN);                      // 蜂鸣器
XboxSeriesXControllerESP32_asukiaaa::Core
xboxController("ac:8e:bd:24:15:2a");


String xbox_string()
{
    String str =
        String(xboxController.isConnected()) + "," +
        String(xboxController.xboxNotif.btnA) + "," +
        String(xboxController.xboxNotif.btnB) + "," +
        String(xboxController.xboxNotif.btnX) + "," +
        String(xboxController.xboxNotif.btnY) + "," +
        String(xboxController.xboxNotif.btnLB) + "," +
        String(xboxController.xboxNotif.btnRB) + "," +
        String(xboxController.xboxNotif.btnSelect) + "," +
        String(xboxController.xboxNotif.btnStart) + "," +
        String(xboxController.xboxNotif.btnXbox) + "," +
        String(xboxController.xboxNotif.btnShare) + "," +
        String(xboxController.xboxNotif.btnLS) + "," +
        String(xboxController.xboxNotif.btnRS) + "," +
        String(xboxController.xboxNotif.btnDirUp) + "," +
        String(xboxController.xboxNotif.btnDirDown) + "," +
        String(xboxController.xboxNotif.btnDirLeft) + "," +
        String(xboxController.xboxNotif.btnDirRight) + "," +
        String((int)xboxController.xboxNotif.joyLHori - 32768) + "," +
        String((int)xboxController.xboxNotif.joyLVert - 32768) + "," +
        String((int)xboxController.xboxNotif.joyRHori - 32768) + "," +
        String((int)xboxController.xboxNotif.joyRVert - 32768) + "," +
        String(xboxController.xboxNotif.trigLT) + "," +
        String(xboxController.xboxNotif.trigRT) + "\n";
    return str;
};

// 初始化程序
void setup() {
    Serial.begin(115200); // 初始化串口
    /*↓↓↓↓↓↓↓初始化相关外设↓↓↓↓↓↓↓*/
    Temperature_sensor.setup();//温度传感器初始化
    power_output.setup();//电源控制初始化
    power_output.on();//插电默认关闭电源
    buzz.setup();//蜂鸣器初始化
    xboxController.begin();

    /*初始化相关外设*/

    /*↓↓↓↓↓↓↓创建后台任务↓↓↓↓↓↓↓*/
    BUTTON::button_detect_thread.start("button_detect",/*stacksize=*/512);// 按键任务
    POWERMETER::updatePower_thread.start("updatePower");// 电压电流更新任务
    SCREEN::updatescreen_thread.start("updatescreen",/*stacksize=*/8192);// 屏幕更新任务
    /*↑↑↑↑↑↑↑↑创建后台任务↑↑↑↑↑↑*/

    delay(2000);// 延时防止重启
    Serial0.begin(115200);

}

// Arduino主循环,本项目不使用
void loop()
{
    xboxController.onLoop();
    if (xboxController.isConnected())
    {
        if (xboxController.isWaitingForFirstNotification())
        {
            Serial.println("waiting for first notification");
        }
        else
        {
            Serial.print(xbox_string());
            // demoVibration();
            // demoVibration_2();
        }
    }
    else
    {
        Serial.println("not connected");
        if (xboxController.getCountFailedConnection() > 2)
        {
            ESP.restart();
        }
    }
    Serial0.print(xbox_string());
    delay(20);
}
