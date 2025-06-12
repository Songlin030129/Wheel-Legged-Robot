#ifndef __XBOXCONTROLLER_H__
#define __XBOXCONTROLLER_H__

#include "common_inc.h"
#include <string>
#include <vector>
#define XBOXCONTROLLER_RX_BUFFER_SIZE_MAX 512

#define KEY_ClickThreshold 20             // 按键按下时间阈值
#define KEY_HoldThreshold 600             // 按键长按时间阈值
#define KEY_IntervalThreshold 200         // 按键松开后再次按下时间间隔阈值
#define KEY_HoldTriggerFirstThreshold 600 // 按键长按时间阈值
#define KEY_HoldTriggerThreshold 100      // 按键长按持续触发时间间隔


class KEY
{
public:
    enum states
    {
        Release = 0,
        PrePress,
        Prelong,
        LongHold,
        MultiClick
    }; // 状态枚举
    int val;
    states state;
    uint32_t pressTimer;    // 按下计时
    uint32_t intervalTimer; // 放开计时
    int16_t triggerTimer;   // 长按触发计时
    uint32_t holdTime;      // 长按计时
    uint8_t clickCnt;       // 按下计数
};

class XboxController
{
public:
    void Init(UART_HandleTypeDef* huart);
    void UartReceive_IDLE_DMA_Callback(UART_HandleTypeDef* huart, uint16_t Size);

    int connected;

    int joyLHori, joyLVert, joyRHori, joyRVert;
    int trigLT, trigRT;

    KEY btnA, btnB, btnX, btnY,
        btnShare, btnStart, btnSelect, btnXbox,
        btnLB, btnRB,
        btnLS, btnRS,
        btnDirUp, btnDirLeft, btnDirRight, btnDirDown;

    void AddKey(KEY* key);
    void KeysHandler();

private:
    UART_HandleTypeDef* huart;
    uint8_t RxBuffer[XBOXCONTROLLER_RX_BUFFER_SIZE_MAX];

    std::vector<KEY*> keys;
    uint8_t keyCnt = 0;
    uint32_t nowTime, lastTime, deltaTime;

};
extern XboxController xbox;


/**
 * @brief 按键按下回调函数
 *
 */
void KEY_KeyClickCallback(KEY* key);
/**
 * @brief 按键多击回调函数
 *
 */
void KEY_MultipleClickCallback(KEY* key);
/**
 * @brief 按键长按回调函数
 *
 */
void KEY_LongHoldCallback(KEY* key);
/**
 * @brief 按键长按触发回调函数
 *
 */
void KEY_HoldTriggerCallback(KEY* key);


#endif