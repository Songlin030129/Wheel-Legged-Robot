#include "XboxController.h"
XboxController xbox;

void XboxController::Init(UART_HandleTypeDef* huart)
{
    this->huart = huart;
    HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*)this->RxBuffer, XBOXCONTROLLER_RX_BUFFER_SIZE_MAX);
    AddKey(&btnA);
    AddKey(&btnB);
    AddKey(&btnX);
    AddKey(&btnY);
    AddKey(&btnShare);
    AddKey(&btnStart);
    AddKey(&btnSelect);
    AddKey(&btnXbox);
    AddKey(&btnLB);
    AddKey(&btnRB);
    AddKey(&btnLS);
    AddKey(&btnRS);
    AddKey(&btnDirUp);
    AddKey(&btnDirLeft);
    AddKey(&btnDirRight);
    AddKey(&btnDirDown);
}
void XboxController::UartReceive_IDLE_DMA_Callback(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (huart == this->huart)
    {
        char RxData[XBOXCONTROLLER_RX_BUFFER_SIZE_MAX];
        for (int i = 0; i < Size; i++)
        {
            RxData[i] = RxBuffer[i];
        }
        uint8_t RxFlag = sscanf(RxData,
            "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
            &connected,
            &btnA.val, &btnB.val, &btnX.val, &btnY.val,
            &btnLB.val, &btnRB.val,
            &btnSelect.val, &btnStart.val, &btnXbox.val, &btnShare.val,
            &btnLS.val, &btnRS.val,
            &btnDirUp.val, &btnDirDown.val, &btnDirLeft.val, &btnDirRight.val,
            &joyLHori, &joyLVert, &joyRHori, &joyRVert,
            &trigLT, &trigRT
        );
        if (RxFlag <= 0)
            printf("Error: sscanf failed\n");

        if (joyLVert >= -2000 && joyLVert <= 2000) joyLVert = 0;
        if (joyLHori >= -2000 && joyLHori <= 2000) joyLHori = 0;
        if (joyRVert >= -2000 && joyRVert <= 2000) joyRVert = 0;
        if (joyRHori >= -2000 && joyRHori <= 2000) joyRHori = 0;

        HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*)this->RxBuffer, XBOXCONTROLLER_RX_BUFFER_SIZE_MAX);
    }
}

void XboxController::AddKey(KEY* key)
{
    key->state = KEY::Release;
    keys.push_back(key);
    keyCnt++;
}

void XboxController::KeysHandler()
{
    // 获取时间戳
    nowTime = HAL_GetTick();
    deltaTime = nowTime - lastTime;
    lastTime = nowTime;
    for (int i = 0; i < keyCnt; i++)
    {
        // 按下计时
        if (!keys[i]->val)
        {
            if (keys[i]->state == KEY::LongHold)
            {
                keys[i]->holdTime = keys[i]->pressTimer;
            }
            keys[i]->pressTimer = 0;
        }
        if (keys[i]->val)
        {
            keys[i]->pressTimer += deltaTime;
        }
        // 间隔计时
        if (keys[i]->state == KEY::MultiClick)
        {
            keys[i]->intervalTimer += deltaTime;
        }
        else
        {
            keys[i]->intervalTimer = 0;
        }

        // 事件生成
        switch (keys[i]->state)
        {
        case KEY::Release:
            keys[i]->clickCnt = 0;

            if (keys[i]->val)
            {
                keys[i]->state = KEY::PrePress;
            }
            break;
        case KEY::PrePress:

            if (!keys[i]->val)
            {
                keys[i]->state = KEY::Release;
            }
            else if (keys[i]->pressTimer >= KEY_ClickThreshold)
            {
                keys[i]->state = KEY::Prelong;
            }
            break;
        case KEY::Prelong:

            if (!keys[i]->val)
            {
                keys[i]->state = KEY::MultiClick;
                keys[i]->clickCnt++;
            }
            else if (keys[i]->pressTimer >= KEY_HoldThreshold)
            {
                keys[i]->state = KEY::LongHold;
                keys[i]->triggerTimer = KEY_HoldTriggerFirstThreshold;
                KEY_LongHoldCallback(keys[i]);
            }
            break;
        case KEY::LongHold:

            if (keys[i]->triggerTimer > 0)
                keys[i]->triggerTimer -= deltaTime;
            else
            {
                keys[i]->triggerTimer = KEY_HoldTriggerThreshold;
                KEY_HoldTriggerCallback(keys[i]);
            }

            if (!keys[i]->val)
            {
                keys[i]->state = KEY::Release;
            }

            break;
        case KEY::MultiClick:

            if (keys[i]->intervalTimer >= KEY_IntervalThreshold)
            {
                if (keys[i]->clickCnt > 1)
                {
                    KEY_MultipleClickCallback(keys[i]);
                }
                else if (keys[i]->clickCnt == 1)
                {
                    KEY_KeyClickCallback(keys[i]);
                }
                keys[i]->state = KEY::Release;
            }
            else if (keys[i]->pressTimer >= KEY_ClickThreshold)
            {
                keys[i]->state = KEY::Prelong;
            }
            break;

        default:
            break;
        }
    }
}

/**
 * @brief 按键按下回调函数
 *
 */
__attribute__((weak)) void KEY_KeyClickCallback(KEY* key)
{
    UNUSED(key);
}
/**
 * @brief 按键多击回调函数
 *
 */
__attribute__((weak)) void KEY_MultipleClickCallback(KEY* key)
{
    UNUSED(key);
}
/**
 * @brief 按键长按回调函数
 *
 */
__attribute__((weak)) void KEY_LongHoldCallback(KEY* key)
{
    UNUSED(key);
}
/**
 * @brief 按键长按触发回调函数
 *
 */
__attribute__((weak)) void KEY_HoldTriggerCallback(KEY* key)
{
    UNUSED(key);
}
