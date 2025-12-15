#include "XboxController.h"
#include <string.h>

XboxController xbox;

/* helper 初始化所有按键并添加到列表 */
static void xbox_init_keys(XboxController* _xbox)
{
    memset(&_xbox->btnA, 0, sizeof(KEY));
    memset(&_xbox->btnB, 0, sizeof(KEY));
    memset(&_xbox->btnX, 0, sizeof(KEY));
    memset(&_xbox->btnY, 0, sizeof(KEY));
    memset(&_xbox->btnShare, 0, sizeof(KEY));
    memset(&_xbox->btnStart, 0, sizeof(KEY));
    memset(&_xbox->btnSelect, 0, sizeof(KEY));
    memset(&_xbox->btnXbox, 0, sizeof(KEY));
    memset(&_xbox->btnLB, 0, sizeof(KEY));
    memset(&_xbox->btnRB, 0, sizeof(KEY));
    memset(&_xbox->btnLS, 0, sizeof(KEY));
    memset(&_xbox->btnRS, 0, sizeof(KEY));
    memset(&_xbox->btnDirUp, 0, sizeof(KEY));
    memset(&_xbox->btnDirLeft, 0, sizeof(KEY));
    memset(&_xbox->btnDirRight, 0, sizeof(KEY));
    memset(&_xbox->btnDirDown, 0, sizeof(KEY));

    _xbox->keyCnt = 0;
    xbox_add_key(_xbox, &_xbox->btnA);
    xbox_add_key(_xbox, &_xbox->btnB);
    xbox_add_key(_xbox, &_xbox->btnX);
    xbox_add_key(_xbox, &_xbox->btnY);
    xbox_add_key(_xbox, &_xbox->btnShare);
    xbox_add_key(_xbox, &_xbox->btnStart);
    xbox_add_key(_xbox, &_xbox->btnSelect);
    xbox_add_key(_xbox, &_xbox->btnXbox);
    xbox_add_key(_xbox, &_xbox->btnLB);
    xbox_add_key(_xbox, &_xbox->btnRB);
    xbox_add_key(_xbox, &_xbox->btnLS);
    xbox_add_key(_xbox, &_xbox->btnRS);
    xbox_add_key(_xbox, &_xbox->btnDirUp);
    xbox_add_key(_xbox, &_xbox->btnDirLeft);
    xbox_add_key(_xbox, &_xbox->btnDirRight);
    xbox_add_key(_xbox, &_xbox->btnDirDown);
}

void xbox_init(XboxController* _xbox, UART_HandleTypeDef* _huart)
{
    if (!_xbox)
        return;
    memset(_xbox, 0, sizeof(XboxController));
    _xbox->huart = _huart;
    xbox_init_keys(_xbox);
    HAL_UARTEx_ReceiveToIdle_DMA(_huart, (uint8_t*)_xbox->RxBuffer, XBOXCONTROLLER_RX_BUFFER_SIZE_MAX);
}

/* DMA IDLE 回调，Size 为接收字节数 */
void xbox_uart_receive_idle_dma_callback(XboxController* _xbox, UART_HandleTypeDef* _huart, uint16_t _size)
{
    if (!_xbox || _huart != _xbox->huart)
        return;

    __HAL_UART_CLEAR_OREFLAG(_huart);
    __HAL_UART_CLEAR_FEFLAG(_huart);
    __HAL_UART_CLEAR_PEFLAG(_huart);
    __HAL_UART_CLEAR_NEFLAG(_huart);

    char RxData[XBOXCONTROLLER_RX_BUFFER_SIZE_MAX + 1];
    if (_size > XBOXCONTROLLER_RX_BUFFER_SIZE_MAX)
        _size = XBOXCONTROLLER_RX_BUFFER_SIZE_MAX;
    for (int i = 0; i < _size; i++)
        RxData[i] = (char)_xbox->RxBuffer[i];
    RxData[_size] = '\0';

    int parsed =
        sscanf(RxData, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", &_xbox->connected, &_xbox->btnA.val, &_xbox->btnB.val, &_xbox->btnX.val, &_xbox->btnY.val,
               &_xbox->btnLB.val, &_xbox->btnRB.val, &_xbox->btnSelect.val, &_xbox->btnStart.val, &_xbox->btnXbox.val, &_xbox->btnShare.val, &_xbox->btnLS.val, &_xbox->btnRS.val, &_xbox->btnDirUp.val,
               &_xbox->btnDirDown.val, &_xbox->btnDirLeft.val, &_xbox->btnDirRight.val, &_xbox->joyLHori, &_xbox->joyLVert, &_xbox->joyRHori, &_xbox->joyRVert, &_xbox->trigLT, &_xbox->trigRT);

    if (_xbox->joyLVert >= -2000 && _xbox->joyLVert <= 2000)
        _xbox->joyLVert = 0;
    if (_xbox->joyLHori >= -2000 && _xbox->joyLHori <= 2000)
        _xbox->joyLHori = 0;
    if (_xbox->joyRVert >= -2000 && _xbox->joyRVert <= 2000)
        _xbox->joyRVert = 0;
    if (_xbox->joyRHori >= -2000 && _xbox->joyRHori <= 2000)
        _xbox->joyRHori = 0;

    HAL_UARTEx_ReceiveToIdle_DMA(_huart, (uint8_t*)_xbox->RxBuffer, XBOXCONTROLLER_RX_BUFFER_SIZE_MAX);
}

void xbox_uart_err_callback(XboxController* _xbox, UART_HandleTypeDef* _huart)
{
    // 停止当前的DMA传输
    HAL_UART_DMAStop(_huart);

    // 清除所有错误标志位
    __HAL_UART_CLEAR_OREFLAG(_huart);
    __HAL_UART_CLEAR_FEFLAG(_huart);
    __HAL_UART_CLEAR_PEFLAG(_huart);
    __HAL_UART_CLEAR_NEFLAG(_huart);
    __HAL_UART_CLEAR_IDLEFLAG(_huart);

    // 清空接收FIFO
    while (__HAL_UART_GET_FLAG(_huart, UART_FLAG_RXNE)) {
        volatile uint8_t temp = _huart->Instance->RDR;
        (void)temp;
    }
    __HAL_UART_FLUSH_DRREGISTER(_huart);
    // 清空缓冲区
    memset(_xbox->RxBuffer, 0, XBOXCONTROLLER_RX_BUFFER_SIZE_MAX);

    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(_huart, (uint8_t*)_xbox->RxBuffer, XBOXCONTROLLER_RX_BUFFER_SIZE_MAX);
}

void xbox_add_key(XboxController* _xbox, KEY* _key)
{
    if (!_xbox || !_key)
        return;
    _key->state = KEY_Release;
    if (_xbox->keyCnt < (uint8_t)(sizeof(_xbox->keys) / sizeof(_xbox->keys[0]))) {
        _xbox->keys[_xbox->keyCnt++] = _key;
    }
}

void xbox_keys_handler(XboxController* _xbox)
{
    if (!_xbox)
        return;
    _xbox->nowTime = HAL_GetTick();
    _xbox->deltaTime = _xbox->nowTime - _xbox->lastTime;
    _xbox->lastTime = _xbox->nowTime;

    for (int i = 0; i < _xbox->keyCnt; i++) {
        KEY* k = _xbox->keys[i];
        if (!k)
            continue;

        if (!k->val) {
            if (k->state == KEY_LongHold) {
                k->holdTime = k->pressTimer;
            }
            k->pressTimer = 0;
        }
        if (k->val) {
            k->pressTimer += _xbox->deltaTime;
        }

        if (k->state == KEY_MultiClick) {
            k->intervalTimer += _xbox->deltaTime;
        } else {
            k->intervalTimer = 0;
        }

        switch (k->state) {
            case KEY_Release:
                k->clickCnt = 0;
                if (k->val)
                    k->state = KEY_PrePress;
                break;
            case KEY_PrePress:
                if (!k->val) {
                    k->state = KEY_Release;
                } else if (k->pressTimer >= KEY_ClickThreshold) {
                    k->state = KEY_Prelong;
                }
                break;
            case KEY_Prelong:
                if (!k->val) {
                    k->state = KEY_MultiClick;
                    k->clickCnt++;
                } else if (k->pressTimer >= KEY_HoldThreshold) {
                    k->state = KEY_LongHold;
                    k->triggerTimer = KEY_HoldTriggerFirstThreshold;
                    KEY_LongHoldCallback(k);
                }
                break;
            case KEY_LongHold:
                if (k->triggerTimer > 0)
                    k->triggerTimer -= (int16_t)_xbox->deltaTime;
                else {
                    k->triggerTimer = KEY_HoldTriggerThreshold;
                    KEY_HoldTriggerCallback(k);
                }
                if (!k->val)
                    k->state = KEY_Release;
                break;
            case KEY_MultiClick:
                if (k->intervalTimer >= KEY_IntervalThreshold) {
                    if (k->clickCnt > 1)
                        KEY_MultipleClickCallback(k);
                    else if (k->clickCnt == 1)
                        KEY_KeyClickCallback(k);
                    k->state = KEY_Release;
                } else if (k->pressTimer >= KEY_ClickThreshold) {
                    k->state = KEY_Prelong;
                }
                break;
            default:
                break;
        }
    }
}

/* 弱定义回调，用户可在应用覆盖 */
__attribute__((weak)) void KEY_KeyClickCallback(KEY* _key)
{
    UNUSED(_key);
}
__attribute__((weak)) void KEY_MultipleClickCallback(KEY* _key)
{
    UNUSED(_key);
}
__attribute__((weak)) void KEY_LongHoldCallback(KEY* _key)
{
    UNUSED(_key);
}
__attribute__((weak)) void KEY_HoldTriggerCallback(KEY* _key)
{
    UNUSED(_key);
}