#ifndef __XBOXCONTROLLER_H__
#define __XBOXCONTROLLER_H__

#include <stdio.h>
#include "common_inc.h"

#define XBOXCONTROLLER_RX_BUFFER_SIZE_MAX 1024

#define KEY_ClickThreshold 20
#define KEY_HoldThreshold 600
#define KEY_IntervalThreshold 200
#define KEY_HoldTriggerFirstThreshold 600
#define KEY_HoldTriggerThreshold 100

typedef enum { KEY_Release = 0, KEY_PrePress, KEY_Prelong, KEY_LongHold, KEY_MultiClick } KEY_State;

typedef struct {
    int val;
    KEY_State state;
    uint32_t pressTimer;
    uint32_t intervalTimer;
    int16_t triggerTimer;
    uint32_t holdTime;
    uint8_t clickCnt;
} KEY;

typedef struct {
    /* runtime data */
    int connected;
    int joyLHori, joyLVert, joyRHori, joyRVert;
    int trigLT, trigRT;

    /* keys (原类成员) */
    KEY btnA, btnB, btnX, btnY;
    KEY btnShare, btnStart, btnSelect, btnXbox;
    KEY btnLB, btnRB;
    KEY btnLS, btnRS;
    KEY btnDirUp, btnDirLeft, btnDirRight, btnDirDown;

    /* internal */
    UART_HandleTypeDef* huart;
    uint8_t RxBuffer[XBOXCONTROLLER_RX_BUFFER_SIZE_MAX];
    KEY* keys[32];
    uint8_t keyCnt;
    uint32_t nowTime;
    uint32_t lastTime;
    uint32_t deltaTime;
} XboxController;

extern XboxController xbox;

/* API */
void xbox_init(XboxController* _xbox, UART_HandleTypeDef* _huart);
void xbox_uart_receive_idle_dma_callback(XboxController* _xbox, UART_HandleTypeDef* _huart, uint16_t _size);
void xbox_uart_err_callback(XboxController* _xbox, UART_HandleTypeDef* _huart);

void xbox_add_key(XboxController* _xbox, KEY* _key);
void xbox_keys_handler(XboxController* _xbox);

/* 回调声明（可被用户覆盖） */
void KEY_KeyClickCallback(KEY* _key);
void KEY_MultipleClickCallback(KEY* _key);
void KEY_LongHoldCallback(KEY* _key);
void KEY_HoldTriggerCallback(KEY* _key);

#endif