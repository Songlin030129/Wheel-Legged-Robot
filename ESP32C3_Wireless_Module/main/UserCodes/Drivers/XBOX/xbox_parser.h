#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


#define XBOX_INPUT_REPORT_LEN 16

typedef struct {
    // 轴/扳机（原始 0..65535，小端）
    uint16_t joyLHori;
    uint16_t joyLVert;
    uint16_t joyRHori;
    uint16_t joyRVert;
    uint16_t trigLT;
    uint16_t trigRT;

    // 十字键
    uint8_t hat;  // 0..8
    bool dirUp, dirRight, dirDown, dirLeft;

    // 面键 / 肩键
    bool btnA, btnB, btnX, btnY, btnLB, btnRB;

    // 中区键
    bool btnSelect, btnStart, btnXbox, btnLS, btnRS;

    // Share
    bool btnShare;
} xbox_input_t;

int xbox_parse_input_report(const uint8_t* data, size_t len, xbox_input_t* out);
