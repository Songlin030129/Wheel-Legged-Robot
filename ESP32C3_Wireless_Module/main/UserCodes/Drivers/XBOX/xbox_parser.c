#include "xbox_parser.h"

static inline uint16_t u16le(const uint8_t* p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void hat_to_dirs(uint8_t hat, bool* up, bool* right, bool* down, bool* left)
{
    // 与原 C++ 逻辑一致（含斜向）
    *up = (hat == 1 || hat == 2 || hat == 8);
    *right = (hat >= 2 && hat <= 4);
    *down = (hat >= 4 && hat <= 6);
    *left = (hat >= 6 && hat <= 8);
}

int xbox_parse_input_report(const uint8_t* d, size_t len, xbox_input_t* o)
{
    if (!d || !o || len != XBOX_INPUT_REPORT_LEN)
        return -1;

    o->joyLHori = u16le(&d[0]);
    o->joyLVert = u16le(&d[2]);
    o->joyRHori = u16le(&d[4]);
    o->joyRVert = u16le(&d[6]);
    o->trigLT = u16le(&d[8]);
    o->trigRT = u16le(&d[10]);

    o->hat = d[12];
    hat_to_dirs(o->hat, &o->dirUp, &o->dirRight, &o->dirDown, &o->dirLeft);

    uint8_t mainBtn = d[13];
    o->btnA = mainBtn & 0x01;
    o->btnB = mainBtn & 0x02;
    o->btnX = mainBtn & 0x08;
    o->btnY = mainBtn & 0x10;
    o->btnLB = mainBtn & 0x40;
    o->btnRB = mainBtn & 0x80;

    uint8_t midBtn = d[14];
    o->btnSelect = midBtn & 0x04;
    o->btnStart = midBtn & 0x08;
    o->btnXbox = midBtn & 0x10;
    o->btnLS = midBtn & 0x20;
    o->btnRS = midBtn & 0x40;

    uint8_t share = d[15];
    o->btnShare = share & 0x01;

    return 0;
}
