#pragma once
#include <stddef.h>
#include <stdint.h>


#define XBOX_OUT_REPORT_LEN 8

typedef struct {
    // 选择位（bit）：0:center 1:shake 2:right 3:left 其余保留
    uint8_t select_bits;

    // 功率 0..100（>100 将在打包时截断为 100）
    uint8_t power_left;
    uint8_t power_right;
    uint8_t power_shake;
    uint8_t power_center;

    // 时序（单位 0.01s）
    uint8_t time_active;  // 最多 2.55s
    uint8_t time_silent;
    uint8_t count_repeat;
} xbox_out_t;

void xbox_out_init_full_1s(xbox_out_t* r);
void xbox_out_init_all_off(xbox_out_t* r);
void xbox_out_to_bytes(const xbox_out_t* r, uint8_t out[XBOX_OUT_REPORT_LEN]);
