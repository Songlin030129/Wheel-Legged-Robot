#include "xbox_hid_report.h"

static inline uint8_t clamp100(uint8_t v)
{
    return v > 100 ? 100 : v;
}

void xbox_out_init_full_1s(xbox_out_t* r)
{
    r->select_bits = 0;
    r->select_bits |= (1u << 0);  // center
    r->select_bits |= (1u << 1);  // shake
    r->select_bits |= (1u << 2);  // right
    r->select_bits |= (1u << 3);  // left

    r->power_left = 100;
    r->power_right = 100;
    r->power_shake = 100;
    r->power_center = 100;

    r->time_active = 100;  // 1.00s
    r->time_silent = 0;
    r->count_repeat = 0;
}

void xbox_out_init_all_off(xbox_out_t* r)
{
    r->select_bits = 0;
    r->power_left = 0;
    r->power_right = 0;
    r->power_shake = 0;
    r->power_center = 0;
    r->time_active = 0;
    r->time_silent = 0;
    r->count_repeat = 0;
}

void xbox_out_to_bytes(const xbox_out_t* r, uint8_t out[XBOX_OUT_REPORT_LEN])
{
    out[0] = r->select_bits & 0x0F;  // 保留高 4 位为 0
    out[1] = clamp100(r->power_left);
    out[2] = clamp100(r->power_right);
    out[3] = clamp100(r->power_shake);
    out[4] = clamp100(r->power_center);
    out[5] = r->time_active;
    out[6] = r->time_silent;
    out[7] = r->count_repeat;
}
