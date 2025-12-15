#include "LPF.h"

void lpf_init(LowPassFilter* _lpf, float _tf)
{
    _lpf->tf = _tf;
    _lpf->y_prev = 0;
}
float lpf_cal(LowPassFilter* _lpf, float _input)
{
    // 获取时间间隔Ts
    uint32_t current_tick = HAL_GetTick();
    if (_lpf->last_tick == 0) {
        _lpf->last_tick = current_tick;
        return 0;
    }
    _lpf->input = _input;
    _lpf->dt = (current_tick - _lpf->last_tick) / 1000.0f;
    _lpf->last_tick = current_tick;

    float alpha = _lpf->tf / (_lpf->tf + _lpf->dt);
    float y = alpha * _lpf->y_prev + (1.0f - alpha) * _lpf->input;
    _lpf->y_prev = y;
    _lpf->output = y;
    return y;
}