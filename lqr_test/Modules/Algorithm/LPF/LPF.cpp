#include "LPF.h"


void LowPassFilter::Init(float _Tf)
{
    Tf = _Tf;
    y_prev = 0;
}

float LowPassFilter::operator()(float x)
{
    //获取时间间隔Ts
    uint32_t currentTime = HAL_GetTick();
    if (Last_Time == 0)
    {
        Last_Time = currentTime;
        return 0;
    }
    Ts = (currentTime - Last_Time) / 1000.0f;
    Last_Time = currentTime;

    float alpha = Tf / (Tf + Ts);
    float y = alpha * y_prev + (1.0f - alpha) * x;
    y_prev = y;
    return y;
}
