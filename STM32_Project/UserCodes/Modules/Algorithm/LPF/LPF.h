#ifndef __LPF_H__
#define __LPF_H__

#include "common_inc.h"

typedef struct {
    float tf;      //!< Low pass filter time constant
    float y_prev;  //!< filtered value in previous execution step
    float dt;
    float input;
    float output;
    uint32_t last_tick;
} LowPassFilter;
void lpf_init(LowPassFilter* _lpf, float _tf);
float lpf_cal(LowPassFilter* _lpf, float _input);
#endif  // LOWPASS_FILTER_H