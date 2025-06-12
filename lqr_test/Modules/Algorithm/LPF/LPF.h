#ifndef __LPF_H__
#define __LPF_H__

#include "common_inc.h"
/**
 *  Low pass filter class
 */
class LowPassFilter
{
public:
    /**
     * @param Tf - Low pass filter time constant
     */
    void Init(float _Tf);

    float operator()(float x);
    float Tf;     //!< Low pass filter time constant
    float y_prev; //!< filtered value in previous execution step
    float Ts;
    uint32_t Last_Time;
};

#endif // LOWPASS_FILTER_H