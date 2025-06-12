#ifndef PID_H
#define PID_H

#include "common_inc.h"
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

/**
 *  PID controller class
 */
class PIDController
{
public:
    /**
     *
     * @param P - Proportional gain
     * @param I - Integral gain
     * @param D - Derivative gain
     * @param ramp - Maximum speed of change of the output value
     * @param limit - Maximum output value
     */
    void Init(float P, float I, float D, float ramp, float limit, float deadzone);
    float Cal(float error);
    void reset();

    float P;            //!< Proportional gain
    float I;            //!< Integral gain
    float D;            //!< Derivative gain
    float output_ramp;  //!< Maximum speed of change of the output value
    float output_limit; //!< Maximum output value
    float Ts;           //!< time stamp for the controller
    float output_value; //!< Last calculated output value
    bool Enable;        //!< Enable the PID controller
    float DeadZone;     //!< DeadZone for the PID controller
    float Error;        //!< Last calculated error value
    uint32_t Last_Time;
private:
    float error_prev;    //!< last tracking error value
    float output_prev;   //!< last pid output value
    float integral_prev; //!< last integral component value
};

#endif // PID_H