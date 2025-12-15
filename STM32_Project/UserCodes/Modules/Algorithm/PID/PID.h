#ifndef PID_H
#define PID_H

#include "common_inc.h"

typedef struct {
    float kp;            //!< Proportional gain
    float ki;            //!< Integral gain
    float kd;            //!< Derivative gain
    float output_ramp;   //!< Maximum speed of change of the output value
    float output_limit;  //!< Maximum output value
    float dt;            //!< time stamp for the controller
    float output_value;  //!< Last calculated output value
    uint8_t enable;      //!< Enable the PID controller
    float deadzone;      //!< DeadZone for the PID controller
    float error;         //!< Last calculated error value
    uint32_t last_tick;

    float error_prev;     //!< last tracking error value
    float output_prev;    //!< last pid output value
    float integral_prev;  //!< last integral component value
} PIDController;

void pid_init(PIDController* _pid, float _kp, float _ki, float _kd, float _ramp, float _limit, float _deadzone);
void pid_reset(PIDController* _pid);
float pid_cal(PIDController* _pid, float _error);
#endif  // PID_H