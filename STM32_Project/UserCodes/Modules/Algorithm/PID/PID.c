#include "PID.h"

void pid_init(PIDController* _pid, float _kp, float _ki, float _kd, float _ramp, float _limit, float _deadzone)
{
    _pid->kp = _kp;
    _pid->ki = _ki;
    _pid->kd = _kd;
    _pid->output_limit = _limit;
    _pid->output_ramp = _ramp;
    _pid->deadzone = _deadzone;
    _pid->error_prev = 0;
    _pid->output_prev = 0;
    _pid->integral_prev = 0;
    _pid->output_value = 0;
    _pid->enable = 1;
}
void pid_reset(PIDController* _pid)
{
    _pid->integral_prev = 0.0f;
    _pid->output_prev = 0.0f;
    _pid->error_prev = 0.0f;
}
float pid_cal(PIDController* _pid, float _error)
{
    // 获取时间间隔Ts
    uint32_t current_tick = HAL_GetTick();
    if (_pid->last_tick == 0) {
        _pid->last_tick = current_tick;
        return 0;
    }
    _pid->dt = (current_tick - _pid->last_tick) / 1000.0f;
    _pid->last_tick = current_tick;

    if (_pid->enable) {
        // if deadzone defined
        // if the error is within the deadzone, set it to zero

        if (_pid->deadzone > 0) {
            if (_error < _pid->deadzone && _error > -_pid->deadzone) {
                _error = 0;
            }
        }
        _pid->error = _error;
        // u(s) = (P + I/s + Ds)e(s)
        // Discrete implementations
        // proportional part
        // u_p  = P *e(k)
        float proportional = _pid->kp * _pid->error;
        // Tustin transform of the integral part
        // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
        float integral = _pid->integral_prev + _pid->ki * _pid->dt * 0.5f * (_pid->error + _pid->error_prev);
        // antiwindup - limit the output
        if (_pid->output_limit > 0)
            integral = _constrain(integral, -_pid->output_limit, _pid->output_limit);
        // Discrete derivation
        // u_dk = D(ek - ek_1)/Ts
        float derivative = _pid->kd * (_pid->error - _pid->error_prev) / _pid->dt;

        // sum all the components
        float output = proportional + integral + derivative;
        // antiwindup - limit the output variable
        if (_pid->output_limit > 0)
            output = _constrain(output, -_pid->output_limit, _pid->output_limit);

        // if output ramp defined
        if (_pid->output_ramp > 0) {
            // limit the acceleration by ramping the output
            float output_rate = (output - _pid->output_prev) / _pid->dt;
            if (output_rate > _pid->output_ramp)
                output = _pid->output_prev + _pid->output_ramp * _pid->dt;
            else if (output_rate < -_pid->output_ramp)
                output = _pid->output_prev - _pid->output_ramp * _pid->dt;
        }
        // saving for the next pass
        _pid->integral_prev = integral;
        _pid->output_prev = output;
        _pid->error_prev = _pid->error;
        _pid->output_value = output;
        return output;
    } else {
        _pid->error_prev = 0;
        _pid->output_prev = 0;
        _pid->integral_prev = 0;
        _pid->output_value = 0;
        return 0;
    }
}