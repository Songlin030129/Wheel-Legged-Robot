#include "DMMotor.h"

DMMotor joint_motor_left_1;
DMMotor joint_motor_left_2;
DMMotor joint_motor_right_1;
DMMotor joint_motor_right_2;

static float dmmotor_uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static int dmmotor_float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

void dmmotor_init(DMMotor* _motor, FDCAN_HandleTypeDef* _hcan, uint8_t _slave_id,
                  uint8_t _master_id, DMMotor_Control_Mode _mode)
{
    if (!_motor)
        return;
    _motor->hcan = _hcan;
    _motor->motor_id = _slave_id;
    _motor->master_id = _master_id;
    _motor->mode = _mode;
    _motor->position = 0.0f;
    _motor->velocity = 0.0f;
    _motor->torque = 0.0f;
}

void dmmotor_recv_callback(DMMotor* _motor, FDCAN_HandleTypeDef* _hcan,
                           FDCAN_RxHeaderTypeDef rxheader, uint8_t* recvbuf)
{
    if (!_motor)
        return;
    if (_hcan == _motor->hcan) {
        if (rxheader.Identifier == _motor->master_id) {
            int p_int = (recvbuf[1] << 8) | recvbuf[2];
            int v_int = (recvbuf[3] << 4) | (recvbuf[4] >> 4);
            int t_int = ((recvbuf[4] & 0xF) << 8) | recvbuf[5];
            _motor->position = dmmotor_uint_to_float(p_int, P_MIN, P_MAX, 16);
            _motor->velocity = dmmotor_uint_to_float(v_int, V_MIN, V_MAX, 12);
            _motor->torque = dmmotor_uint_to_float(t_int, T_MIN, T_MAX, 12);
        }
    }
}

void dmmotor_enable(DMMotor* _motor)
{
    if (!_motor)
        return;
    uint16_t id = _motor->motor_id + _motor->mode;
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    bsp_canx_send_data(_motor->hcan, id, data, 8);
}

void dmmotor_disable(DMMotor* _motor)
{
    if (!_motor)
        return;
    uint16_t id = _motor->motor_id + _motor->mode;
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    bsp_canx_send_data(_motor->hcan, id, data, 8);
}

void dmmotor_save_zero_point(DMMotor* _motor)
{
    if (!_motor)
        return;
    uint16_t id = _motor->motor_id + _motor->mode;
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    bsp_canx_send_data(_motor->hcan, id, data, 8);
}

void dmmotor_clear_err(DMMotor* _motor)
{
    if (!_motor)
        return;
    uint16_t id = _motor->motor_id + _motor->mode;
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    bsp_canx_send_data(_motor->hcan, id, data, 8);
}

void dmmotor_control_mit(DMMotor* _motor, float _pos, float _vel, float _KP, float _KD, float _torq)
{
    if (!_motor)
        return;
    if (_motor->mode == CONTROL_MODE_MIT) {
        uint8_t data[8];
        uint16_t id = _motor->motor_id + _motor->mode;
        uint16_t pos_tmp = dmmotor_float_to_uint(_pos, P_MIN, P_MAX, 16);
        uint16_t vel_tmp = dmmotor_float_to_uint(_vel, V_MIN, V_MAX, 12);
        uint16_t kp_tmp = dmmotor_float_to_uint(_KP, KP_MIN, KP_MAX, 12);
        uint16_t kd_tmp = dmmotor_float_to_uint(_KD, KD_MIN, KD_MAX, 12);
        uint16_t tor_tmp = dmmotor_float_to_uint(_torq, T_MIN, T_MAX, 12);

        data[0] = (pos_tmp >> 8) & 0xFF;
        data[1] = pos_tmp & 0xFF;
        data[2] = (vel_tmp >> 4) & 0xFF;
        data[3] = ((vel_tmp & 0xF) << 4) | ((kp_tmp >> 8) & 0xF);
        data[4] = kp_tmp & 0xFF;
        data[5] = (kd_tmp >> 4) & 0xFF;
        data[6] = ((kd_tmp & 0xF) << 4) | ((tor_tmp >> 8) & 0xF);
        data[7] = tor_tmp & 0xFF;
        bsp_canx_send_data(_motor->hcan, id, data, 8);
    }
}

void dmmotor_control_posvel(DMMotor* _motor, float _vel, float _pos)
{
    if (!_motor)
        return;
    if (_motor->mode == CONTROL_MODE_POS_VEL) {
        uint16_t id = _motor->motor_id + _motor->mode;
        uint8_t data[8];
        uint8_t* pbuf = (uint8_t*)&_pos;
        uint8_t* vbuf = (uint8_t*)&_vel;
        data[0] = pbuf[0];
        data[1] = pbuf[1];
        data[2] = pbuf[2];
        data[3] = pbuf[3];
        data[4] = vbuf[0];
        data[5] = vbuf[1];
        data[6] = vbuf[2];
        data[7] = vbuf[3];
        bsp_canx_send_data(_motor->hcan, id, data, 8);
    }
}

void dmmotor_control_vel(DMMotor* _motor, float _vel)
{
    if (!_motor)
        return;
    if (_motor->mode == CONTROL_MODE_VEL) {
        uint16_t id = _motor->motor_id + _motor->mode;
        uint8_t data4[4];
        uint8_t* vbuf = (uint8_t*)&_vel;
        data4[0] = vbuf[0];
        data4[1] = vbuf[1];
        data4[2] = vbuf[2];
        data4[3] = vbuf[3];
        uint8_t data8[8] = {data4[0], data4[1], data4[2], data4[3], 0, 0, 0, 0};
        bsp_canx_send_data(_motor->hcan, id, data8, 8);
    }
}
