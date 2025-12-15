#include "LKMotor.h"

// LKMotor wheel_motor_left;
// LKMotor wheel_motor_right;

void lkmotor_init(LKMotor* _motor, FDCAN_HandleTypeDef* _hcan, uint8_t _id)
{
    if (!_motor)
        return;
    _motor->hcan = _hcan;
    _motor->id = _id;
    _motor->temperature = 0;
    _motor->torque = 0.0f;
    _motor->velocity = 0.0f;
}

void lkmotor_enable(LKMotor* _motor)
{
    if (!_motor)
        return;
    uint16_t _ID = _motor->id + 0x140;
    uint8_t data[8] = {0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    bsp_canx_send_data(_motor->hcan, _ID, data, 8);
}

void lkmotor_disable(LKMotor* _motor)
{
    if (!_motor)
        return;
    uint16_t _ID = _motor->id + 0x140;
    uint8_t data[8] = {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    bsp_canx_send_data(_motor->hcan, _ID, data, 8);
}

void lkmotor_stop(LKMotor* _motor)
{
    if (!_motor)
        return;
    uint16_t _ID = _motor->id + 0x140;
    uint8_t data[8] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    bsp_canx_send_data(_motor->hcan, _ID, data, 8);
}

void lkmotor_control(LKMotor* _motor, float _torque)
{
    if (!_motor)
        return;
    float _current = _torque / 0.322f;
    int16_t int16_value = (int16_t)(((_current + 16.5f) * 4096.0f) / 33.0f - 2048.0f);
    uint16_t _ID = _motor->id + 0x140;
    uint8_t data[8] = {0};
    data[0] = 0xA1;
    data[4] = (uint8_t)(int16_value & 0xFF);
    data[5] = (uint8_t)((int16_value >> 8) & 0xFF);
    bsp_canx_send_data(_motor->hcan, _ID, data, 8);
}

void lkmotor_recv_callback(LKMotor* _motor, FDCAN_HandleTypeDef* _hcan, FDCAN_RxHeaderTypeDef rxheader, uint8_t* recvbuf)
{
    if (!_motor)
        return;
    if (_hcan == _motor->hcan) {
        if (rxheader.Identifier == (uint32_t)(0x140 + _motor->id)) {
            if (recvbuf[0] == 0xA1) {
                _motor->temperature = (int)recvbuf[1];
                int16_t temp_iq = (int16_t)((recvbuf[3] << 8) | recvbuf[2]);
                _motor->torque = (((float)(temp_iq + 2048) * 33.0f) / 4096.0f - 16.5f) * 0.322f;
                int16_t temp_vel = (int16_t)((recvbuf[5] << 8) | recvbuf[4]);
                _motor->velocity = (float)temp_vel * 3.14f / 180.0f;
            }
        }
    }
}