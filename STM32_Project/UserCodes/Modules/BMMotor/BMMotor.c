#include "BMMotor.h"
#include "bsp_can.h"
#include "common_inc.h"
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

BMMotor_Group bmmotor_group_left;
BMMotor_Group bmmotor_group_right;
BMMotor wheel_motor_left;
BMMotor wheel_motor_right;

BMMotorStatus bmmotor_group_init(BMMotor_Group* _motor_group, FDCAN_HandleTypeDef* _hcan)
{
    if (!_motor_group || !_hcan)
        return BMMOTOR_ERR_PARAM;
    _motor_group->hcan = _hcan;
    _motor_group->motor_cnt = 0;
    memset(_motor_group->motors, 0, sizeof(_motor_group->motors));
    return BMMOTOR_OK;
}
BMMotorStatus bmmotor_group_add_motor(BMMotor_Group* _motor_group, BMMotor* _motor, uint8_t _id)
{
    if (!_motor || !_motor_group)
        return BMMOTOR_ERR_PARAM;
    if (_motor_group->motor_cnt >= 8)
        return BMMOTOR_ERR_FULL;
    if (_id < 1 || _id > 8)
        return BMMOTOR_ERR_ID_RANGE;
    if (_motor_group->motors[_id - 1])
        return BMMOTOR_ERR_ALREADY_USED;
    _motor->hcan = _motor_group->hcan;
    _motor->id = _id;
    _motor->mode = BMMOTOR_MODE_ENABLE;
    _motor->err_code = 0x00;
    _motor->position = 0.0f;
    _motor->velocity = 0.0f;
    _motor->torque = 0.0f;
    _motor_group->motors[_id - 1] = _motor;
    _motor_group->motor_cnt++;
    return BMMOTOR_OK;
}

void bmmotor_group_recv_callback(BMMotor_Group* _motor_group, FDCAN_HandleTypeDef* _hcan, FDCAN_RxHeaderTypeDef rxheader, uint8_t* recvbuf)
{
    if (!_motor_group || !recvbuf)
        return;
    if (_hcan == _motor_group->hcan) {
        if (rxheader.Identifier > 0x96 && rxheader.Identifier <= 0x9E) {
            uint8_t recv_id = rxheader.Identifier - 0x96;
            if (!_motor_group->motors[recv_id - 1])
                return;
            int16_t temp = (recvbuf[0] << 8) | recvbuf[1];
            _motor_group->motors[recv_id - 1]->velocity = (int16_t)temp * 0.1f * _PI / 30.0f;
            temp = (recvbuf[2] << 8) | recvbuf[3];
            _motor_group->motors[recv_id - 1]->torque = (int16_t)temp * 55.0f / 32767 * 0.8f;
            temp = (recvbuf[4] << 8) | recvbuf[5];
            _motor_group->motors[recv_id - 1]->position = (int16_t)temp * 360.0f / 32767.0f;
            _motor_group->motors[recv_id - 1]->err_code = recvbuf[6];
            _motor_group->motors[recv_id - 1]->mode = recvbuf[7];
        } else if (rxheader.Identifier > 0x200 && rxheader.Identifier <= 0x208) {
            uint8_t recv_id = rxheader.Identifier - 0x200;
            if (!_motor_group->motors[recv_id - 1])
                return;
            _motor_group->motors[recv_id - 1]->mode = recvbuf[0];
        }
    }
}

BMMotorStatus bmmotor_group_send_control(BMMotor_Group* _motor_group)
{
    if (!_motor_group)
        return BMMOTOR_ERR_PARAM;
    uint8_t data[16] = {0};
    for (int i = 0; i < 8; i++) {
        BMMotor* motor_temp = _motor_group->motors[i];
        if (!motor_temp)
            continue;
        int16_t int16_value = 0;
        float float_value = 0.0f;
        switch (motor_temp->mode_set) {
            case BMMOTOR_MODE_VOLTAGE:
                int16_value = motor_temp->voltage_set;
                int16_value = _constrain(int16_value, -16384, 16384);
                data[i * 2] = (int16_value >> 8) & 0xFF;
                data[i * 2 + 1] = int16_value & 0xFF;
                break;
            case BMMOTOR_MODE_VELOCITY:
                float_value = motor_temp->velocity_set * 30.0f / _PI * 10.0f;
                float_value = _constrain(float_value, -7500, 7500);
                int16_value = (int16_t)float_value;
                data[i * 2] = (int16_value >> 8) & 0xFF;
                data[i * 2 + 1] = int16_value & 0xFF;
                break;
            case BMMOTOR_MODE_CURRENT:
                float_value = motor_temp->torque_set / 0.8f / 55 * 32767;
                float_value = _constrain(float_value, -16383, 16383);
                int16_value = (int16_t)float_value;
                data[i * 2] = (int16_value >> 8) & 0xFF;
                data[i * 2 + 1] = int16_value & 0xFF;
                break;
            default:
                break;
        }
    }
    uint8_t data1[8] = {0};
    memcpy(data1, data, 8);
    bsp_canx_send_data(_motor_group->hcan, 0x32, data1, 8);
    if (_motor_group->motors[4] || _motor_group->motors[5] || _motor_group->motors[6] || _motor_group->motors[7]) {
        uint8_t data2[8] = {0};
        memcpy(data2, data + 8, 8);
        bsp_canx_send_data(_motor_group->hcan, 0x33, data2, 8);
    }
    return BMMOTOR_OK;
}

static BMMotorStatus bmmotor_group_send_mode(BMMotor_Group* _motor_group)
{
    if (!_motor_group)
        return BMMOTOR_ERR_PARAM;
    uint8_t data[8] = {0};
    for (int i = 0; i < 8; i++) {
        if (!_motor_group->motors[i])
            continue;
        data[i] = _motor_group->motors[i]->mode_set;
    }
    bsp_canx_send_data(_motor_group->hcan, 0x105, data, 8);
    return BMMOTOR_OK;
}

BMMotorStatus bmmotor_set_torque(BMMotor* _motor, float _torque)
{
    if (!_motor)
        return BMMOTOR_ERR_PARAM;
    _motor->torque_set = _torque;
    return BMMOTOR_OK;
}
BMMotorStatus bmmotor_set_velocity(BMMotor* _motor, float _velocity)
{
    if (!_motor)
        return BMMOTOR_ERR_PARAM;
    _motor->velocity_set = _velocity;
    return BMMOTOR_OK;
}
BMMotorStatus bmmotor_set_voltage(BMMotor* _motor, int16_t _set)
{
    if (!_motor)
        return BMMOTOR_ERR_PARAM;
    _motor->voltage_set = _set;
    return BMMOTOR_OK;
}

BMMotorStatus bmmotor_set_mode(BMMotor_Group* _motor_group, BMMotor* _motor, BMMotor_Mode _mode)
{
    if (!_motor || !_motor_group)
        return BMMOTOR_ERR_PARAM;
    vTaskDelay(1);
    _motor->mode_set = _mode;
    bmmotor_group_send_mode(_motor_group);
    vTaskDelay(1);
    return BMMOTOR_OK;
}
BMMotorStatus bmmotor_set_id(FDCAN_HandleTypeDef* _hcan, uint8_t _id)
{
    if (!_hcan)
        return BMMOTOR_ERR_PARAM;
    if (_id < 1 || _id > 8)
        return BMMOTOR_ERR_ID_RANGE;
    uint8_t data[8] = {0};
    data[0] = _id;
    vTaskDelay(1);
    bsp_canx_send_data(_hcan, 0x108, data, 8);
    vTaskDelay(1);
    return BMMOTOR_OK;
}
BMMotorStatus bmmotor_calibrate(FDCAN_HandleTypeDef* _hcan)
{
    if (!_hcan)
        return BMMOTOR_ERR_PARAM;
    uint8_t data[8] = {0};
    vTaskDelay(1);
    bsp_canx_send_data(_hcan, 0x104, data, 8);
    vTaskDelay(1);
    return BMMOTOR_OK;
}
