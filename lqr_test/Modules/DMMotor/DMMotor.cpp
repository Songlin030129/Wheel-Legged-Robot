#include "DMMotor.h"
DMMotor Joint_Motor_Left_1, Joint_Motor_Left_2, Joint_Motor_Right_1, Joint_Motor_Right_2;
DMMotor Wheel_Motor_Left, Wheel_Motor_Right;

void DMMotor::Init(FDCAN_HandleTypeDef* _hcan, Motor_Type _type, uint8_t _Slave_ID, uint8_t _Master_ID, Control_Mode _Mode)
{
    hcan = _hcan;
    Motor_ID = _Slave_ID;
    Master_ID = _Master_ID;
    Mode = _Mode;
    Type = _type;
}

void DMMotor::Recv_Callback(FDCAN_HandleTypeDef* _hcan, FDCAN_RxHeaderTypeDef RxHeader, uint8_t* RecvBuf)
{
    if (_hcan == hcan)
    {
        if (RxHeader.Identifier == Master_ID)
        {
            int p_int = (RecvBuf[1] << 8) | RecvBuf[2];
            int v_int = (RecvBuf[3] << 4) | (RecvBuf[4] >> 4);
            int t_int = ((RecvBuf[4] & 0xF) << 8) | RecvBuf[5];
            if (Type == MOTOR_TYPE_JOINT) {
                Position = uint_to_float(p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
                Velocity = uint_to_float(v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
                Torque = uint_to_float(t_int, T_MIN, T_MAX, 12); // (-18.0,18.0) 
            }
            else if (Type == MOTOR_TYPE_WHEEL) {
                Position = uint_to_float(p_int, P_MIN2, P_MAX2, 16); // (-12.5,12.5)
                Velocity = uint_to_float(v_int, V_MIN2, V_MAX2, 12); // (-45.0,45.0)
                Torque = uint_to_float(t_int, T_MIN2, T_MAX2, 12); // (-18.0,18.0) 
            }
        }
    }
}

void DMMotor::Enable()
{
    uint16_t id = Motor_ID + Mode;
    uint8_t Data[8];
    Data[0] = 0xFF;
    Data[1] = 0xFF;
    Data[2] = 0xFF;
    Data[3] = 0xFF;
    Data[4] = 0xFF;
    Data[5] = 0xFF;
    Data[6] = 0xFF;
    Data[7] = 0xFC;
    canx_send_data(hcan, id, Data, 8);

}

void DMMotor::Disable()
{
    uint16_t id = Motor_ID + Mode;
    uint8_t Data[8];
    Data[0] = 0xFF;
    Data[1] = 0xFF;
    Data[2] = 0xFF;
    Data[3] = 0xFF;
    Data[4] = 0xFF;
    Data[5] = 0xFF;
    Data[6] = 0xFF;
    Data[7] = 0xFD;
    canx_send_data(hcan, id, Data, 8);

}

void DMMotor::SaveZeroPoint()
{
    uint16_t id = Motor_ID + Mode;
    uint8_t Data[8];
    Data[0] = 0xFF;
    Data[1] = 0xFF;
    Data[2] = 0xFF;
    Data[3] = 0xFF;
    Data[4] = 0xFF;
    Data[5] = 0xFF;
    Data[6] = 0xFF;
    Data[7] = 0xFE;
    canx_send_data(hcan, id, Data, 8);

}

void DMMotor::ClearErr()
{
    uint16_t id = Motor_ID + Mode;
    uint8_t Data[8];
    Data[0] = 0xFF;
    Data[1] = 0xFF;
    Data[2] = 0xFF;
    Data[3] = 0xFF;
    Data[4] = 0xFF;
    Data[5] = 0xFF;
    Data[6] = 0xFF;
    Data[7] = 0xFB;
    canx_send_data(hcan, id, Data, 8);

}


void DMMotor::Control(float _pos, float _vel, float _KP, float _KD, float _torq)
{
    if (Mode == CONTROL_MODE_MIT)
    {
        uint8_t Data[8];
        uint16_t id = Motor_ID + Mode;
        uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
        if (Type == MOTOR_TYPE_JOINT) {
            pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
            vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
            kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
            kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
            tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);
        }
        else if (Type == MOTOR_TYPE_WHEEL) {
            pos_tmp = float_to_uint(_pos, P_MIN2, P_MAX2, 16);
            vel_tmp = float_to_uint(_vel, V_MIN2, V_MAX2, 12);
            kp_tmp = float_to_uint(_KP, KP_MIN2, KP_MAX2, 12);
            kd_tmp = float_to_uint(_KD, KD_MIN2, KD_MAX2, 12);
            tor_tmp = float_to_uint(_torq, T_MIN2, T_MAX2, 12);
        }
        Data[0] = (pos_tmp >> 8);
        Data[1] = pos_tmp;
        Data[2] = (vel_tmp >> 4);
        Data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
        Data[4] = kp_tmp;
        Data[5] = (kd_tmp >> 4);
        Data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
        Data[7] = tor_tmp;
        canx_send_data(hcan, id, Data, 8);

    }
}

void DMMotor::Control(float _vel, float _pos)
{
    if (Mode == CONTROL_MODE_POS_VEL)
    {
        uint16_t id = Motor_ID + Mode;
        uint8_t* pbuf, * vbuf;
        pbuf = (uint8_t*)&_pos;
        vbuf = (uint8_t*)&_vel;
        uint8_t Data[8];
        Data[0] = *pbuf;
        Data[1] = *(pbuf + 1);
        Data[2] = *(pbuf + 2);
        Data[3] = *(pbuf + 3);
        Data[4] = *vbuf;
        Data[5] = *(vbuf + 1);
        Data[6] = *(vbuf + 2);
        Data[7] = *(vbuf + 3);
        canx_send_data(hcan, id, Data, 8);
    }
}

void DMMotor::Control(float _vel)
{
    if (Mode == CONTROL_MODE_VEL)
    {
        uint16_t id = Motor_ID + Mode;
        uint8_t Data[4];
        uint8_t* vbuf;
        vbuf = (uint8_t*)&_vel;
        Data[0] = *vbuf;
        Data[1] = *(vbuf + 1);
        Data[2] = *(vbuf + 2);
        Data[3] = *(vbuf + 3);
        canx_send_data(hcan, id, Data, 8);
    }
}

float DMMotor::uint_to_float(int x_int, float x_min, float x_max, int bits) {
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
int DMMotor::float_to_uint(float x, float x_min, float x_max, int bits) {
    /// Converts a float to an unsigned int, given range and number of bits

    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
