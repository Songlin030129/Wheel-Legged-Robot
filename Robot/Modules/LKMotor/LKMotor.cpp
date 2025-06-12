#include "LKMotor.h"
LKMotor Wheel_Motor_Left, Wheel_Motor_Right;
void LKMotor::Init(FDCAN_HandleTypeDef* _hcan, uint8_t _id)
{
    hcan = _hcan;
    id = _id;
}

void LKMotor::Enable()
{
    uint16_t _ID = id + 0x140;
    uint8_t Data[8];
    Data[0] = 0x88;
    Data[1] = 0x00;
    Data[2] = 0x00;
    Data[3] = 0x00;
    Data[4] = 0x00;
    Data[5] = 0x00;
    Data[6] = 0x00;
    Data[7] = 0x00;
    canx_send_data(hcan, _ID, Data, 8);

}

void LKMotor::Disable()
{
    uint16_t _ID = id + 0x140;
    uint8_t Data[8];
    Data[0] = 0x80;
    Data[1] = 0x00;
    Data[2] = 0x00;
    Data[3] = 0x00;
    Data[4] = 0x00;
    Data[5] = 0x00;
    Data[6] = 0x00;
    Data[7] = 0x00;
    canx_send_data(hcan, _ID, Data, 8);

}

void LKMotor::Stop()
{
    uint16_t _ID = id + 0x140;
    uint8_t Data[8];
    Data[0] = 0x81;
    Data[1] = 0x00;
    Data[2] = 0x00;
    Data[3] = 0x00;
    Data[4] = 0x00;
    Data[5] = 0x00;
    Data[6] = 0x00;
    Data[7] = 0x00;
    canx_send_data(hcan, _ID, Data, 8);

}

void LKMotor::Control(float _torque)
{
    float _current = _torque / 0.322f;
    int16_t int16_value = (int16_t)(((_current + 16.5f) * 4096) / 33 - 2048);
    uint16_t _ID = id + 0x140;
    uint8_t Data[8];
    Data[0] = 0xA1;
    Data[1] = 0x00;
    Data[2] = 0x00;
    Data[3] = 0x00;
    Data[4] = *(uint8_t*)(&int16_value);
    Data[5] = *((uint8_t*)(&int16_value) + 1);
    Data[6] = 0x00;
    Data[7] = 0x00;
    canx_send_data(hcan, _ID, Data, 8);

}

void LKMotor::Recv_Callback(FDCAN_HandleTypeDef* _hcan, FDCAN_RxHeaderTypeDef RxHeader, uint8_t* RecvBuf)
{
    if (_hcan == hcan) {
        if (RxHeader.Identifier == 0x140 + id) {
            if (RecvBuf[0] == 0xA1) {
                Temperature = RecvBuf[1];
                int16_t temp_iq = (RecvBuf[3] << 8) | RecvBuf[2];
                Torque = (((float)(temp_iq + 2048) * 33) / 4096 - 16.5f) * 0.322f;
                int16_t temp_Vel = (RecvBuf[5] << 8) | RecvBuf[4];
                Velocity = (float)temp_Vel * 3.14f / 180.0f;
            }
        }
    }
}
