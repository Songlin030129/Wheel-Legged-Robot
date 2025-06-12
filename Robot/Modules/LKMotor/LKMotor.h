#ifndef __LKMOTOR_H__
#define __LKMOTOR_H__


#include "common_inc.h"
#include "fdcan.h"
#include "bsp_can.h"


class LKMotor
{
public:
    void Init(FDCAN_HandleTypeDef* _hcan, uint8_t _id);
    void Enable();
    void Disable();
    void Stop();
    void Control(float _torque);
    void Recv_Callback(FDCAN_HandleTypeDef* _hcan, FDCAN_RxHeaderTypeDef RxHeader, uint8_t* RecvBuf);

    int Temperature;
    float Torque;
    float Velocity;

private:
    FDCAN_HandleTypeDef* hcan;
    uint8_t id;

};

extern LKMotor Wheel_Motor_Left, Wheel_Motor_Right;



#endif