#ifndef __DM4310_DRV_H__
#define __DM4310_DRV_H__
#include "common_inc.h"
#include "fdcan.h"
#include "bsp_can.h"
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f


#define P_MIN2 -12.0f
#define P_MAX2 12.0f
#define V_MIN2 -45.0f
#define V_MAX2 45.0f
#define KP_MIN2 0.0f
#define KP_MAX2 500.0f
#define KD_MIN2 0.0f
#define KD_MAX2 5.0f
#define T_MIN2 -18.0f
#define T_MAX2 18.0f

typedef enum
{
    CONTROL_MODE_MIT = 0,
    CONTROL_MODE_POS_VEL = 0x100,
    CONTROL_MODE_VEL = 0x200,
} Control_Mode;

typedef enum
{
    MOTOR_TYPE_JOINT = 0,
    MOTOR_TYPE_WHEEL = 1,
} Motor_Type;
class DMMotor
{
public:

    void Init(FDCAN_HandleTypeDef* _hcan, Motor_Type _type, uint8_t _Slave_ID, uint8_t _Master_ID, Control_Mode _Mode);
    void Recv_Callback(FDCAN_HandleTypeDef* _hcan, FDCAN_RxHeaderTypeDef RxHeader, uint8_t* RecvBuf);

    void Enable();
    void Disable();
    void SaveZeroPoint();
    void ClearErr();

    void Control(float _pos, float _vel, float _KP, float _KD, float _torq);
    void Control(float _vel, float _pos);
    void Control(float _vel);

    float Position, Velocity, Torque;

private:
    uint8_t Motor_ID, Master_ID;
    Control_Mode Mode;
    Motor_Type Type;
    FDCAN_HandleTypeDef* hcan;
    float uint_to_float(int x_int, float x_min, float x_max, int bits);
    int float_to_uint(float x, float x_min, float x_max, int bits);

};

extern DMMotor Joint_Motor_Left_1, Joint_Motor_Left_2, Joint_Motor_Right_1, Joint_Motor_Right_2;

#endif



