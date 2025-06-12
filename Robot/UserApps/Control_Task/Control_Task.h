#ifndef __CONTROL_TASK_H__
#define __CONTROL_TASK_H__

#include "common_inc.h"
#include "DMMotor.h"
#include "LKMotor.h"
#include "VMC.h"
#include "PID.h"
#include "INS_Task.h"
#include "bsp_dwt.h"    
#include "LPF.h"
#define WIGHT_GAIN 20.0f
#define WHEEL_RADIUS 0.065f
typedef struct
{
    float theta;
    float d_theta;
    float x;
    float d_x;
    float phi;
    float d_phi;
} state_t;

typedef struct
{
    float wheel_T;
    float joint_Tp;
} input_t;

class Control
{
public:
    void Task();
    float LQR_Get_K(float* coe, float len);

    float Tar_Yaw;
    float Tar_Roll;
    float Tar_x;
    float Tar_d_x;
    float Tar_L0;

    float LQR_K_L[12];
    float LQR_K_R[12];

    state_t sys_state_l, sys_state_r;
    input_t sys_input_l, sys_input_r;

    float Control_Enable;
    float dt;
    uint32_t control_dwt_count;
    VMC vmc_left, vmc_right;
    PIDController PID_L0_L, PID_L0_R;
    PIDController PID_Roll;
    PIDController PID_Tp;
    PIDController PID_Yaw;
    LowPassFilter LPF_d_x;
    LowPassFilter LPF_tar_yaw;
    LowPassFilter LPF_tar_L0;
    LowPassFilter LPF_wheelL_T;
    LowPassFilter LPF_wheelR_T;

};
void Control_Task();
void mySaturate(float* in, float min, float max);

extern Control control;

#endif // !__CONTROL_TASK_H__
