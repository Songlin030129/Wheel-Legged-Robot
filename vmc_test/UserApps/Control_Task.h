#ifndef __CONTROL_TASK_H__
#define __CONTROL_TASK_H__

#include "common_inc.h"
#include "DMMotor.h"
#include "VMC.h"
#include "PID.h"
#include "INS_Task.h"
#include "bsp_dwt.h"    
#define WIGHT_GAIN 20.0f
#define WHEEL_RADIUS 0.0603f
class Control
{
public:
    VMC vmc_left, vmc_right;
    PIDController PID_L0_L, PID_L0_R;
    void Task();
    float LQR_Get_K(float* coe, float len);
    void mySaturate(float* in, float min, float max);
    uint8_t myJudge(float in, float min, float max);

    float Tar_L0_L;
    float Tar_L0_R;

    float dt;
    uint32_t control_dwt_count;
    float Control_Enable;
};

extern Control control;

#endif // !__CONTROL_TASK_H__
