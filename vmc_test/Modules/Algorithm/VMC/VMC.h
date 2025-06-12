#ifndef __VMC_H__
#define __VMC_H__

#include "common_inc.h"
#include "arm_math.h"
#include "INS_Task.h"
#include "bsp_dwt.h"
#define pi 3.1415926f
#define GRAVITY 9.8
#define WHEEL_M 0.6
class VMC
{
public:
    /*左右两腿的公共参数，固定不变*/
    float l1;//单位为m
    float l2;//单位为m
    float l3;//单位为m
    float l4;//单位为m
    float l5;//AE长度 //单位为m

    float XB, YB;//B点的坐标
    float XD, YD;//D点的坐标

    float XC, YC;//C点的直角坐标
    float L0, phi0;//C点的极坐标
    float alpha;
    float d_alpha;

    float lBD;//BD两点的距离

    float d_phi0;//现在C点角度phi0的变换率
    float last_phi0;//上一次C点角度，用于计算角度phi0的变换率d_phi0

    float A0, B0, C0;//中间变量
    float phi2, phi3;
    float phi1, phi4;

    float j11, j12, j21, j22;//笛卡尔空间力到关节空间的力的雅可比矩阵系数
    float torque_set[2];

    float F0;
    float Tp;

    float theta;
    float d_theta;//theta的一阶导数
    float last_d_theta;
    float dd_theta;//theta的二阶导数

    float phi;
    float d_phi;

    float d_L0;//L0的一阶导数
    float dd_L0;//L0的二阶导数
    float last_L0;
    float last_d_L0;

    float F0_reverse;
    float Tp_reverse;
    float FN;//支持力

    uint8_t first_flag;
    float dt;
    uint32_t vmc_dwt_count;
    void init();

    void calc_kf_right();
    void calc_kf_left();

    void calc_vmc();

    void calc_fn(float _feedback_t1, float _feedback_t2);
    INS_t* imu;

};



#endif