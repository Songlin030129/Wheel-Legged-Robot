#include "VMC.h"

void VMC::init()
{
    l1 = 0.12f;//单位为m
    l2 = 0.20f;//单位为m
    l3 = 0.20f;//单位为m
    l4 = 0.12f;//单位为m
    l5 = 0.1016f;//AE长度 //单位为m
}

void VMC::calc_kf()
{
    dt = GetDeltaT(&vmc_dwt_count);

    phi = INS.Pitch;
    d_phi = INS.Gyro[0];

    YD = l4 * arm_sin_f32(phi4);//D的y坐标
    YB = l1 * arm_sin_f32(phi1);//B的y坐标
    XD = l5 + l4 * arm_cos_f32(phi4);//D的x坐标
    XB = l1 * arm_cos_f32(phi1); //B的x坐标

    lBD = sqrt((XD - XB) * (XD - XB) + (YD - YB) * (YD - YB));

    A0 = 2 * l2 * (XD - XB);
    B0 = 2 * l2 * (YD - YB);
    C0 = l2 * l2 + lBD * lBD - l3 * l3;
    phi2 = 2 * atan2f((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)), A0 + C0);
    phi3 = atan2f(YB - YD + l2 * arm_sin_f32(phi2), XB - XD + l2 * arm_cos_f32(phi2));
    //C点直角坐标
    XC = l1 * arm_cos_f32(phi1) + l2 * arm_cos_f32(phi2);
    YC = l1 * arm_sin_f32(phi1) + l2 * arm_sin_f32(phi2);
    //C点极坐标
    L0 = sqrt((XC - l5 / 2.0f) * (XC - l5 / 2.0f) + YC * YC);

    phi0 = atan2f(YC, (XC - l5 / 2.0f));//phi0用于计算lqr需要的theta		
    alpha = pi / 2.0f - phi0;

    if (first_flag == 0)
    {
        last_phi0 = phi0;
        first_flag = 1;
    }
    d_phi0 = (phi0 - last_phi0) / dt;//计算phi0变化率，d_phi0用于计算lqr需要的d_theta
    d_alpha = 0.0f - d_phi0;

    theta = alpha - phi;//得到状态变量1
    d_theta = (d_alpha - d_phi);//得到状态变量2

    last_phi0 = phi0;

    d_L0 = (L0 - last_L0) / dt;//腿长L0的一阶导数
    dd_L0 = (d_L0 - last_d_L0) / dt;//腿长L0的二阶导数

    last_d_L0 = d_L0;
    last_L0 = L0;

    dd_theta = (d_theta - last_d_theta) / dt;
    last_d_theta = d_theta;

}

void VMC::calc_vmc()
{
    j11 = (l1 * arm_sin_f32(phi0 - phi3) * arm_sin_f32(phi1 - phi2)) / arm_sin_f32(phi3 - phi2);
    j12 = (l1 * arm_cos_f32(phi0 - phi3) * arm_sin_f32(phi1 - phi2)) / (L0 * arm_sin_f32(phi3 - phi2));
    j21 = (l4 * arm_sin_f32(phi0 - phi2) * arm_sin_f32(phi3 - phi4)) / arm_sin_f32(phi3 - phi2);
    j22 = (l4 * arm_cos_f32(phi0 - phi2) * arm_sin_f32(phi3 - phi4)) / (L0 * arm_sin_f32(phi3 - phi2));

    torque_set[0] = j11 * F0 + j12 * Tp;//得到RightFront的输出轴期望力矩，F0为五连杆机构末端沿腿的推力 
    torque_set[1] = j21 * F0 + j22 * Tp;//得到RightBack的输出轴期望力矩，Tp为沿中心轴的力矩 

}

void VMC::calc_fn(float _feedback_t1, float _feedback_t2)
{
    Tp_reverse = (_feedback_t2 - _feedback_t1 * (j21 / j11)) * j11 / (j11 * j22 - j21 * j12);
    F0_reverse = (_feedback_t1 - j12 * Tp_reverse) / j11;
    float sin_theta = arm_sin_f32(theta);
    float cos_theta = arm_cos_f32(theta);
    float P = F0_reverse * cos_theta + Tp_reverse * sin_theta / L0;
    float dd_Zm = (INS.Accel[1] * arm_sin_f32(-phi) + INS.Accel[2] * arm_cos_f32(-phi)) - GRAVITY;
    float dd_Zw = dd_Zm - dd_L0 * cos_theta +
        2 * d_L0 * d_theta * sin_theta +
        L0 * dd_theta * sin_theta +
        L0 * d_theta * d_theta * cos_theta;
    FN = P + WHEEL_M * GRAVITY + dd_Zw * WHEEL_M;
}
