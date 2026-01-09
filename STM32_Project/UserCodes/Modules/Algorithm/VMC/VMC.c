#include "VMC.h"
#include "bsp_dwt.h"
#include "common_inc.h"

//     l1 = 0.12f;    // 单位为m
//     l2 = 0.20f;    // 单位为m
//     l3 = 0.20f;    // 单位为m
//     l4 = 0.12f;    // 单位为m
//     l5 = 0.1016f;  // AE长度 //单位为m
//     wheel_m = 0.6f;

void vmc_init(VMC* _vmc, float _wheel_m, float _l1, float _l2, float _l3, float _l4, float _l5)
{
    _vmc->wheel_m = _wheel_m;
    _vmc->l1 = _l1;
    _vmc->l2 = _l2;
    _vmc->l3 = _l3;
    _vmc->l4 = _l4;
    _vmc->l5 = _l5;
}

void vmc_calc_kf(VMC* _vmc)
{
    _vmc->dt = bsp_get_dt(&_vmc->vmc_dwt_count);

    _vmc->phi = INS.Pitch;
    _vmc->d_phi = (_vmc->phi - _vmc->last_phi) / _vmc->dt;
    _vmc->last_phi = _vmc->phi;
    // d_phi = INS.Gyro[0];

    _vmc->YD = _vmc->l4 * arm_sin_f32(_vmc->phi4);             // D的y坐标
    _vmc->YB = _vmc->l1 * arm_sin_f32(_vmc->phi1);             // B的y坐标
    _vmc->XD = _vmc->l5 + _vmc->l4 * arm_cos_f32(_vmc->phi4);  // D的x坐标
    _vmc->XB = _vmc->l1 * arm_cos_f32(_vmc->phi1);             // B的x坐标

    _vmc->lBD = sqrt((_vmc->XD - _vmc->XB) * (_vmc->XD - _vmc->XB) + (_vmc->YD - _vmc->YB) * (_vmc->YD - _vmc->YB));

    _vmc->A0 = 2 * _vmc->l2 * (_vmc->XD - _vmc->XB);
    _vmc->B0 = 2 * _vmc->l2 * (_vmc->YD - _vmc->YB);
    _vmc->C0 = _vmc->l2 * _vmc->l2 + _vmc->lBD * _vmc->lBD - _vmc->l3 * _vmc->l3;
    _vmc->phi2 = 2 * atan2f((_vmc->B0 + sqrt(_vmc->A0 * _vmc->A0 + _vmc->B0 * _vmc->B0 - _vmc->C0 * _vmc->C0)), _vmc->A0 + _vmc->C0);
    _vmc->phi3 = atan2f(_vmc->YB - _vmc->YD + _vmc->l2 * arm_sin_f32(_vmc->phi2), _vmc->XB - _vmc->XD + _vmc->l2 * arm_cos_f32(_vmc->phi2));
    // C点直角坐标
    _vmc->XC = _vmc->l1 * arm_cos_f32(_vmc->phi1) + _vmc->l2 * arm_cos_f32(_vmc->phi2);
    _vmc->YC = _vmc->l1 * arm_sin_f32(_vmc->phi1) + _vmc->l2 * arm_sin_f32(_vmc->phi2);
    // C点极坐标
    _vmc->L0 = sqrt((_vmc->XC - _vmc->l5 / 2.0f) * (_vmc->XC - _vmc->l5 / 2.0f) + _vmc->YC * _vmc->YC);

    _vmc->phi0 = atan2f(_vmc->YC, (_vmc->XC - _vmc->l5 / 2.0f));  // phi0用于计算lqr需要的theta
    _vmc->alpha = _PI / 2.0f - _vmc->phi0;

    if (_vmc->first_flag == 0) {
        _vmc->last_phi0 = _vmc->phi0;
        _vmc->first_flag = 1;
    }
    _vmc->d_phi0 = (_vmc->phi0 - _vmc->last_phi0) / _vmc->dt;  // 计算phi0变化率，d_phi0用于计算lqr需要的d_theta
    _vmc->d_alpha = 0.0f - _vmc->d_phi0;

    _vmc->theta = _vmc->alpha - _vmc->phi;          // 得到状态变量1
    _vmc->d_theta = (_vmc->d_alpha - _vmc->d_phi);  // 得到状态变量2

    _vmc->last_phi0 = _vmc->phi0;

    _vmc->d_L0 = (_vmc->L0 - _vmc->last_L0) / _vmc->dt;       // 腿长L0的一阶导数
    _vmc->dd_L0 = (_vmc->d_L0 - _vmc->last_d_L0) / _vmc->dt;  // 腿长L0的二阶导数

    _vmc->last_d_L0 = _vmc->d_L0;
    _vmc->last_L0 = _vmc->L0;

    _vmc->dd_theta = (_vmc->d_theta - _vmc->last_d_theta) / _vmc->dt;
    _vmc->last_d_theta = _vmc->d_theta;
}

void vmc_calc_torque(VMC* _vmc)
{
    _vmc->j11 = (_vmc->l1 * arm_sin_f32(_vmc->phi0 - _vmc->phi3) * arm_sin_f32(_vmc->phi1 - _vmc->phi2)) / arm_sin_f32(_vmc->phi3 - _vmc->phi2);
    _vmc->j12 = (_vmc->l1 * arm_cos_f32(_vmc->phi0 - _vmc->phi3) * arm_sin_f32(_vmc->phi1 - _vmc->phi2)) / (_vmc->L0 * arm_sin_f32(_vmc->phi3 - _vmc->phi2));
    _vmc->j21 = (_vmc->l4 * arm_sin_f32(_vmc->phi0 - _vmc->phi2) * arm_sin_f32(_vmc->phi3 - _vmc->phi4)) / arm_sin_f32(_vmc->phi3 - _vmc->phi2);
    _vmc->j22 = (_vmc->l4 * arm_cos_f32(_vmc->phi0 - _vmc->phi2) * arm_sin_f32(_vmc->phi3 - _vmc->phi4)) / (_vmc->L0 * arm_sin_f32(_vmc->phi3 - _vmc->phi2));

    _vmc->torque_set[0] = _vmc->j11 * _vmc->F0 + _vmc->j12 * _vmc->Tp;  // 得到RightFront的输出轴期望力矩，F0为五连杆机构末端沿腿的推力
    _vmc->torque_set[1] = _vmc->j21 * _vmc->F0 + _vmc->j22 * _vmc->Tp;  // 得到RightBack的输出轴期望力矩，Tp为沿中心轴的力矩
}

void vmc_calc_fn(VMC* _vmc, float _feedback_t1, float _feedback_t2)
{
    _vmc->Tp_reverse = (_feedback_t2 - _feedback_t1 * (_vmc->j21 / _vmc->j11)) * _vmc->j11 / (_vmc->j11 * _vmc->j22 - _vmc->j21 * _vmc->j12);
    _vmc->F0_reverse = (_feedback_t1 - _vmc->j12 * _vmc->Tp_reverse) / _vmc->j11;
    float sin_theta = arm_sin_f32(_vmc->theta);
    float cos_theta = arm_cos_f32(_vmc->theta);
    float P = _vmc->F0_reverse * cos_theta + _vmc->Tp_reverse * sin_theta / _vmc->L0;
    float dd_Zm = (INS.Accel[1] * arm_sin_f32(-_vmc->phi) + INS.Accel[2] * arm_cos_f32(-_vmc->phi)) - _GRAVITY;
    float dd_Zw = dd_Zm - _vmc->dd_L0 * cos_theta + 2 * _vmc->d_L0 * _vmc->d_theta * sin_theta + _vmc->L0 * _vmc->dd_theta * sin_theta + _vmc->L0 * _vmc->d_theta * _vmc->d_theta * cos_theta;
    _vmc->FN = P + _vmc->wheel_m * _GRAVITY + dd_Zw * _vmc->wheel_m;
}