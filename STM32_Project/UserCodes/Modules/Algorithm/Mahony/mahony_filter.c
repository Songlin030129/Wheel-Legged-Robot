#include "mahony_filter.h"

/* ================= 内部函数 ================= */

/* 更新旋转矩阵（地系 -> 机体系） */
static void rotation_matrix_update(MahonyFilter *_filter)
{
    float q0 = _filter->q0;
    float q1 = _filter->q1;
    float q2 = _filter->q2;
    float q3 = _filter->q3;

    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    _filter->r_mat[0][0] = q0q0 + q1q1 - q2q2 - q3q3;
    _filter->r_mat[0][1] = 2.0f * (q1 * q2 - q0 * q3);
    _filter->r_mat[0][2] = 2.0f * (q1 * q3 + q0 * q2);

    _filter->r_mat[1][0] = 2.0f * (q1 * q2 + q0 * q3);
    _filter->r_mat[1][1] = q0q0 - q1q1 + q2q2 - q3q3;
    _filter->r_mat[1][2] = 2.0f * (q2 * q3 - q0 * q1);

    _filter->r_mat[2][0] = 2.0f * (q1 * q3 - q0 * q2);
    _filter->r_mat[2][1] = 2.0f * (q2 * q3 + q0 * q1);
    _filter->r_mat[2][2] = q0q0 - q1q1 - q2q2 + q3q3;
}

/* ================= 对外接口实现 ================= */

void mahony_init(MahonyFilter *_filter, float _kp, float _ki, float _dt)
{
    if (!_filter) {
        return;
    }

    _filter->kp = _kp;
    _filter->ki = _ki;
    _filter->dt = _dt;

    _filter->q0 = 1.0f;
    _filter->q1 = 0.0f;
    _filter->q2 = 0.0f;
    _filter->q3 = 0.0f;

    _filter->ex_int = 0.0f;
    _filter->ey_int = 0.0f;
    _filter->ez_int = 0.0f;
}

void mahony_input(MahonyFilter *_filter, Axis3f _gyro, Axis3f _acc)
{
    if (!_filter) {
        return;
    }

    _filter->gyro = _gyro;
    _filter->acc = _acc;
}

void mahony_update(MahonyFilter *_filter)
{
    if (!_filter) {
        return;
    }

    float norm;
    float ex, ey, ez;

    /* 单位化加速度 */
    norm = sqrtf(_filter->acc.x * _filter->acc.x + _filter->acc.y * _filter->acc.y +
                 _filter->acc.z * _filter->acc.z);

    if (norm <= 0.0f) {
        return;
    }

    _filter->acc.x /= norm;
    _filter->acc.y /= norm;
    _filter->acc.z /= norm;

    /* 估计重力方向与测量值的叉积误差 */
    ex = (_filter->acc.y * _filter->r_mat[2][2] - _filter->acc.z * _filter->r_mat[2][1]);

    ey = (_filter->acc.z * _filter->r_mat[2][0] - _filter->acc.x * _filter->r_mat[2][2]);

    ez = (_filter->acc.x * _filter->r_mat[2][1] - _filter->acc.y * _filter->r_mat[2][0]);

    /* 积分误差 */
    _filter->ex_int += _filter->ki * ex * _filter->dt;
    _filter->ey_int += _filter->ki * ey * _filter->dt;
    _filter->ez_int += _filter->ki * ez * _filter->dt;

    /* PI 修正陀螺仪 */
    _filter->gyro.x += _filter->kp * ex + _filter->ex_int;
    _filter->gyro.y += _filter->kp * ey + _filter->ey_int;
    _filter->gyro.z += _filter->kp * ez + _filter->ez_int;

    /* 四元数更新 */
    float half_dt = 0.5f * _filter->dt;

    float q0 = _filter->q0;
    float q1 = _filter->q1;
    float q2 = _filter->q2;
    float q3 = _filter->q3;

    _filter->q0 += (-q1 * _filter->gyro.x - q2 * _filter->gyro.y - q3 * _filter->gyro.z) * half_dt;

    _filter->q1 += (q0 * _filter->gyro.x + q2 * _filter->gyro.z - q3 * _filter->gyro.y) * half_dt;

    _filter->q2 += (q0 * _filter->gyro.y - q1 * _filter->gyro.z + q3 * _filter->gyro.x) * half_dt;

    _filter->q3 += (q0 * _filter->gyro.z + q1 * _filter->gyro.y - q2 * _filter->gyro.x) * half_dt;

    /* 四元数归一化 */
    norm = sqrtf(_filter->q0 * _filter->q0 + _filter->q1 * _filter->q1 + _filter->q2 * _filter->q2 +
                 _filter->q3 * _filter->q3);

    if (norm <= 0.0f) {
        return;
    }

    _filter->q0 /= norm;
    _filter->q1 /= norm;
    _filter->q2 /= norm;
    _filter->q3 /= norm;

    rotation_matrix_update(_filter);
}

void mahony_output(MahonyFilter *_filter)
{
    if (!_filter) {
        return;
    }

    _filter->pitch = -asinf(_filter->r_mat[2][0]);
    _filter->roll = atan2f(_filter->r_mat[2][1], _filter->r_mat[2][2]);
    _filter->yaw = atan2f(_filter->r_mat[1][0], _filter->r_mat[0][0]);
}
