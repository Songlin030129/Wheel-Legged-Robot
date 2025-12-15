#ifndef MAHONY_FILTER_H
#define MAHONY_FILTER_H

#include <math.h>
#include <stdint.h>

/* ================= 宏定义 ================= */

#define DEG2RAD 0.0174532925f
#define RAD2DEG 57.2957795f

/* ================= 基础类型 ================= */

typedef struct {
    float x;
    float y;
    float z;
} Axis3f;

/* ================= Mahony 滤波器结构体 ================= */

typedef struct {
    /* 参数 */
    float kp;
    float ki;
    float dt;

    /* 传感器输入 */
    Axis3f gyro;
    Axis3f acc;

    /* 误差积分 */
    float ex_int;
    float ey_int;
    float ez_int;

    /* 四元数 */
    float q0;
    float q1;
    float q2;
    float q3;

    /* 旋转矩阵（地系 -> 机体系） */
    float r_mat[3][3];

    /* 欧拉角输出（rad） */
    float roll;
    float pitch;
    float yaw;

} MahonyFilter;

/* ================= 对外接口 ================= */

#ifdef __cplusplus
extern "C" {
#endif

void mahony_init(MahonyFilter *_filter, float _kp, float _ki, float _dt);

void mahony_input(MahonyFilter *_filter, Axis3f _gyro, Axis3f _acc);

void mahony_update(MahonyFilter *_filter);

void mahony_output(MahonyFilter *_filter);

#ifdef __cplusplus
}
#endif

#endif /* MAHONY_FILTER_H */
