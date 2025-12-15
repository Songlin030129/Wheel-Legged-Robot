/**
  *********************************************************************
  * @file      ins_task.c/h
  * @brief     该任务是用mahony方法获取机体姿态，同时获取机体在绝对坐标系下的运动加速度
  * @note
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */

#include "INS_Task.h"
#include "bsp_dwt.h"

INS_t INS;

void INS_task(void)
{
    /* BMI088初始化 */
    while (BMI088_init(&hspi2, 0) != BMI088_NO_ERROR) {
        ;
    }
    mahony_init(&INS.mahony, 1.0f, 0.0f, 0.001f);
    INS.AccelLPF = 0.0089f;
    INS.ins_dt = 0.0f;
    INS.INS_DWT_Count = 0;
    while (1) {
        INS.ins_dt = bsp_dwt_get_dt(&INS.INS_DWT_Count);

        INS.mahony.dt = INS.ins_dt;

        BMI088_Read(&BMI088);

        INS.Accel[0] = BMI088.Accel[0];
        INS.Accel[1] = BMI088.Accel[1];
        INS.Accel[2] = BMI088.Accel[2];
        INS.Gyro[0] = BMI088.Gyro[0];
        INS.Gyro[1] = BMI088.Gyro[1];
        INS.Gyro[2] = BMI088.Gyro[2];
        Axis3f Gyro, Accel;
        Gyro.x = BMI088.Gyro[0];
        Gyro.y = BMI088.Gyro[1];
        Gyro.z = BMI088.Gyro[2];
        Accel.x = BMI088.Accel[0];
        Accel.y = BMI088.Accel[1];
        Accel.z = BMI088.Accel[2];

        mahony_input(&INS.mahony, Gyro, Accel);
        mahony_update(&INS.mahony);
        mahony_output(&INS.mahony);

        INS.q[0] = INS.mahony.q0;
        INS.q[1] = INS.mahony.q1;
        INS.q[2] = INS.mahony.q2;
        INS.q[3] = INS.mahony.q3;

        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
        float gravity_b[3];
        float gravity[3] = {0, 0, 9.81f};
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; i++)  // 同样过一个低通滤波
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * INS.ins_dt / (INS.AccelLPF + INS.ins_dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + INS.ins_dt);
        }
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q);  // 转换回导航系n

        // 死区处理
        if (fabsf(INS.MotionAccel_n[0]) < 0.02f) {
            INS.MotionAccel_n[0] = 0.0f;  // x轴
        }
        if (fabsf(INS.MotionAccel_n[1]) < 0.02f) {
            INS.MotionAccel_n[1] = 0.0f;  // y轴
        }
        if (fabsf(INS.MotionAccel_n[2]) < 0.04f) {
            INS.MotionAccel_n[2] = 0.0f;  // z轴
        }

        if (INS.ins_time > 3000.0f) {
            INS.ins_flag = 1;  // 四元数基本收敛，加速度也基本收敛，可以开始底盘任务
            // 获取最终数据
            INS.Pitch = INS.mahony.roll - PITCH_OFFSET;
            INS.Roll = INS.mahony.pitch - ROLL_OFFSET;
            INS.Yaw = INS.mahony.yaw;

            if (INS.Yaw - INS.YawAngleLast > 3.1415926f) {
                INS.YawRoundCount--;
            } else if (INS.Yaw - INS.YawAngleLast < -3.1415926f) {
                INS.YawRoundCount++;
            }
            INS.YawTotalAngle = 6.283f * INS.YawRoundCount + INS.Yaw;
            INS.YawAngleLast = INS.Yaw;
        } else {
            INS.ins_time++;
        }

        vTaskDelay(1);
    }
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float* vecBF, float* vecEF, float* q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] + (q[1] * q[2] - q[0] * q[3]) * vecBF[1] + (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] + (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] + (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] + (q[2] * q[3] + q[0] * q[1]) * vecBF[1] + (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float* vecEF, float* vecBF, float* q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] + (q[1] * q[2] + q[0] * q[3]) * vecEF[1] + (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] + (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] + (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] + (q[2] * q[3] - q[0] * q[1]) * vecEF[1] + (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}
