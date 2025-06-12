#include "Control_Task.h"
Control control;

void Control::Task()
{
    while (INS.ins_flag == 0) {
        vTaskDelay(1);
    }
    Joint_Motor_Left_1.Init(&hfdcan1, MOTOR_TYPE_JOINT, 4, 3, CONTROL_MODE_MIT);
    Joint_Motor_Left_2.Init(&hfdcan1, MOTOR_TYPE_JOINT, 6, 5, CONTROL_MODE_MIT);
    Wheel_Motor_Left.Init(&hfdcan1, MOTOR_TYPE_WHEEL, 2, 1, CONTROL_MODE_MIT);

    Joint_Motor_Right_1.Init(&hfdcan2, MOTOR_TYPE_JOINT, 4, 3, CONTROL_MODE_MIT);
    Joint_Motor_Right_2.Init(&hfdcan2, MOTOR_TYPE_JOINT, 6, 5, CONTROL_MODE_MIT);
    Wheel_Motor_Right.Init(&hfdcan2, MOTOR_TYPE_WHEEL, 2, 1, CONTROL_MODE_MIT);

    Joint_Motor_Left_1.Enable();
    vTaskDelay(1);
    Joint_Motor_Left_2.Enable();
    vTaskDelay(1);
    Wheel_Motor_Left.Enable();
    vTaskDelay(1);
    Joint_Motor_Right_1.Enable();
    vTaskDelay(1);
    Joint_Motor_Right_2.Enable();
    vTaskDelay(1);
    Wheel_Motor_Right.Enable();
    vTaskDelay(1);

    vmc_left.init();
    vmc_right.init();

    PID_L0_L.Init(300, 10, 10, 0, 100, 0);
    PID_L0_R.Init(300, 10, 10, 0, 100, 0);

    Tar_L0_L = 0.25;
    Tar_L0_R = 0.25;

    while (1)
    {
        dt = DWT_GetDeltaT(&control_dwt_count);
        if (Control_Enable <= 0.5f) {
            Joint_Motor_Left_1.Control(0, 0, 0, 0, 0);
            vTaskDelay(1);
            Joint_Motor_Left_2.Control(0, 0, 0, 0, 0);
            vTaskDelay(1);
            Wheel_Motor_Left.Control(0, 0, 0, 0, 0);
            vTaskDelay(1);

            Joint_Motor_Right_1.Control(0, 0, 0, 0, 0);
            vTaskDelay(1);
            Joint_Motor_Right_2.Control(0, 0, 0, 0, 0);
            vTaskDelay(1);
            Wheel_Motor_Right.Control(0, 0, 0, 0, 0);
            vTaskDelay(1);
            return;
        }

        vmc_left.phi1 = pi / 2.0f - Joint_Motor_Left_1.Position;
        vmc_left.phi4 = pi / 2.0f - Joint_Motor_Left_2.Position;
        vmc_left.calc_kf_left();

        vmc_right.phi1 = pi / 2.0f + Joint_Motor_Right_1.Position;
        vmc_right.phi4 = pi / 2.0f + Joint_Motor_Right_2.Position;
        vmc_right.calc_kf_right();

        PID_L0_L.Cal(Tar_L0_L - vmc_left.L0);
        PID_L0_R.Cal(Tar_L0_R - vmc_right.L0);

        // vmc_left.Tp = 0;
        // vmc_right.Tp = 0;

        vmc_left.F0 = PID_L0_L.output_value;
        vmc_right.F0 = PID_L0_R.output_value;

        mySaturate(&vmc_left.F0, -100.0f, 100.0f);
        mySaturate(&vmc_right.F0, -100.0f, 100.0f);

        vmc_left.calc_vmc();
        vmc_right.calc_vmc();

        mySaturate(&vmc_left.torque_set[0], -3.0f, 3.0f);
        mySaturate(&vmc_left.torque_set[1], -3.0f, 3.0f);
        mySaturate(&vmc_right.torque_set[0], -3.0f, 3.0f);
        mySaturate(&vmc_right.torque_set[1], -3.0f, 3.0f);


        Joint_Motor_Left_1.Control(0, 0, 0, 0, -vmc_left.torque_set[0]);
        Joint_Motor_Right_1.Control(0, 0, 0, 0, vmc_right.torque_set[0]);
        vTaskDelay(1);
        Joint_Motor_Left_2.Control(0, 0, 0, 0, -vmc_left.torque_set[1]);
        Joint_Motor_Right_2.Control(0, 0, 0, 0, vmc_right.torque_set[1]);
        vTaskDelay(1);

    }

}

float Control::LQR_Get_K(float* coe, float len)
{
    return coe[0] * len * len * len + coe[1] * len * len + coe[2] * len + coe[3];
}

void Control::mySaturate(float* in, float min, float max)
{
    if (*in < min)
    {
        *in = min;
    }
    else if (*in > max)
    {
        *in = max;
    }
}

uint8_t Control::myJudge(float in, float min, float max)
{
    if (in >= min && in <= max)
        return 1;
    else
        return 0;
}
