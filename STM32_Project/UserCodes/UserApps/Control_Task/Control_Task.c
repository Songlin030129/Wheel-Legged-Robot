#include "Control_Task.h"
#include "DMMotor.h"
#include "LKMotor.h"
#include "LPF.h"
#include "PID.h"
#include "VMC.h"
#include "bsp_dwt.h"
#include "fdcan.h"
Control control;

// float Poly_Coefficient[12][4] = {
// {  -83.887f,   84.362f,  -47.914f,   -1.953f},
// {    6.225f,   -5.470f,   -4.232f,   -0.193f},
// {   -0.280f,    0.457f,   -0.222f,   -1.377f},
// {    2.741f,   -1.128f,   -0.755f,   -2.792f},
// {    5.008f,   13.845f,  -13.343f,    4.110f},
// {   -1.863f,    2.203f,   -1.024f,    0.292f},
// {  131.481f,  -96.885f,   21.592f,    0.984f},
// {   13.505f,  -11.082f,    3.174f,    0.064f},
// {    6.651f,    0.137f,   -2.836f,    1.020f},
// {   14.804f,   -1.274f,   -5.124f,    2.017f},
// {   16.452f,  -21.772f,    9.681f,   43.438f},
// {    0.968f,   -1.324f,    0.626f,    0.956f} };
float Poly_Coefficient[12][4] = {{-88.969f, 92.009f, -58.901f, -2.321f}, {8.477f, -8.134f, -5.531f, -0.238f}, {-1.134f, 1.272f, -0.504f, -3.087f},  {5.591f, -2.643f, -1.451f, -3.946f},
                                 {-14.836f, 32.931f, -20.541f, 5.471f},  {-2.679f, 2.938f, -1.277f, 0.359f},  {98.727f, -73.032f, 16.346f, 0.977f}, {10.128f, -8.361f, 2.434f, 0.077f},
                                 {1.499f, 6.036f, -5.451f, 1.557f},      {3.912f, 5.480f, -6.101f, 1.926f},   {23.726f, -25.035f, 9.568f, 43.612f}, {1.173f, -1.354f, 0.579f, 0.970f}};

static float LQR_Get_K(float* coe, float len)
{
    return coe[0] * len * len * len + coe[1] * len * len + coe[2] * len + coe[3];
}

static void mySaturate(float* in, float min, float max)
{
    if (*in < min) {
        *in = min;
    } else if (*in > max) {
        *in = max;
    }
}

void Control_Task()
{
    while (INS.ins_flag == 0) {
        vTaskDelay(1);
    }
    dmmotor_init(&joint_motor_left_1, &hfdcan1, 4, 3, CONTROL_MODE_MIT);
    dmmotor_init(&joint_motor_left_2, &hfdcan1, 6, 5, CONTROL_MODE_MIT);
    lkmotor_init(&wheel_motor_left, &hfdcan1, 1);

    dmmotor_init(&joint_motor_right_1, &hfdcan2, 4, 3, CONTROL_MODE_MIT);
    dmmotor_init(&joint_motor_right_2, &hfdcan2, 6, 5, CONTROL_MODE_MIT);
    lkmotor_init(&wheel_motor_right, &hfdcan1, 2);

    dmmotor_enable(&joint_motor_left_1);
    vTaskDelay(1);
    dmmotor_enable(&joint_motor_left_2);
    vTaskDelay(1);
    lkmotor_enable(&wheel_motor_left);
    vTaskDelay(1);
    dmmotor_enable(&joint_motor_right_1);
    vTaskDelay(1);
    dmmotor_enable(&joint_motor_right_2);
    vTaskDelay(1);
    lkmotor_enable(&wheel_motor_right);
    vTaskDelay(1);

    vmc_init(&control.vmc_left, 0.6f, 0.12f, 0.2f, 0.2f, 0.12f, 0.1016f);
    vmc_init(&control.vmc_right, 0.6f, 0.12f, 0.2f, 0.2f, 0.12f, 0.1016f);

    pid_init(&control.PID_L0_L, 300, 0, 10, 0, 100, 0);
    pid_init(&control.PID_L0_R, 300, 0, 10, 0, 100, 0);
    pid_init(&control.PID_Roll, 100, 0, 1, 0, 10, 0);
    pid_init(&control.PID_Tp, 10, 0, 0, 0, 2, 0);
    pid_init(&control.PID_Yaw, 2.0f, 0, 0.4f, 0, 2, 0);

    lpf_init(&control.LPF_d_x, 0.05f);
    lpf_init(&control.LPF_tar_yaw, 0.05f);
    lpf_init(&control.LPF_tar_L0, 0.05f);
    lpf_init(&control.LPF_wheelL_T, 0.02f);
    lpf_init(&control.LPF_wheelR_T, 0.02f);

    control.Tar_Yaw = 0;
    control.Tar_Roll = 0;
    control.Tar_x = 0;
    control.Tar_d_x = 0;
    control.Tar_L0 = 0.15;

    while (1) {
        control.dt = bsp_get_dt(&control.control_dwt_count);
        if (control.Control_Enable <= 0.5f) {
            dmmotor_control_mit(&joint_motor_left_1, 0, 0, 0, 0, 0);
            vTaskDelay(1);
            dmmotor_control_mit(&joint_motor_left_2, 0, 0, 0, 0, 0);
            vTaskDelay(1);
            lkmotor_control(&wheel_motor_right, 0);
            vTaskDelay(1);

            dmmotor_control_mit(&joint_motor_right_1, 0, 0, 0, 0, 0);
            vTaskDelay(1);
            dmmotor_control_mit(&joint_motor_right_2, 0, 0, 0, 0, 0);
            vTaskDelay(1);
            lkmotor_control(&wheel_motor_right, 0);
            vTaskDelay(1);
        } else {
            /*五连杆正运动学解算，计算腿长 L0, 状态变量 theta, d_theta, phi, d_phi*/
            control.vmc_left.phi1 = _PI / 2.0f - joint_motor_left_1.position;
            control.vmc_left.phi4 = _PI / 2.0f - joint_motor_left_2.position;
            vmc_calc_kf(&control.vmc_left);

            control.vmc_right.phi1 = _PI / 2.0f + joint_motor_right_1.position;
            control.vmc_right.phi4 = _PI / 2.0f + joint_motor_right_2.position;
            vmc_calc_kf(&control.vmc_right);

            /*获取状态变量 x, d_x*/
            control.sys_state_l.theta = control.vmc_left.theta;
            control.sys_state_r.theta = control.vmc_right.theta;

            control.sys_state_l.d_theta = control.vmc_left.d_theta;
            control.sys_state_r.d_theta = control.vmc_right.d_theta;

            control.sys_state_l.phi = control.vmc_left.phi;
            control.sys_state_r.phi = control.vmc_right.phi;

            control.sys_state_l.d_phi = control.vmc_left.d_phi;
            control.sys_state_r.d_phi = control.vmc_right.d_phi;

            control.sys_state_l.d_x = lpf_cal(&control.LPF_d_x, wheel_motor_left.velocity * WHEEL_RADIUS - wheel_motor_right.velocity * WHEEL_RADIUS) * 0.5f;
            control.sys_state_r.d_x = control.sys_state_l.d_x;

            control.sys_state_l.x += (control.sys_state_l.d_x + control.sys_state_r.d_x) * control.dt * 0.5f;
            control.sys_state_r.x += (control.sys_state_l.d_x + control.sys_state_r.d_x) * control.dt * 0.5f;

            /*根据腿长L0获取LQR反馈增益K矩阵*/
            for (int i = 0; i < 12; i++) {
                control.LQR_K_L[i] = LQR_Get_K(&Poly_Coefficient[i][0], control.vmc_left.L0);
            }
            for (int i = 0; i < 12; i++) {
                control.LQR_K_R[i] = LQR_Get_K(&Poly_Coefficient[i][0], control.vmc_right.L0);
            }

            /*计算PID*/
            pid_cal(&control.PID_Yaw, lpf_cal(&control.LPF_tar_yaw, control.Tar_Yaw) - INS.YawTotalAngle);  // Yaw轴补偿
            pid_cal(&control.PID_Roll, control.Tar_Roll - INS.Roll);                                        // Roll轴补偿
            pid_cal(&control.PID_Tp, control.vmc_right.theta - control.vmc_left.theta);                     // 双腿角度协调控制

            lpf_cal(&control.LPF_tar_L0, control.Tar_L0);
            pid_cal(&control.PID_L0_L, control.LPF_tar_L0.output - control.vmc_left.L0);   // 左腿腿长控制
            pid_cal(&control.PID_L0_R, control.LPF_tar_L0.output - control.vmc_right.L0);  // 右腿腿长控制

            /*根据电机反馈力矩vmc逆解算机体支持力*/
            vmc_calc_fn(&control.vmc_left, -joint_motor_left_1.torque, -joint_motor_left_2.torque);
            vmc_calc_fn(&control.vmc_right, joint_motor_right_1.torque, joint_motor_right_2.torque);

            /*根据状态变量(x)与反馈增益(K)计算输入(u), u = -K(x - xd), 加入Yaw轴补偿, 离地检测*/
            if (control.vmc_left.FN >= 15) {
                control.sys_input_l.wheel_T = -(control.LQR_K_L[0] * (control.sys_state_l.theta - 0.0f) + control.LQR_K_L[1] * (control.sys_state_l.d_theta - 0.0f) +
                                                control.LQR_K_L[2] * (control.sys_state_l.x - control.Tar_x) + control.LQR_K_L[3] * (control.sys_state_l.d_x - control.Tar_d_x) +
                                                control.LQR_K_L[4] * (control.sys_state_l.phi - 0.0f) + control.LQR_K_L[5] * (control.sys_state_l.d_phi - 0.0f));
                control.sys_input_l.joint_Tp = -(control.LQR_K_L[6] * (control.sys_state_l.theta - 0.0f) + control.LQR_K_L[7] * (control.sys_state_l.d_theta - 0.0f) +
                                                 control.LQR_K_L[8] * (control.sys_state_l.x - control.Tar_x) + control.LQR_K_L[9] * (control.sys_state_l.d_x - control.Tar_d_x) +
                                                 control.LQR_K_L[10] * (control.sys_state_l.phi - 0.0f) + control.LQR_K_L[11] * (control.sys_state_l.d_phi - 0.0f));

                control.sys_input_l.wheel_T -= control.PID_Yaw.output_value;
            } else {
                control.sys_input_l.wheel_T = 0;
                control.sys_input_l.joint_Tp = -(control.LQR_K_L[6] * (control.sys_state_l.theta - 0.0f) + control.LQR_K_L[7] * (control.sys_state_l.d_theta - 0.0f));
            }

            if (control.vmc_right.FN >= 15) {
                control.sys_input_r.wheel_T = -(control.LQR_K_R[0] * (control.sys_state_r.theta - 0.0f) + control.LQR_K_R[1] * (control.sys_state_r.d_theta - 0.0f) +
                                                control.LQR_K_R[2] * (control.sys_state_r.x - control.Tar_x) + control.LQR_K_R[3] * (control.sys_state_r.d_x - control.Tar_d_x) +
                                                control.LQR_K_R[4] * (control.sys_state_r.phi - 0.0f) + control.LQR_K_R[5] * (control.sys_state_r.d_phi - 0.0f));
                control.sys_input_r.joint_Tp = -(control.LQR_K_R[6] * (control.sys_state_r.theta - 0.0f) + control.LQR_K_R[7] * (control.sys_state_r.d_theta - 0.0f) +
                                                 control.LQR_K_R[8] * (control.sys_state_r.x - control.Tar_x) + control.LQR_K_R[9] * (control.sys_state_r.d_x - control.Tar_d_x) +
                                                 control.LQR_K_R[10] * (control.sys_state_r.phi - 0.0f) + control.LQR_K_R[11] * (control.sys_state_r.d_phi - 0.0f));

                control.sys_input_r.wheel_T += control.PID_Yaw.output_value;
            } else {
                control.sys_input_r.wheel_T = 0;
                control.sys_input_r.joint_Tp = -(control.LQR_K_R[6] * (control.sys_state_r.theta - 0.0f) + control.LQR_K_R[7] * (control.sys_state_r.d_theta - 0.0f));
            }

            /*获取vmc正解所需的杆力矩(Tp)和支持力(F0), 加入双腿协调控制, Roll轴补偿*/
            control.vmc_left.Tp = -control.sys_input_l.joint_Tp;
            control.vmc_right.Tp = -control.sys_input_r.joint_Tp;

            control.vmc_left.Tp -= control.PID_Tp.output_value;
            control.vmc_right.Tp += control.PID_Tp.output_value;

            control.vmc_left.F0 = WIGHT_GAIN + control.PID_L0_L.output_value;
            control.vmc_right.F0 = WIGHT_GAIN + control.PID_L0_R.output_value;

            control.vmc_left.F0 += control.PID_Roll.output_value;
            control.vmc_right.F0 -= control.PID_Roll.output_value;
            mySaturate(&control.vmc_left.F0, -100.0f, 100.0f);
            mySaturate(&control.vmc_right.F0, -100.0f, 100.0f);

            /*根据F0和Tp, vmc正解得出两关节电机控制力矩T1和T2*/
            vmc_calc_torque(&control.vmc_left);
            vmc_calc_torque(&control.vmc_right);

            /*电机控制力矩限幅*/
            mySaturate(&control.vmc_left.torque_set[0], -3.0f, 3.0f);
            mySaturate(&control.vmc_left.torque_set[1], -3.0f, 3.0f);
            mySaturate(&control.vmc_right.torque_set[0], -3.0f, 3.0f);
            mySaturate(&control.vmc_right.torque_set[1], -3.0f, 3.0f);
            mySaturate(&control.sys_input_l.wheel_T, -4.0f, 4.0f);
            mySaturate(&control.sys_input_r.wheel_T, -4.0f, 4.0f);

            /*发送电机控制信号*/
            dmmotor_control_mit(&joint_motor_left_1, 0, 0, 0, 0, -control.vmc_left.torque_set[0]);
            dmmotor_control_mit(&joint_motor_right_1, 0, 0, 0, 0, control.vmc_right.torque_set[0]);
            vTaskDelay(1);
            dmmotor_control_mit(&joint_motor_left_2, 0, 0, 0, 0, -control.vmc_left.torque_set[1]);
            dmmotor_control_mit(&joint_motor_right_2, 0, 0, 0, 0, control.vmc_right.torque_set[1]);
            vTaskDelay(1);
            lkmotor_control(&wheel_motor_left, lpf_cal(&control.LPF_wheelL_T, control.sys_input_l.wheel_T));
            lkmotor_control(&wheel_motor_right, -lpf_cal(&control.LPF_wheelR_T, control.sys_input_r.wheel_T));
            vTaskDelay(1);
        }
    }
}
