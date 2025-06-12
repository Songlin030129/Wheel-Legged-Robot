#include "Control_Task.h"
Control control;
// float LQR_K[12] = {
//   -5.380f,
//   -0.627f,
//   -0.253f,
//   -1.940f,
//    1.017f,
//    0.070f,
//    4.225f,
//    0.575f,
//    0.755f,
//    1.229f,
//   24.380f,
//    0.898f, };

float LQR_K[12] = { 
  -7.999f,
  -1.129f,
  -0.992f,
  -2.145f,
   1.169f,
   0.113f,
   3.146f,
   0.496f,
   0.548f,
   1.157f,
  31.590f,
   0.957f, };

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

    PID_L0_L.Init(250, 0, 8, 0, 100, 0);
    PID_L0_R.Init(250, 0, 8, 0, 100, 0);
    PID_Roll.Init(100, 0, 1, 0, 10, 0);
    PID_Tp.Init(10, 0, 0.1, 0, 2, 0);
    PID_Yaw.Init(2, 0, 0.2, 0, 2, 0);

    LPF_d_x.Init(0.05);


    Tar_Yaw = 0;
    Tar_Roll = 0;
    Tar_x = 0;
    Tar_d_x = 0;
    Tar_L0_L = 0.25;
    Tar_L0_R = 0.25;

    while (1)
    {
        dt = GetDeltaT(&control_dwt_count);
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

        /*更新控制量*/
        Tar_L0_R = Tar_L0_L;
        Tar_Yaw -= Tar_d_Yaw * dt;
        Tar_x += dt * Tar_d_x;

        /*五连杆正运动学解算，计算腿长 L0, 状态变量 theta, d_theta, phi, d_phi*/
        vmc_left.phi1 = pi / 2.0f - Joint_Motor_Left_1.Position;
        vmc_left.phi4 = pi / 2.0f - Joint_Motor_Left_2.Position;
        vmc_left.calc_kf();

        vmc_right.phi1 = pi / 2.0f + Joint_Motor_Right_1.Position;
        vmc_right.phi4 = pi / 2.0f + Joint_Motor_Right_2.Position;
        vmc_right.calc_kf();

        /*获取状态变量 x, d_x*/
        sys_state_l.theta = vmc_left.theta;
        sys_state_r.theta = vmc_right.theta;

        sys_state_l.d_theta = vmc_left.d_theta;
        sys_state_r.d_theta = vmc_right.d_theta;

        sys_state_l.phi = vmc_left.phi;
        sys_state_r.phi = vmc_right.phi;

        sys_state_l.d_phi = vmc_left.d_phi;
        sys_state_r.d_phi = vmc_right.d_phi;

        // sys_state_l.d_x = LPF_d_x_l(Wheel_Motor_Left.Velocity * WHEEL_RADIUS +
        //     vmc_left.L0 * vmc_left.d_theta * arm_cos_f32(vmc_left.theta) +
        //     vmc_left.d_L0 * arm_sin_f32(vmc_left.theta));
        // sys_state_r.d_x = LPF_d_x_r(-Wheel_Motor_Right.Velocity * WHEEL_RADIUS +
        //     vmc_right.L0 * vmc_right.d_theta * arm_cos_f32(vmc_right.theta) +
        //     vmc_right.d_L0 * arm_sin_f32(vmc_right.theta));


        sys_state_l.d_x = LPF_d_x(Wheel_Motor_Left.Velocity * WHEEL_RADIUS - Wheel_Motor_Right.Velocity * WHEEL_RADIUS) * 0.5f;
        sys_state_r.d_x = sys_state_l.d_x;

        sys_state_l.x += (sys_state_l.d_x + sys_state_r.d_x) * dt * 0.5f;
        sys_state_r.x += (sys_state_l.d_x + sys_state_r.d_x) * dt * 0.5f;


        /*根据腿长L0获取LQR反馈增益K矩阵*/
        for (int i = 0;i < 12;i++)
        {
            LQR_K_L[i] = LQR_K[i];
        }
        for (int i = 0;i < 12;i++)
        {
            LQR_K_R[i] = LQR_K[i];
        }

        /*计算PID*/
        PID_Yaw.Cal(Tar_Yaw - INS.YawTotalAngle); // Yaw轴补偿
        PID_Roll.Cal(Tar_Roll - INS.Roll); // Roll轴补偿
        PID_Tp.Cal(vmc_right.theta - vmc_left.theta); // 双腿角度协调控制
        PID_L0_L.Cal(Tar_L0_L - vmc_left.L0); // 左腿腿长控制
        PID_L0_R.Cal(Tar_L0_R - vmc_right.L0); // 右腿腿长控制

        /*根据电机反馈力矩vmc逆解算机体支持力*/
        vmc_left.calc_fn(-Joint_Motor_Left_1.Torque, -Joint_Motor_Left_2.Torque);
        vmc_right.calc_fn(Joint_Motor_Right_1.Torque, Joint_Motor_Right_2.Torque);
        // printf("FN_Left:%f, FN_Right:%f\r\n", vmc_left.FN, vmc_right.FN);


        /*根据状态变量(x)与反馈增益(K)计算输入(u), u = -K(x - xd), 加入Yaw轴补偿, 离地检测*/
        if (vmc_left.FN >= 15) {
            sys_input_l.wheel_T = -(
                LQR_K_L[0] * (sys_state_l.theta - 0.0f) +
                LQR_K_L[1] * (sys_state_l.d_theta - 0.0f) +
                LQR_K_L[2] * (sys_state_l.x - Tar_x) +
                LQR_K_L[3] * (sys_state_l.d_x - Tar_d_x) +
                LQR_K_L[4] * (sys_state_l.phi - 0.0f) +
                LQR_K_L[5] * (sys_state_l.d_phi - 0.0f));
            sys_input_l.joint_Tp = -(
                LQR_K_L[6] * (sys_state_l.theta - 0.0f) +
                LQR_K_L[7] * (sys_state_l.d_theta - 0.0f) +
                LQR_K_L[8] * (sys_state_l.x - Tar_x) +
                LQR_K_L[9] * (sys_state_l.d_x - Tar_d_x) +
                LQR_K_L[10] * (sys_state_l.phi - 0.0f) +
                LQR_K_L[11] * (sys_state_l.d_phi - 0.0f));

            sys_input_l.wheel_T -= PID_Yaw.output_value;
        }
        else {
            sys_input_l.wheel_T = 0;
            sys_input_l.joint_Tp = -(
                LQR_K_L[6] * (sys_state_l.theta - 0.0f) +
                LQR_K_L[7] * (sys_state_l.d_theta - 0.0f));
        }
        // printf("theta:%f, d_theta:%f, x:%f, d_x:%f, phi:%f, d_phi:%f\r\n", vmc_left.theta, vmc_left.d_theta, x_L, d_x_L, vmc_left.phi, vmc_left.d_phi);

        if (vmc_right.FN >= 15) {
            sys_input_r.wheel_T = -(
                LQR_K_R[0] * (sys_state_r.theta - 0.0f) +
                LQR_K_R[1] * (sys_state_r.d_theta - 0.0f) +
                LQR_K_R[2] * (sys_state_r.x - Tar_x) +
                LQR_K_R[3] * (sys_state_r.d_x - Tar_d_x) +
                LQR_K_R[4] * (sys_state_r.phi - 0.0f) +
                LQR_K_R[5] * (sys_state_r.d_phi - 0.0f));
            sys_input_r.joint_Tp = -(
                LQR_K_R[6] * (sys_state_r.theta - 0.0f) +
                LQR_K_R[7] * (sys_state_r.d_theta - 0.0f) +
                LQR_K_R[8] * (sys_state_r.x - Tar_x) +
                LQR_K_R[9] * (sys_state_r.d_x - Tar_d_x) +
                LQR_K_R[10] * (sys_state_r.phi - 0.0f) +
                LQR_K_R[11] * (sys_state_r.d_phi - 0.0f));

            sys_input_r.wheel_T += PID_Yaw.output_value;
        }
        else {
            sys_input_r.wheel_T = 0;
            sys_input_r.joint_Tp = -(
                LQR_K_R[6] * (sys_state_r.theta - 0.0f) +
                LQR_K_R[7] * (sys_state_r.d_theta - 0.0f));
        }
        // printf("theta:%f, d_theta:%f, x:%f, d_x:%f, phi:%f, d_phi:%f\r\n", vmc_right.theta, vmc_right.d_theta, x_R, d_x_R, vmc_right.phi, vmc_right.d_phi);

        /*获取vmc正解所需的杆力矩(Tp)和支持力(F0), 加入双腿协调控制, Roll轴补偿*/
        vmc_left.Tp = -sys_input_l.joint_Tp;
        vmc_right.Tp = -sys_input_r.joint_Tp;

        vmc_left.Tp -= PID_Tp.output_value;
        vmc_right.Tp += PID_Tp.output_value;

        vmc_left.F0 = WIGHT_GAIN + PID_L0_L.output_value;
        vmc_right.F0 = WIGHT_GAIN + PID_L0_R.output_value;

        vmc_left.F0 += PID_Roll.output_value;
        vmc_right.F0 -= PID_Roll.output_value;
        mySaturate(&vmc_left.F0, -100.0f, 100.0f);
        mySaturate(&vmc_right.F0, -100.0f, 100.0f);

        /*根据F0和Tp, vmc正解得出两关节电机控制力矩T1和T2*/
        vmc_left.calc_vmc();
        vmc_right.calc_vmc();

        /*电机控制力矩限幅*/
        mySaturate(&vmc_left.torque_set[0], -3.0f, 3.0f);
        mySaturate(&vmc_left.torque_set[1], -3.0f, 3.0f);
        mySaturate(&vmc_right.torque_set[0], -3.0f, 3.0f);
        mySaturate(&vmc_right.torque_set[1], -3.0f, 3.0f);
        mySaturate(&sys_input_l.wheel_T, -1.5f, 1.5f);
        mySaturate(&sys_input_r.wheel_T, -1.5f, 1.5f);

        /*发送电机控制信号*/
        Joint_Motor_Left_1.Control(0, 0, 0, 0, -vmc_left.torque_set[0]);
        Joint_Motor_Right_1.Control(0, 0, 0, 0, vmc_right.torque_set[0]);
        vTaskDelay(1);
        Joint_Motor_Left_2.Control(0, 0, 0, 0, -vmc_left.torque_set[1]);
        Joint_Motor_Right_2.Control(0, 0, 0, 0, vmc_right.torque_set[1]);
        vTaskDelay(1);
        Wheel_Motor_Left.Control(0, 0, 0, 0, sys_input_l.wheel_T);
        Wheel_Motor_Right.Control(0, 0, 0, 0, -sys_input_r.wheel_T);
        vTaskDelay(1);

        // Joint_Motor_Left_1.Control(0, 0, 0, 0, 0);
        // Joint_Motor_Right_1.Control(0, 0, 0, 0, 0);
        // vTaskDelay(1);
        // Joint_Motor_Left_2.Control(0, 0, 0, 0, 0);
        // Joint_Motor_Right_2.Control(0, 0, 0, 0, 0);
        // vTaskDelay(1);
        // Wheel_Motor_Left.Control(0, 0, 0, 0, 0);
        // Wheel_Motor_Right.Control(0, 0, 0, 0, 0);
        // vTaskDelay(1);

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