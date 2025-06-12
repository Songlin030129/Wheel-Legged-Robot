#include "Control_Task.h"
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
float Poly_Coefficient[12][4] = {
    {  -88.969f,   92.009f,  -58.901f,   -2.321f},
    {    8.477f,   -8.134f,   -5.531f,   -0.238f},
    {   -1.134f,    1.272f,   -0.504f,   -3.087f},
    {    5.591f,   -2.643f,   -1.451f,   -3.946f},
    {  -14.836f,   32.931f,  -20.541f,    5.471f},
    {   -2.679f,    2.938f,   -1.277f,    0.359f},
    {   98.727f,  -73.032f,   16.346f,    0.977f},
    {   10.128f,   -8.361f,    2.434f,    0.077f},
    {    1.499f,    6.036f,   -5.451f,    1.557f},
    {    3.912f,    5.480f,   -6.101f,    1.926f},
    {   23.726f,  -25.035f,    9.568f,   43.612f},
    {    1.173f,   -1.354f,    0.579f,    0.970f} };

void Control::Task()
{
    while (INS.ins_flag == 0) {
        vTaskDelay(1);
    }
    Joint_Motor_Left_1.Init(&hfdcan1, MOTOR_TYPE_JOINT, 4, 3, CONTROL_MODE_MIT);
    Joint_Motor_Left_2.Init(&hfdcan1, MOTOR_TYPE_JOINT, 6, 5, CONTROL_MODE_MIT);
    Wheel_Motor_Left.Init(&hfdcan1, 1);

    Joint_Motor_Right_1.Init(&hfdcan2, MOTOR_TYPE_JOINT, 4, 3, CONTROL_MODE_MIT);
    Joint_Motor_Right_2.Init(&hfdcan2, MOTOR_TYPE_JOINT, 6, 5, CONTROL_MODE_MIT);
    Wheel_Motor_Right.Init(&hfdcan2, 1);

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

    PID_L0_L.Init(300, 0, 10, 0, 100, 0);
    PID_L0_R.Init(300, 0, 10, 0, 100, 0);
    PID_Roll.Init(100, 0, 1, 0, 10, 0);
    PID_Tp.Init(10, 0, 0, 0, 2, 0);
    PID_Yaw.Init(2.0, 0, 0.4, 0, 2, 0);

    LPF_d_x.Init(0.05);
    LPF_tar_yaw.Init(0.05);
    LPF_tar_L0.Init(0.05);
    LPF_wheelL_T.Init(0.02);
    LPF_wheelR_T.Init(0.02);

    Tar_Yaw = 0;
    Tar_Roll = 0;
    Tar_x = 0;
    Tar_d_x = 0;
    Tar_L0 = 0.15;

    while (1)
    {
        dt = GetDeltaT(&control_dwt_count);
        if (Control_Enable <= 0.5f) {
            Joint_Motor_Left_1.Control(0, 0, 0, 0, 0);
            vTaskDelay(1);
            Joint_Motor_Left_2.Control(0, 0, 0, 0, 0);
            vTaskDelay(1);
            Wheel_Motor_Left.Control(0);
            vTaskDelay(1);

            Joint_Motor_Right_1.Control(0, 0, 0, 0, 0);
            vTaskDelay(1);
            Joint_Motor_Right_2.Control(0, 0, 0, 0, 0);
            vTaskDelay(1);
            Wheel_Motor_Right.Control(0);
            vTaskDelay(1);
        }
        else {
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

            sys_state_l.d_x = LPF_d_x(Wheel_Motor_Left.Velocity * WHEEL_RADIUS - Wheel_Motor_Right.Velocity * WHEEL_RADIUS) * 0.5f;
            sys_state_r.d_x = sys_state_l.d_x;

            sys_state_l.x += (sys_state_l.d_x + sys_state_r.d_x) * dt * 0.5f;
            sys_state_r.x += (sys_state_l.d_x + sys_state_r.d_x) * dt * 0.5f;


            /*根据腿长L0获取LQR反馈增益K矩阵*/
            for (int i = 0;i < 12;i++)
            {
                LQR_K_L[i] = LQR_Get_K(&Poly_Coefficient[i][0], vmc_left.L0);
            }
            for (int i = 0;i < 12;i++)
            {
                LQR_K_R[i] = LQR_Get_K(&Poly_Coefficient[i][0], vmc_right.L0);
            }

            /*计算PID*/
            PID_Yaw.Cal(LPF_tar_yaw(Tar_Yaw) - INS.YawTotalAngle); // Yaw轴补偿
            PID_Roll.Cal(Tar_Roll - INS.Roll); // Roll轴补偿
            PID_Tp.Cal(vmc_right.theta - vmc_left.theta); // 双腿角度协调控制
            float temp_tar_l0 = LPF_tar_L0(Tar_L0);
            PID_L0_L.Cal(temp_tar_l0 - vmc_left.L0); // 左腿腿长控制
            PID_L0_R.Cal(temp_tar_l0 - vmc_right.L0); // 右腿腿长控制

            /*根据电机反馈力矩vmc逆解算机体支持力*/
            vmc_left.calc_fn(-Joint_Motor_Left_1.Torque, -Joint_Motor_Left_2.Torque);
            vmc_right.calc_fn(Joint_Motor_Right_1.Torque, Joint_Motor_Right_2.Torque);


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
            mySaturate(&sys_input_l.wheel_T, -4.0f, 4.0f);
            mySaturate(&sys_input_r.wheel_T, -4.0f, 4.0f);

            /*发送电机控制信号*/
            Joint_Motor_Left_1.Control(0, 0, 0, 0, -vmc_left.torque_set[0]);
            Joint_Motor_Right_1.Control(0, 0, 0, 0, vmc_right.torque_set[0]);
            vTaskDelay(1);
            Joint_Motor_Left_2.Control(0, 0, 0, 0, -vmc_left.torque_set[1]);
            Joint_Motor_Right_2.Control(0, 0, 0, 0, vmc_right.torque_set[1]);
            vTaskDelay(1);
            Wheel_Motor_Left.Control(LPF_wheelL_T(sys_input_l.wheel_T));
            Wheel_Motor_Right.Control(-LPF_wheelR_T(sys_input_r.wheel_T));
            vTaskDelay(1);

        }


    }

}

float Control::LQR_Get_K(float* coe, float len)
{
    return coe[0] * len * len * len + coe[1] * len * len + coe[2] * len + coe[3];
}

void mySaturate(float* in, float min, float max)
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

void Control_Task()
{
    control.Task();
}
