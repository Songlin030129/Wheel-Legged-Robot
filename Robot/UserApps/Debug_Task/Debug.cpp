#include "Debug.h"
void Debug_Task()
{
    debug.Init(&huart1);
    debug.Add_ValueCommander("E", &control.Control_Enable);
    debug.Add_ValueCommander("H", &control.Tar_L0);
    while (1)
    {
        debug.Run_Debug();
        // printf("BTNA:%d, BTNB:%d, LH:%d, LV:%d, RH:%d, RV:%d, LT:%d, RT:%d\r\n",
        //     xbox.btnA, xbox.btnB, xbox.joyLHori, xbox.joyLVert, xbox.joyRHori, xbox.joyRVert, xbox.trigLT, xbox.trigRT);

        // printf("Joint_Motor1_Tor:%f, Joint_Motor2_Tor:%f, Wheel_Motor_Tor:%f\r\n",
        //     Joint_Motor_Left_1.Torque, Joint_Motor_Left_2.Torque, Wheel_Motor_Left.Torque);

        printf(":%d, theta:%f, d_theta:%f, x:%f, d_x:%f, phi:%f, d_phi:%f, T:%f, Tp:%f, L0:%f, tar_x:%f, tar_d_x:%f, tar_L0:%f\r\n",
            0,
            control.sys_state_r.theta, control.sys_state_r.d_theta,
            control.sys_state_r.x, control.sys_state_r.d_x,
            control.sys_state_r.phi, control.sys_state_r.d_phi,
            control.sys_input_r.wheel_T, control.sys_input_r.joint_Tp,
            control.vmc_left.L0,
            control.Tar_x, control.Tar_d_x,
            control.Tar_L0);

        // printf("WheelL_dx:%f, LPF_WheelL_dx:%f\r\n", control.LPF_tar_yaw.input, control.LPF_tar_yaw.output);

        // printf("left_L0:%f, right_L0:%f, tar_L0:%f, left_output:%f, right_output:%f\r\n",
        //     control.vmc_left.L0, control.vmc_right.L0, control.Tar_L0,
        //     control.PID_L0_L.output_value, control.PID_L0_R.output_value);

        // printf("Yaw:%f, Tar_Yaw:%f, Out:%f\r\n", INS.Yaw, control.LPF_tar_yaw.output, control.PID_Yaw.output_value);

        // printf("Pitch:%f, RawPitch:%f, Roll:%f, RawRoll:%f, Yaw:%f\r\n", INS.Pitch, BMI088.Accel[0] / BMI088.Accel[2], INS.Roll, BMI088.Accel[1] / BMI088.Accel[2], INS.Yaw);
        vTaskDelay(10);

    }

}