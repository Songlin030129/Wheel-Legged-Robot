#include "Debug_Task.h"
#include "BMMotor.h"
#include "VOFA_Debug.h"
#include "fdcan.h"
float torque_l, torque_r;
void Debug_Task()
{
    vofa_debug_init(&debug, &huart1);

    // bmmotor_group_init(&bmmotor_group_left, &hfdcan1);
    // bmmotor_group_add_motor(&bmmotor_group_left, &wheel_motor_left, 1);
    // bmmotor_set_mode(&bmmotor_group_left, &wheel_motor_left, BMMOTOR_MODE_CURRENT);

    // bmmotor_group_init(&bmmotor_group_right, &hfdcan2);
    // bmmotor_group_add_motor(&bmmotor_group_right, &wheel_motor_right, 1);
    // bmmotor_set_mode(&bmmotor_group_right, &wheel_motor_right, BMMOTOR_MODE_CURRENT);
    while (1) {
        vofa_debug_run(&debug);

        // bmmotor_set_torque(&wheel_motor_left, torque_l);
        // bmmotor_group_send_control(&bmmotor_group_left);

        // bmmotor_set_torque(&wheel_motor_right, -torque_r);
        // bmmotor_group_send_control(&bmmotor_group_right);

        vTaskDelay(10);
    }
}