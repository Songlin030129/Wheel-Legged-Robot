#include "Remote_Control.h"
static float v_set;
void slope_following(float* target, float* set, float acc)
{
    if (*target > *set)
    {
        *set = *set + acc;
        if (*set >= *target)
            *set = *target;
    }
    else if (*target < *set)
    {
        *set = *set - acc;
        if (*set <= *target)
            *set = *target;
    }

}
void Saturate(float* in, float min, float max)
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


void Remote_Control_Task(void)
{
    MX_UART7_Init();
    xbox.Init(&huart7);
    while (1)
    {
        xbox.KeysHandler();
        static float max_vel = 2.0f;
        if (xbox.connected) {
            if (control.Control_Enable) {
                float v_x = (float)(xbox.trigRT - xbox.trigLT) * max_vel / 1024;
                slope_following(&v_x, &v_set, 0.01f);
                control.Tar_d_x = v_set;
                control.Tar_x += control.Tar_d_x * 0.01f;

                control.Tar_Yaw += -(float)xbox.joyLHori / 30000 * 0.01f;

                control.Tar_L0 += -(float)xbox.joyRVert / 200000 * 0.01f;
                Saturate(&control.Tar_L0, 0.10f, 0.30f);

                // control.Tar_Roll += (float)xbox.joyRHori / 50000 * 0.01f;
                // Saturate(&control.Tar_Roll, -0.40f, 0.40f);

            }
        }
        else {
            control.Control_Enable = 0;
        }
        vTaskDelay(10);
    }
}

void KEY_KeyClickCallback(KEY* key)
{
    if (key == &xbox.btnA) {
        if (xbox.connected)
            control.Control_Enable = 1;
        printf("clickA\r\n");
    }
    if (key == &xbox.btnB) {
        if (xbox.connected)
            control.Control_Enable = 0;
        printf("clickB\r\n");
    }
    if (key == &xbox.btnX) {
        if (xbox.connected)
            control.Tar_Roll = 0;
        printf("clickX\r\n");
    }
    if (key == &xbox.btnY) {
        if (xbox.connected)
            control.Tar_Roll = 0;
        printf("clickY\r\n");
    }
    if (key == &xbox.btnStart) {
        if (xbox.connected)
            if (control.Control_Enable == 0)
                NVIC_SystemReset();
        printf("clickStart\r\n");
    }
}
void KEY_MultipleClickCallback(KEY* key)
{
    if (key == &xbox.btnA) {
        printf("mult click\r\n");
    }
}
void KEY_LongHoldCallback(KEY* key)
{
    if (key == &xbox.btnA) {
        printf("long hold\r\n");
    }

}
void KEY_HoldTriggerCallback(KEY* key)
{
    if (key == &xbox.btnA) {
        printf("hold trigger\r\n");
    }

}
