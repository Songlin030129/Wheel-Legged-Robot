#include "Remote_Control.h"
#include "XboxController.h"
static float v_set;
static void slope_following(float* _target, float* _set, float _acc)
{
    if (*_target > *_set) {
        *_set = *_set + _acc;
        if (*_set >= *_target)
            *_set = *_target;
    } else if (*_target < *_set) {
        *_set = *_set - _acc;
        if (*_set <= *_target)
            *_set = *_target;
    }
}
static void saturate(float* _in, float _min, float _max)
{
    if (*_in < _min) {
        *_in = _min;
    } else if (*_in > _max) {
        *_in = _max;
    }
}

void Remote_Control_Task(void)
{
    xbox_init(&xbox, &huart7);
    while (1) {
        xbox_keys_handler(&xbox);
        static float max_vel = 2.0f;
        if (xbox.connected) {
            if (control.Control_Enable) {
                float v_x = (float)(xbox.trigRT - xbox.trigLT) * max_vel / 1024;
                slope_following(&v_x, &v_set, 0.01f);
                control.Tar_d_x = v_set;
                control.Tar_x += control.Tar_d_x * 0.01f;

                control.Tar_Yaw += -(float)xbox.joyLHori / 30000 * 0.01f;

                control.Tar_L0 += -(float)xbox.joyRVert / 200000 * 0.01f;
                saturate(&control.Tar_L0, 0.10f, 0.30f);
            }
        } else {
            control.Control_Enable = 0;
        }
        vTaskDelay(10);
    }
}

void KEY_KeyClickCallback(KEY* _key)
{
    if (_key == &xbox.btnA) {
        if (xbox.connected)
            control.Control_Enable = 1;
        printf("clickA\r\n");
    }
    if (_key == &xbox.btnB) {
        if (xbox.connected)
            control.Control_Enable = 0;
        printf("clickB\r\n");
    }
    if (_key == &xbox.btnX) {
        if (xbox.connected)
            control.Tar_Roll = 0;
        printf("clickX\r\n");
    }
    if (_key == &xbox.btnY) {
        if (xbox.connected)
            control.Tar_Roll = 0;
        printf("clickY\r\n");
    }
    if (_key == &xbox.btnStart) {
        if (xbox.connected)
            if (control.Control_Enable == 0)
                NVIC_SystemReset();
        printf("clickStart\r\n");
    }
}
void KEY_MultipleClickCallback(KEY* _key)
{
    if (_key == &xbox.btnA) {
        printf("mult click\r\n");
    }
}
void KEY_LongHoldCallback(KEY* _key)
{
    if (_key == &xbox.btnA) {
        printf("long hold\r\n");
    }
}
void KEY_HoldTriggerCallback(KEY* _key)
{
    if (_key == &xbox.btnA) {
        printf("hold trigger\r\n");
    }
}
