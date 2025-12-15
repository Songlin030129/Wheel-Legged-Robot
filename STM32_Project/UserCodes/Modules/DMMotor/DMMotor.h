#ifndef __DM4310_DRV_H__
#define __DM4310_DRV_H__

#include "bsp_can.h"
#include "common_inc.h"
#include "fdcan.h"

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

typedef enum {
    CONTROL_MODE_MIT = 0,
    CONTROL_MODE_POS_VEL = 0x100,
    CONTROL_MODE_VEL = 0x200,
} DMMotor_Control_Mode;

typedef struct {
    uint8_t motor_id;
    uint8_t master_id;
    DMMotor_Control_Mode mode;
    FDCAN_HandleTypeDef* hcan;
    float position;
    float velocity;
    float torque;
} DMMotor;

/* 初始化 */
void dmmotor_init(DMMotor* _motor, FDCAN_HandleTypeDef* _hcan, uint8_t _slave_id,
                  uint8_t _master_id, DMMotor_Control_Mode _mode);

/* 接收回调 */
void dmmotor_recv_callback(DMMotor* _motor, FDCAN_HandleTypeDef* _hcan,
                           FDCAN_RxHeaderTypeDef rxheader, uint8_t* recvbuf);

/* 控制与命令 */
void dmmotor_enable(DMMotor* _motor);
void dmmotor_disable(DMMotor* _motor);
void dmmotor_save_zero_point(DMMotor* _motor);
void dmmotor_clear_err(DMMotor* _motor);

void dmmotor_control_mit(DMMotor* _motor, float _pos, float _vel, float _KP, float _KD,
                         float _torq);
void dmmotor_control_posvel(DMMotor* _motor, float _vel, float _pos);
void dmmotor_control_vel(DMMotor* _motor, float _vel);

/* 全局实例 */
extern DMMotor joint_motor_left_1;
extern DMMotor joint_motor_left_2;
extern DMMotor joint_motor_right_1;
extern DMMotor joint_motor_right_2;

#endif