#ifndef __BMMOTOR_H__
#define __BMMOTOR_H__
#include "common_inc.h"

typedef enum {
    BMMOTOR_MODE_VOLTAGE = 0x00,
    BMMOTOR_MODE_CURRENT = 0x01,
    BMMOTOR_MODE_VELOCITY = 0x02,
    BMMOTOR_MODE_DISABLE = 0x09,
    BMMOTOR_MODE_ENABLE = 0x0A,
} BMMotor_Mode;

typedef struct {
    FDCAN_HandleTypeDef* hcan;
    uint8_t id;
    int16_t voltage_set;
    float velocity_set;
    float torque_set;
    BMMotor_Mode mode_set;

    BMMotor_Mode mode;
    uint8_t err_code;
    float torque;
    float velocity;
    float position;

} BMMotor;

typedef struct {
    FDCAN_HandleTypeDef* hcan;
    uint8_t motor_cnt;
    BMMotor* motors[8];
} BMMotor_Group;
extern BMMotor_Group bmmotor_group_left;
extern BMMotor_Group bmmotor_group_right;
extern BMMotor wheel_motor_left;
extern BMMotor wheel_motor_right;

void bmmotor_group_init(BMMotor_Group* _motor_group, FDCAN_HandleTypeDef* _hcan);
void bmmotor_group_add_motor(BMMotor_Group* _motor_group, BMMotor* _motor, uint8_t _id);
void bmmotor_group_recv_callback(BMMotor_Group* _motor_group, FDCAN_HandleTypeDef* _hcan, FDCAN_RxHeaderTypeDef rxheader, uint8_t* recvbuf);
void bmmotor_group_send_control(BMMotor_Group* _motor_group);

void bmmotor_set_torque(BMMotor* _motor, float _torque);
void bmmotor_set_velocity(BMMotor* _motor, float _velocity);
void bmmotor_set_voltage(BMMotor* _motor, int16_t _set);

void bmmotor_set_mode(BMMotor_Group* _motor_group, BMMotor* _motor, BMMotor_Mode _mode);
void bmmotor_set_id(FDCAN_HandleTypeDef* _hcan, uint8_t _id);
#endif