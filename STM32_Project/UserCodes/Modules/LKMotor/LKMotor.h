#ifndef __LKMOTOR_H__
#define __LKMOTOR_H__

#include "bsp_can.h"
#include "common_inc.h"
#include "fdcan.h"

typedef struct {
    FDCAN_HandleTypeDef* hcan;
    uint8_t id;
    int temperature;
    float torque;
    float velocity;
} LKMotor;

void lkmotor_init(LKMotor* _motor, FDCAN_HandleTypeDef* _hcan, uint8_t _id);
void lkmotor_enable(LKMotor* _motor);
void lkmotor_disable(LKMotor* _motor);
void lkmotor_stop(LKMotor* _motor);
void lkmotor_control(LKMotor* _motor, float _torque);
void lkmotor_recv_callback(LKMotor* _motor, FDCAN_HandleTypeDef* _hcan,
                           FDCAN_RxHeaderTypeDef rxheader, uint8_t* recvbuf);

extern LKMotor wheel_motor_left;
extern LKMotor wheel_motor_right;

#endif