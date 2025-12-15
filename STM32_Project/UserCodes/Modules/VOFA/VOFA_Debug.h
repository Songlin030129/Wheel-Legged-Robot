#ifndef __VOFA_DEBUG_H
#define __VOFA_DEBUG_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "common_inc.h"

/* ================= 配置区 ================= */

#define DEBUG_RX_BUFFER_SIZE_MAX 100
#define DEBUG_COMMAND_SIZE_MAX 20
#define DEBUG_VALUE_CMD_MAX 32
#define DEBUG_FUNC_CMD_MAX 32

/* ================= 数据结构 ================= */

typedef struct {
    char command[DEBUG_COMMAND_SIZE_MAX];
    float *value1;
    float *value2;  // 可为 NULL
} VOFA_ValueCommander;

typedef struct {
    char command[DEBUG_COMMAND_SIZE_MAX];
    void (*func1)(float);
    void (*func2)(float, float);
    uint8_t param_num;  // 1 或 2
} VOFA_FunctionCommander;

typedef struct {
    UART_HandleTypeDef *huart;

    volatile int16_t rx_flag;  // 0:无命令  1:1参数  2:2参数
    float rx_value1;
    float rx_value2;
    char rx_command[DEBUG_COMMAND_SIZE_MAX];

    uint8_t rx_buffer[DEBUG_RX_BUFFER_SIZE_MAX];

    VOFA_ValueCommander value_cmds[DEBUG_VALUE_CMD_MAX];
    uint16_t value_cmd_count;

    VOFA_FunctionCommander func_cmds[DEBUG_FUNC_CMD_MAX];
    uint16_t func_cmd_count;
} VOFA_Debug;
extern VOFA_Debug debug;

/* ================= 接口函数 ================= */

#ifdef __cplusplus
extern "C" {
#endif

void vofa_debug_init(VOFA_Debug *_dbg, UART_HandleTypeDef *_huart);

void vofa_debug_uart_receive_idle_dma_callback(VOFA_Debug *_dbg, UART_HandleTypeDef *_huart, uint16_t _size);

void vofa_debug_uart_err_callback(VOFA_Debug *_dbg, UART_HandleTypeDef *_huart);

int vofa_debug_add_value1(VOFA_Debug *_dbg, const char *_command, float *_value);

int vofa_debug_add_value2(VOFA_Debug *_dbg, const char *_command, float *_value1, float *_value2);

int vofa_debug_add_func1(VOFA_Debug *_dbg, const char *_command, void (*_fn)(float));

int vofa_debug_add_func2(VOFA_Debug *_dbg, const char *_command, void (*_fn)(float, float));

void vofa_debug_run(VOFA_Debug *_dbg);

#ifdef __cplusplus
}
#endif

#endif /* __VOFA_DEBUG_H */
