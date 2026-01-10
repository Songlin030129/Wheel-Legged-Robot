#include "VOFA_Debug.h"
VOFA_Debug debug;
/* ================= 内部工具函数 ================= */

static void copy_str(char *_dst, const char *_src, uint16_t _max_len)
{
    if (!_dst || _max_len == 0) {
        return;
    }
    strncpy(_dst, _src, _max_len - 1);
    _dst[_max_len - 1] = '\0';
}

/* ================= 初始化 ================= */

void vofa_debug_init(VOFA_Debug *_dbg, UART_HandleTypeDef *_huart)
{
    if (!_dbg) {
        return;
    }

    memset(_dbg, 0, sizeof(VOFA_Debug));
    _dbg->huart = _huart;

    HAL_UARTEx_ReceiveToIdle_DMA(_huart, _dbg->rx_buffer, DEBUG_RX_BUFFER_SIZE_MAX);
}

/* ================= 串口 Idle DMA 回调 ================= */

void vofa_debug_uart_receive_idle_dma_callback(VOFA_Debug *_dbg, UART_HandleTypeDef *_huart, uint16_t _size)
{
    if (!_dbg || _huart != _dbg->huart) {
        return;
    }
    __HAL_UART_CLEAR_OREFLAG(_huart);
    __HAL_UART_CLEAR_FEFLAG(_huart);
    __HAL_UART_CLEAR_PEFLAG(_huart);
    __HAL_UART_CLEAR_NEFLAG(_huart);
    char rx_data[DEBUG_RX_BUFFER_SIZE_MAX + 1];
    uint16_t n = (_size < DEBUG_RX_BUFFER_SIZE_MAX) ? _size : DEBUG_RX_BUFFER_SIZE_MAX;

    memcpy(rx_data, _dbg->rx_buffer, n);
    rx_data[n] = '\0';  // 确保 sscanf 安全

    _dbg->rx_value1 = 0.0f;
    _dbg->rx_value2 = 0.0f;
    _dbg->rx_command[0] = '\0';

    char temp_cmd[DEBUG_COMMAND_SIZE_MAX] = {0};

    int ret = sscanf(rx_data, "%19[^:]:%f,%f", temp_cmd, &_dbg->rx_value1, &_dbg->rx_value2);

    _dbg->rx_flag = ret - 1;  // 1 或 2
    copy_str(_dbg->rx_command, temp_cmd, DEBUG_COMMAND_SIZE_MAX);

    HAL_UARTEx_ReceiveToIdle_DMA(_huart, _dbg->rx_buffer, DEBUG_RX_BUFFER_SIZE_MAX);
}

void vofa_debug_uart_err_callback(VOFA_Debug *_dbg, UART_HandleTypeDef *_huart)
{
    if (!_dbg || _huart != _dbg->huart) {
        return;
    }
    // 停止当前的DMA传输
    HAL_UART_DMAStop(_huart);

    // 清除所有错误标志位
    __HAL_UART_CLEAR_OREFLAG(_huart);
    __HAL_UART_CLEAR_FEFLAG(_huart);
    __HAL_UART_CLEAR_PEFLAG(_huart);
    __HAL_UART_CLEAR_NEFLAG(_huart);
    __HAL_UART_CLEAR_IDLEFLAG(_huart);

    // 清空接收FIFO
    while (__HAL_UART_GET_FLAG(_huart, UART_FLAG_RXNE)) {
        volatile uint8_t temp = _huart->Instance->RDR;
        (void)temp;
    }
    __HAL_UART_FLUSH_DRREGISTER(_huart);
    // 清空缓冲区
    memset(_dbg->rx_buffer, 0, DEBUG_RX_BUFFER_SIZE_MAX);

    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(_huart, (uint8_t *)_dbg->rx_buffer, DEBUG_RX_BUFFER_SIZE_MAX);
}
/* ================= 命令注册 ================= */

int vofa_debug_add_value1(VOFA_Debug *_dbg, const char *_command, float *_value)
{
    if (!_dbg || !_command || !_value) {
        return -1;
    }
    if (_dbg->value_cmd_count >= DEBUG_VALUE_CMD_MAX) {
        return -2;
    }

    VOFA_ValueCommander *c = &_dbg->value_cmds[_dbg->value_cmd_count++];

    copy_str(c->command, _command, DEBUG_COMMAND_SIZE_MAX);
    c->value1 = _value;
    c->value2 = NULL;
    return 0;
}

int vofa_debug_add_value2(VOFA_Debug *_dbg, const char *_command, float *_value1, float *_value2)
{
    if (!_dbg || !_command || !_value1 || !_value2) {
        return -1;
    }
    if (_dbg->value_cmd_count >= DEBUG_VALUE_CMD_MAX) {
        return -2;
    }

    VOFA_ValueCommander *c = &_dbg->value_cmds[_dbg->value_cmd_count++];

    copy_str(c->command, _command, DEBUG_COMMAND_SIZE_MAX);
    c->value1 = _value1;
    c->value2 = _value2;
    return 0;
}

int vofa_debug_add_func1(VOFA_Debug *_dbg, const char *_command, void (*_fn)(float))
{
    if (!_dbg || !_command || !_fn) {
        return -1;
    }
    if (_dbg->func_cmd_count >= DEBUG_FUNC_CMD_MAX) {
        return -2;
    }

    VOFA_FunctionCommander *c = &_dbg->func_cmds[_dbg->func_cmd_count++];

    copy_str(c->command, _command, DEBUG_COMMAND_SIZE_MAX);
    c->func1 = _fn;
    c->func2 = NULL;
    c->param_num = 1;
    return 0;
}

int vofa_debug_add_func2(VOFA_Debug *_dbg, const char *_command, void (*_fn)(float, float))
{
    if (!_dbg || !_command || !_fn) {
        return -1;
    }
    if (_dbg->func_cmd_count >= DEBUG_FUNC_CMD_MAX) {
        return -2;
    }

    VOFA_FunctionCommander *c = &_dbg->func_cmds[_dbg->func_cmd_count++];

    copy_str(c->command, _command, DEBUG_COMMAND_SIZE_MAX);
    c->func1 = NULL;
    c->func2 = _fn;
    c->param_num = 2;
    return 0;
}

/* ================= 主循环执行 ================= */

void vofa_debug_run(VOFA_Debug *_dbg)
{
    if (!_dbg || _dbg->rx_flag <= 0) {
        return;
    }

    uint8_t found = 0;

    /* ---------- Value Commander ---------- */
    for (uint16_t i = 0; i < _dbg->value_cmd_count; i++) {
        VOFA_ValueCommander *c = &_dbg->value_cmds[i];
        if (strcmp(_dbg->rx_command, c->command) == 0) {
            found = 1;
            if (_dbg->rx_flag == 1) {
                *(c->value1) = _dbg->rx_value1;
                printf("%s:%f\r\n", c->command, *(c->value1));
            } else if (_dbg->rx_flag == 2 && c->value2) {
                *(c->value1) = _dbg->rx_value1;
                *(c->value2) = _dbg->rx_value2;
                printf("%s:%f,%f\r\n", c->command, *(c->value1), *(c->value2));
            }
        }
    }

    /* ---------- Function Commander ---------- */
    for (uint16_t i = 0; i < _dbg->func_cmd_count; i++) {
        VOFA_FunctionCommander *c = &_dbg->func_cmds[i];
        if (strcmp(_dbg->rx_command, c->command) == 0) {
            found = 1;
            if (_dbg->rx_flag == 1 && c->param_num == 1 && c->func1) {
                c->func1(_dbg->rx_value1);
                printf("%s:%f\r\n", c->command, _dbg->rx_value1);
            } else if (_dbg->rx_flag == 2 && c->param_num == 2 && c->func2) {
                c->func2(_dbg->rx_value1, _dbg->rx_value2);
                printf("%s:%f,%f\r\n", c->command, _dbg->rx_value1, _dbg->rx_value2);
            }
        }
    }

    if (!found) {
        printf("Error: Command not found\r\n");
    }

    _dbg->rx_flag = 0;  // 清除标志
}
