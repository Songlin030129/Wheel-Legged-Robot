#ifndef __VOFA_DEBUG_H
#define __VOFA_DEBUG_H

#include "common_inc.h"
#include <string>
#include <vector>
#define DEBUG_RX_BUFFER_SIZE_MAX 100
#define DEBUG_COMMAND_SIZE_MAX 20

struct ValueCommander
{
    std::string command;
    float* value1;
    float* value2;
}; // 命令结构体
struct FunctionCommander
{
    std::string command;
    void (*function)(float);
    void (*function2)(float, float);
    uint8_t param_num;
}; // 命令结构体

class VOFA_Debug
{
public:
    /**
     * @brief VOFA调试串口初始化
     *
     * @param huart
     */
    void Init(UART_HandleTypeDef* huart);
    /**
     * @brief VOFA调试串口空闲中断回调函数（DMA）
     *
     * @param huart 串口句柄
     * @param Size 接收数据长度
     */
    void UartReceive_IDLE_DMA_Callback(UART_HandleTypeDef* huart, uint16_t Size);
    /**
     * @brief 调试命令执行
     *
     */
    void Run_Debug();
    /**
     * @brief 添加调试命令
     *
     * @param command 命令字符串
     * @param value 值指针
     */
    void Add_ValueCommander(std::string command, float* value);
    /**
     * @brief 添加调试命令
     *
     * @param command 命令字符串
     * @param value1 值指针1
     * @param value2 值指针2
     */
    void Add_ValueCommander(std::string command, float* value1, float* value2);
    /**
     * @brief 添加调试命令
     *
     * @param command 命令字符串
     * @param function 函数指针
     */
    void Add_FunctionCommander(std::string command, void (*function)(float));
    /**
     * @brief 添加调试命令
     *
     * @param command 命令字符串
     * @param function 函数指针
     */
    void Add_FunctionCommander(std::string command, void (*function)(float, float));

private:
    UART_HandleTypeDef* huart;
    std::string RxCommand;
    float RxValue1, RxValue2;
    int16_t RxFlag;
    std::vector<ValueCommander> ValueCommanderList;
    std::vector<FunctionCommander> FunctionCommanderList;
    uint8_t RxBuffer[DEBUG_RX_BUFFER_SIZE_MAX];
};
extern VOFA_Debug debug;


#endif // __VOFA_DEBUG_H