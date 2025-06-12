#include "VOFA_Debug.h"
VOFA_Debug debug;

/**
 * @brief VOFA调试串口初始化
 *
 * @param huart
 */
void VOFA_Debug::Init(UART_HandleTypeDef* huart)
{
    this->huart = huart;
    HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*)this->RxBuffer, DEBUG_RX_BUFFER_SIZE_MAX);
}
/**
 * @brief VOFA调试串口空闲中断回调函数（DMA）
 *
 * @param huart 串口句柄
 * @param Size 接收数据长度
 */
void VOFA_Debug::UartReceive_IDLE_DMA_Callback(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (huart == this->huart)
    {
        char RxData[DEBUG_RX_BUFFER_SIZE_MAX];
        for (int i = 0; i < Size; i++)
        {
            RxData[i] = RxBuffer[i];
        }
        RxCommand.clear();
        RxValue1 = 0;
        RxValue2 = 0;
        uint8_t temp[DEBUG_COMMAND_SIZE_MAX];
        RxFlag = sscanf(RxData, "%[^:]:%f,%f", temp, &RxValue1, &RxValue2) - 1;
        RxCommand = (char*)temp;
        if (RxFlag <= 0)
            printf("Error: sscanf failed\n");

        HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*)this->RxBuffer, DEBUG_RX_BUFFER_SIZE_MAX);
    }
}
/**
 * @brief 添加调试命令
 *
 * @param command 命令字符串
 * @param value 值指针
 */
void VOFA_Debug::Add_ValueCommander(std::string command, float* value)
{
    ValueCommander temp;
    temp.command = command;
    temp.value1 = value;
    temp.value2 = NULL;
    ValueCommanderList.push_back(temp);
}
/**
 * @brief 添加调试命令
 *
 * @param command 命令字符串
 * @param value1 值指针1
 * @param value2 值指针2
 */
void VOFA_Debug::Add_ValueCommander(std::string command, float* value1, float* value2)
{
    ValueCommander temp;
    temp.command = command;
    temp.value1 = value1;
    temp.value2 = value2;
    ValueCommanderList.push_back(temp);
}

/**
 * @brief 添加调试命令
 *
 * @param command 命令字符串
 * @param function 函数指针
 */
void VOFA_Debug::Add_FunctionCommander(std::string command, void (*function)(float))
{
    FunctionCommander temp;
    temp.command = command;
    temp.function = function;
    temp.function2 = NULL;
    temp.param_num = 1;
    FunctionCommanderList.push_back(temp);
}
/**
 * @brief 添加调试命令
 *
 * @param command 命令字符串
 * @param function 函数指针
 */
void VOFA_Debug::Add_FunctionCommander(std::string command, void (*function)(float, float))
{
    FunctionCommander temp;
    temp.command = command;
    temp.function = NULL;
    temp.function2 = function;
    temp.param_num = 2;
    FunctionCommanderList.push_back(temp);
}
/**
 * @brief 调试命令执行
 *
 */
void VOFA_Debug::Run_Debug()
{
    if (RxFlag > 0)
    {
        uint8_t Command_Found = 0;
        for (int i = 0; i < ValueCommanderList.size(); i++)
        {
            if (RxCommand == ValueCommanderList[i].command)
            {
                Command_Found = 1;
                if (RxFlag == 1)
                {
                    *ValueCommanderList[i].value1 = RxValue1;
                    printf("%s:%f\n", ValueCommanderList[i].command.c_str(), *ValueCommanderList[i].value1);
                }
                else if (RxFlag == 2)
                {
                    *ValueCommanderList[i].value1 = RxValue1;
                    *ValueCommanderList[i].value2 = RxValue2;
                    printf("%s:%f,%f\n", ValueCommanderList[i].command.c_str(), *ValueCommanderList[i].value1, *ValueCommanderList[i].value2);
                }
                else
                {
                    printf("Error: RxFlag error\n");
                }
            }
        }
        for (int i = 0; i < FunctionCommanderList.size(); i++)
        {
            if (RxCommand == FunctionCommanderList[i].command)
            {
                Command_Found = 1;
                if (RxFlag == 1)
                {
                    if (FunctionCommanderList[i].param_num != 1)
                    {
                        printf("Error: param_num error\n");
                    }
                    else
                    {
                        FunctionCommanderList[i].function(RxValue1);
                        printf("%s:%f\n", FunctionCommanderList[i].command.c_str(), RxValue1);
                    }
                }
                else if (RxFlag == 2)
                {
                    if (FunctionCommanderList[i].param_num != 2)
                    {
                        printf("Error: param_num error\n");
                    }
                    else
                    {
                        FunctionCommanderList[i].function2(RxValue1, RxValue2);
                        printf("%s:%f,%f\n", FunctionCommanderList[i].command.c_str(), RxValue1, RxValue2);
                    }
                }
                else
                {
                    printf("Error: RxFlag error\n");
                }
            }
        }
        if (Command_Found == 0)
        {
            printf("Error: Command not found\n");
        }
        RxFlag = 0;
    }
}