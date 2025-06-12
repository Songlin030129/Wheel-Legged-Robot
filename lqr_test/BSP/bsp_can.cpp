#include "bsp_can.h"
#include "fdcan.h"
#include "string.h"
FDCAN_RxHeaderTypeDef RxHeader1;
uint8_t g_Can1RxData[64];

FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t g_Can2RxData[64];

void FDCAN1_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    /* Configure Rx filter */
    sFilterConfig.IdType = FDCAN_STANDARD_ID; // 扩展ID不接收
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00000000; //
    sFilterConfig.FilterID2 = 0x00000000; //
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* 全局过滤设置 */
    /* 接收到消息ID与标准ID过滤不匹配，不接受 */
    /* 接收到消息ID与扩展ID过滤不匹配，不接受 */
    /* 过滤标准ID远程帧 */
    /* 过滤扩展ID远程帧 */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    /* 开启RX FIFO0的新数据中断 */
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    /* Start the FDCAN module */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

void FDCAN2_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    /* Configure Rx filter */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 1;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x00000000;
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure global filter:
       Filter all remote frames with STD and EXT ID
       Reject non matching frames with STD ID and EXT ID */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    /* Activate Rx FIFO 0 new message notification on both FDCAN instances */
    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
        Error_Handler();
    }
}

uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier = id; // CAN ID
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    if (len <= 8)
    {
        TxHeader.DataLength = len; // 发送长度：8byte
    }
    else if (len == 12)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_12;
    }
    else if (len == 16)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_16;
    }
    else if (len == 20)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_20;
    }
    else if (len == 24)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_24;
    }
    else if (len == 48)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_48;
    }
    else if (len == 64)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_64;
    }

    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // 比特率切换关闭，不适用于经典CAN
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;  // CANFD
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0; // 消息标记

    // 发送CAN指令
    if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data) != HAL_OK)
    {
        // 发送失败处理
        Error_Handler();
    }
    return 0;
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if (hfdcan->Instance == FDCAN1)
        {
            // /* Retrieve Rx messages from RX FIFO0 */
            memset(g_Can1RxData, 0, sizeof(g_Can1RxData)); // 接收前先清空数组
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData);

            Joint_Motor_Left_1.Recv_Callback(hfdcan, RxHeader1, g_Can1RxData);
            Joint_Motor_Left_2.Recv_Callback(hfdcan, RxHeader1, g_Can1RxData);
            Wheel_Motor_Left.Recv_Callback(hfdcan, RxHeader1, g_Can1RxData);
        }
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs)
{
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
    {
        if (hfdcan->Instance == FDCAN2)
        {
            /* Retrieve Rx messages from RX FIFO0 */
            memset(g_Can2RxData, 0, sizeof(g_Can2RxData));
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader2, g_Can2RxData);

            Joint_Motor_Right_1.Recv_Callback(hfdcan, RxHeader2, g_Can2RxData);
            Joint_Motor_Right_2.Recv_Callback(hfdcan, RxHeader2, g_Can2RxData);
            Wheel_Motor_Right.Recv_Callback(hfdcan, RxHeader2, g_Can2RxData);
        }
    }
}
