#include "bsp_can.h"
#include "can.h"

#define CAN_6020_M1_ID 0x205



moto_info_t motor_yaw_info;
uint16_t can_cnt;



void can_filter_init(void)//筛选器配置
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;                //开启该过滤器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;       //筛选器模式是ID掩码模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;      //筛选器位宽
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;                           //选择过滤器0
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;      //把接收到的报文放入到FIFO0中
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);//把过滤器配置配置给can1外设
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)  //can接收中断回调函数
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t             rx_data[8];
    if(hcan->Instance == CAN1)   // 检查 CAN 句柄是否指向 CAN1 外设。
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //CAN数据接收函数
        switch(rx_header.StdId)
        {
            case CAN_6020_M1_ID:
            {
                motor_yaw_info.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);// 从接收到的数据中提取转子角度信息
                motor_yaw_info.rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);// 从接收到的数据中提取转子速度信息
                motor_yaw_info.torque_current = ((rx_data[4] << 8) | rx_data[5]);// 从接收到的数据中提取转矩电流信息
                motor_yaw_info.temp           =   rx_data[6];                   // 从接收到的数据中提取温度信息
                break;
            }
        }
    }
}

void set_GM6020_motor_voltage(CAN_HandleTypeDef* hcan,int16_t v1)
//电机电压控制CAN发送函数(CAN 句柄，指向 CAN 外设的结构体,要发送给电机的电压值，有符号 16 位整数)
{
    CAN_TxHeaderTypeDef tx_header;          //CAN 发送帧的头部结构体
    uint8_t             tx_data[8] = {0};   //8 字节的数组

    tx_header.StdId = 0x1ff;
    // 0x1ff 是 GM6020 电机控制指令的 CAN ID。
    //  通常，如果电机 ID 设置为 1，则控制指令的 CAN ID 为 0x200 - 1 = 0x1FF。
    tx_header.IDE   = CAN_ID_STD;
    // 设置 ID 扩展位 (Identifier Extension) 为 CAN_ID_STD，表示使用标准 ID。
    tx_header.RTR   = CAN_RTR_DATA;
    // 设置远程传输请求位 (Remote Transmission Request) 为 CAN_RTR_DATA，表示发送的是数据帧。
    tx_header.DLC   = 8;
    // 设置数据长度代码 (Data Length Code) 为 8，表示发送的数据长度为 8 字节。


    tx_data[0] = (v1>>8)&0xff;
    tx_data[1] =    (v1)&0xff;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
    // 调用 HAL 库函数 HAL_CAN_AddTxMessage 发送 CAN 消息。
}