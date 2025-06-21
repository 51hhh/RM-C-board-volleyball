/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "main.h"
#include <string.h>
#include <math.h>
#include <stdbool.h>

/* 无符号整型转浮点型 */
static float uint_to_float(uint16_t val, float min, float max, uint8_t bits)
{
    float offset = (max - min) / (1 << bits);
    return min + (float)val * offset;
}



extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
//解码电机信息
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
// 电机反馈状态结构体
typedef struct {
    bool enabled;
    bool feedback_received;
    uint32_t last_send_time;
    float last_pos;
    float last_vel;
} motor_feedback_state_t;

// 电机反馈状态数组(支持3个电机)
static motor_feedback_state_t motor_feedback_states[3] = {0};

/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机*/
static motor_measure_t motor_chassis[7];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_YAW_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
        case CAN_TRIGGER_MOTOR_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_measure(&motor_chassis[i], rx_data);
            break;
        }
        case 0x00: // 达妙电机反馈帧即master id
        {
            static motor_t dm_motor;
            dm_motor_fbdata(&dm_motor, rx_data);
            
            // 更新电机反馈状态
            uint8_t motor_id = dm_motor.para.id;
            if (motor_id >= 1 && motor_id <= 3) {
                uint8_t index = motor_id - 1;
                motor_feedback_states[index].feedback_received = true;
                motor_feedback_states[index].last_pos = dm_motor.para.pos;
                motor_feedback_states[index].last_vel = dm_motor.para.vel;
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

/* 达妙电机反馈数据解析 */
void dm_motor_fbdata(motor_t *motor, uint8_t *rx_data)
{
    motor->para.id = (rx_data[0])&0x0F;
    motor->para.state = (rx_data[0])>>4;
    motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
    motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
    motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
    motor->para.pos = uint_to_float(motor->para.p_int, -motor->tmp.PMAX, motor->tmp.PMAX, 16); // (-12.5,12.5)
    motor->para.vel = uint_to_float(motor->para.v_int, -motor->tmp.VMAX, motor->tmp.VMAX, 12); // (-45.0,45.0)
    motor->para.tor = uint_to_float(motor->para.t_int, -motor->tmp.TMAX, motor->tmp.TMAX, 12); // (-18.0,18.0)
    motor->para.Tmos = (float)(rx_data[6]);
    motor->para.Tcoil = (float)(rx_data[7]);
}



/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000]
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384]
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384]
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384]
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384]
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;

    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;

    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;

    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_chassis1(int16_t motor1, int16_t motor2)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;

    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void CAN_cmd_chassis2(int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;

    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;

    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;
    //can2
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}


/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}


/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}



// 达妙电机控制函数

/**
  * @brief          control motor position and velocity in position-velocity mode
  * @param[in]      can_id: motor CAN ID
  * @param[in]      pos: target position (float, little-endian)
  * @param[in]      vel: target velocity (float, little-endian)
  * @retval         none
  */
/**
  * @brief          位置速度模式控制电机
  * @param[in]      can_id: 电机CAN ID
  * @param[in]      pos: 目标位置(浮点数，小端序)
  * @param[in]      vel: 目标速度(浮点数，小端序)
  * @retval         none
  */
void CAN_cmd_motor_pos_vel_control(float pos, float vel)
{
    // 位置和速度变化阈值(避免因浮点精度问题频繁重发)
    const float POS_THRESHOLD = 0.001f;
    const float VEL_THRESHOLD = 0.001f;
    
    for (uint8_t i = 0; i < 3; i++) {
        // 检查是否需要发送(位置/速度变化或未收到反馈)
        bool need_send = !motor_feedback_states[i].feedback_received || 
                        (fabsf(pos - motor_feedback_states[i].last_pos) > POS_THRESHOLD) ||
                        (fabsf(vel - motor_feedback_states[i].last_vel) > VEL_THRESHOLD);
        
        // 检查发送间隔(最小100ms)
        bool time_ok = (HAL_GetTick() - motor_feedback_states[i].last_send_time) >= 100;
        
        if (need_send && time_ok) {
            uint8_t *pos_ptr = (uint8_t*)&pos;
            uint8_t *vel_ptr = (uint8_t*)&vel;

            // 设置CAN消息头(固定值)
            gimbal_tx_message.IDE = CAN_ID_STD;
            gimbal_tx_message.RTR = CAN_RTR_DATA;
            gimbal_tx_message.DLC = 0x08;
            gimbal_tx_message.StdId = 0x101 + i; // CAN ID从0x101到0x103
            
            // 填充位置数据(小端序)
            gimbal_can_send_data[0] = pos_ptr[0];
            gimbal_can_send_data[1] = pos_ptr[1];
            gimbal_can_send_data[2] = pos_ptr[2];
            gimbal_can_send_data[3] = pos_ptr[3];
            
            // 填充速度数据(小端序)
            gimbal_can_send_data[4] = vel_ptr[0];
            gimbal_can_send_data[5] = vel_ptr[1];
            gimbal_can_send_data[6] = vel_ptr[2];
            gimbal_can_send_data[7] = vel_ptr[3];
            
            // 检查三个邮箱状态，选择空闲邮箱发送
            HAL_StatusTypeDef send_status = HAL_ERROR;
            uint32_t mailbox;

            // 尝试三个邮箱
            for (mailbox = CAN_TX_MAILBOX0; mailbox <= CAN_TX_MAILBOX2; mailbox++) {
                if (HAL_CAN_IsTxMessagePending(&GIMBAL_CAN, mailbox) == HAL_OK) {
                    send_status = HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message,
                                                     gimbal_can_send_data, &mailbox);
                    if (send_status == HAL_OK) {
                        break; // 发送成功则退出循环
                    }
                }
            }
            

            
            // 发送成功后更新状态
            if (send_status == HAL_OK) {
                motor_feedback_states[i].last_send_time = HAL_GetTick();
                motor_feedback_states[i].feedback_received = false;
                motor_feedback_states[i].last_pos = pos;
                motor_feedback_states[i].last_vel = vel;
            }
        }
    }
}



/**
  * @brief          enable motor with specified CAN ID and mode offset
  * @param[in]      can_id: motor CAN ID
  * @param[in]      mode_offset: mode offset ID (0x00, 0x100, 0x200, 0x300)
  * @retval         none
  */
/**
  * @brief          使能指定CAN ID和模式偏移的电机
  * @param[in]      can_id: 电机CAN ID
  * @param[in]      mode_offset: 模式偏移ID (0x00, 0x100, 0x200, 0x300)
  * @retval         none
  */
/**
  * @brief          control motor enable/disable with specified CAN ID and mode offset
  * @param[in]      can_id: motor CAN ID
  * @param[in]      mode_offset: mode offset ID (0x00, 0x100, 0x200, 0x300)
  * @param[in]      enable: True to enable, False to disable
  * @retval         none
  */
/**
  * @brief          控制电机使能/失能
  * @param[in]      can_id: 电机CAN ID
  * @param[in]      mode_offset: 模式偏移ID (0x00, 0x100, 0x200, 0x300)
  * @param[in]      enable: True为使能，False为失能
  * @retval         none
  */
void CAN_cmd_motor_control(uint16_t can_id, uint16_t mode_offset, bool enable)
{
    uint32_t send_mail_box;
    uint8_t motor_index = can_id - 1; // 电机ID转换为数组索引(1->0, 2->1, 3->2)
    
    if (enable) {
        // 检查电机是否已经使能并收到反馈
        if (motor_feedback_states[motor_index].enabled && 
            motor_feedback_states[motor_index].feedback_received) {
            return; // 已经使能并收到反馈，不再发送
        }
        
        // 检查上次发送时间，避免频繁发送
        if (HAL_GetTick() - motor_feedback_states[motor_index].last_send_time < 100) {
            return; // 发送间隔太短，跳过
        }
        
        // 更新状态
        motor_feedback_states[motor_index].enabled = true;
        motor_feedback_states[motor_index].feedback_received = false;
        motor_feedback_states[motor_index].last_send_time = HAL_GetTick();
    } else {
        // 失能电机时重置状态
        motor_feedback_states[motor_index].enabled = false;
        motor_feedback_states[motor_index].feedback_received = false;
    }

    gimbal_tx_message.StdId = can_id + mode_offset;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    
    /* 填充前7字节为0xFF */
    memset(gimbal_can_send_data, 0xFF, 7);
    /* 根据enable参数设置第8字节 */
    gimbal_can_send_data[7] = enable ? 0xFC : 0xFD;
    
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}


