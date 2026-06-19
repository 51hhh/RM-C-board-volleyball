/**
  ******************************************************************************
  * @file    host_comm.h
  * @brief   上位机 UART 通信 (USART1, 115200)
  *
  *   发送: DMA + 软件队列 (uart_queue_send / process_uart_queue)
  *   接收: 单字节中断, 解析 "{y},{x}\n" 协议 -> uart_y / uart_x
  ******************************************************************************
  */
#ifndef HOST_COMM_H
#define HOST_COMM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 协议解析结果: 上位机下发的坐标(被底盘控制读取) */
extern volatile int32_t uart_x;   // 前后
extern volatile int32_t uart_y;   // 左右

/* 初始化: 启动 USART1 单字节接收中断 */
void host_comm_init(void);

/* 将一段数据加入发送队列(非阻塞) */
void uart_queue_send(const char *data, uint16_t length);

/* 处理发送队列: DMA 空闲时发出队首一条(在主循环后台调用) */
void process_uart_queue(void);

#ifdef __cplusplus
}
#endif

#endif /* HOST_COMM_H */
