/**
  ******************************************************************************
  * @file    host_comm.c
  * @brief   上位机 UART 通信实现 (USART1)
  ******************************************************************************
  */
#include "host_comm.h"
#include "main.h"
#include "usart.h"   /* huart1 */
#include <string.h>
#include <stdio.h>

/* 协议解析结果(供底盘控制读取) */
volatile int32_t uart_x = 0;   // 前后
volatile int32_t uart_y = 0;   // 左右

/* ===== 发送队列 ===== */
#define UART_QUEUE_SIZE 32
typedef struct {
    char     buffer[128];
    uint16_t length;
} uart_queue_item_t;

static uart_queue_item_t uart_queue[UART_QUEUE_SIZE];
static uint16_t uart_queue_head = 0;
static uint16_t uart_queue_tail = 0;
static volatile uint8_t dma_tx_complete = 1;  /* DMA 传输完成标志(TX 回调置位) */

/* ===== 接收 ===== */
static uint8_t  rx_byte;            /* 单字节接收缓冲 */
static uint8_t  line_buffer[128];   /* 行缓冲 */
static uint16_t line_pos = 0;

void host_comm_init(void)
{
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

void uart_queue_send(const char *data, uint16_t length)
{
    if (length > sizeof(uart_queue[0].buffer)) length = sizeof(uart_queue[0].buffer);

    /* 多生产者入队：主循环遥测 + USART1 RX 中断 + EXTI 中断都会调用本函数。
     * 用临界区保护"判满 + 写槽 + 推进 head"三步，避免不同上下文抢同一槽位/丢消息。
     * (与 scheduler.c:schedule_after 同一范式；消费端 process_uart_queue 为单一主循环上下文) */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();                       /* 进入临界区 */
    if ((uart_queue_head + 1) % UART_QUEUE_SIZE != uart_queue_tail) {
        memcpy(uart_queue[uart_queue_head].buffer, data, length);
        uart_queue[uart_queue_head].length = length;
        uart_queue_head = (uart_queue_head + 1) % UART_QUEUE_SIZE;
    }
    __set_PRIMASK(primask);                /* 退出临界区，恢复原中断使能状态 */
}

void process_uart_queue(void)
{
    if (dma_tx_complete && uart_queue_tail != uart_queue_head) {
        dma_tx_complete = 0;
        HAL_UART_Transmit_DMA(&huart1,
                              (uint8_t *)uart_queue[uart_queue_tail].buffer,
                              uart_queue[uart_queue_tail].length);
        uart_queue_tail = (uart_queue_tail + 1) % UART_QUEUE_SIZE;
    }
}

/* ===== HAL 中断回调 ===== */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        dma_tx_complete = 1;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        if (rx_byte == '\n') {
            // 收到整行，解析 "{y},{x}" 坐标
            line_buffer[line_pos] = '\0';
            long parsed_y = 0;
            long parsed_x = 0;
            if (sscanf((char *)line_buffer, "%ld,%ld", &parsed_y, &parsed_x) == 2) {
                uart_y = (int32_t)parsed_y;
                uart_x = (int32_t)parsed_x;
                char tmp[64];
                snprintf(tmp, sizeof(tmp), "Parsed coordinates: X=%ld, Y=%ld\r\n",
                         parsed_y, parsed_x);
                uart_queue_send(tmp, strlen(tmp));
            }
            line_pos = 0;
        } else {
            if (line_pos < sizeof(line_buffer) - 1) {
                line_buffer[line_pos++] = rx_byte;
            }
        }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);  // 重新启动单字节接收
    }
}
