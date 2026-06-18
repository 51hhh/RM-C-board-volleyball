/**
  ******************************************************************************
  * @file    led_indicator.c
  * @brief   板载 RGB LED 状态指示实现 (高电平点亮/共阴)
  ******************************************************************************
  */
#include "led_indicator.h"
#include "main.h"   /* HAL GPIO + LED_R/G/B_Pin 宏 */
#include "scheduler.h"  /* g_tick：错误码相位锚定 */

/* 高电平点亮(共阴): SET=亮, RESET=灭 */
#define LED_ON(port, pin)   HAL_GPIO_WritePin((port), (pin), GPIO_PIN_SET)
#define LED_OFF(port, pin)  HAL_GPIO_WritePin((port), (pin), GPIO_PIN_RESET)

/* 错误码闪烁时序 */
#define ERR_BIT_MS    100   /* 每位显示时长 */
#define ERR_BITS      4     /* 码位数 */
#define ERR_GAP_MS    500   /* 一轮结束后的停顿 */
#define RUN_BLINK_MS  500   /* 正常运行绿灯闪烁半周期 */

static led_state_t s_state = LED_INIT;
static uint8_t     s_err_code = 0;   /* 当前错误码(低 4 位) */
static uint32_t    s_phase_stamp = 0; /* 当前阶段起始时间戳(ms) */

/* 同时设置三色(1=点亮) */
static void led_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    if (r) LED_ON(LED_R_GPIO_Port, LED_R_Pin); else LED_OFF(LED_R_GPIO_Port, LED_R_Pin);
    if (g) LED_ON(LED_G_GPIO_Port, LED_G_Pin); else LED_OFF(LED_G_GPIO_Port, LED_G_Pin);
    if (b) LED_ON(LED_B_GPIO_Port, LED_B_Pin); else LED_OFF(LED_B_GPIO_Port, LED_B_Pin);
}

void led_init(void)
{
    led_rgb(0, 0, 0);
    s_state = LED_INIT;
    s_phase_stamp = 0;
}

void led_set_init(void)    { s_state = LED_INIT; }
void led_set_running(void) { s_state = LED_RUNNING; }

void led_set_error(uint8_t code)
{
    s_err_code = code & 0x0F;   /* 仅低 4 位 */
    s_phase_stamp = g_tick;     /* 锚定到进入错误的时刻，保证首轮从最高位开始 */
    s_state = LED_ERROR;
}

/* 节拍驱动状态机：now = g_tick (ms) */
void led_update(uint32_t now)
{
    switch (s_state) {
    case LED_INIT:
        led_rgb(0, 0, 1);              /* 蓝色常亮 */
        break;

    case LED_RUNNING: {
        /* 绿色闪烁：每 RUN_BLINK_MS 翻转一次 */
        uint8_t on = ((now / RUN_BLINK_MS) & 1U) == 0;
        led_rgb(0, on, 0);
        break;
    }

    case LED_ERROR: {
        /* 一轮总时长 = 4 位 x 100ms + 停顿 500ms = 900ms */
        uint32_t cycle = (uint32_t)ERR_BITS * ERR_BIT_MS + ERR_GAP_MS;
        uint32_t pos = (now - s_phase_stamp) % cycle;  /* 轮内位置 */
        if (pos < (uint32_t)ERR_BITS * ERR_BIT_MS) {
            uint32_t idx = pos / ERR_BIT_MS;           /* 第几位(0..3) */
            /* 高位在前：bit3 先显示 */
            uint8_t  bit = (s_err_code >> (ERR_BITS - 1 - idx)) & 1U;
            if (bit) led_rgb(0, 1, 0);                 /* 1 -> 绿 */
            else     led_rgb(1, 0, 0);                 /* 0 -> 红 */
        } else {
            led_rgb(0, 0, 0);                          /* 停顿全灭 */
        }
        break;
    }

    default:
        led_rgb(0, 0, 0);
        break;
    }
}

