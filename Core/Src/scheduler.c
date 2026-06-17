/**
  ******************************************************************************
  * @file    scheduler.c
  * @brief   1kHz 时基(TIM6) + DWT 微秒计时 + 非阻塞延时事件调度 实现
  ******************************************************************************
  */
#include "scheduler.h"
#include "main.h"   /* HAL 类型、CMSIS(DWT/CoreDebug)、SystemCoreClock */

volatile uint32_t g_tick = 0;
volatile uint8_t  g_tick_flag = 0;

static TIM_HandleTypeDef htim6;

/* ===== 延时事件表 ===== */
#define SCHED_MAX_EVENTS 8
typedef struct {
    uint32_t due;     /* 到期的 g_tick 值 */
    evt_cb_t cb;      /* 回调 */
    uint8_t  used;    /* 槽位占用 */
} sched_evt_t;
static sched_evt_t s_evts[SCHED_MAX_EVENTS];

/* ===== DWT 微秒计数器 ===== */
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t micros(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

/* ===== 时基：TIM6 @ 1kHz =====
 * TIM6 挂在 APB1，定时器时钟 = APB1(42MHz) x2 = 84MHz
 * 84MHz / (PSC+1=84) / (ARR+1=1000) = 1000 Hz
 */
void timebase_init(void)
{
    dwt_init();

    __HAL_RCC_TIM6_CLK_ENABLE();
    htim6.Instance               = TIM6;
    htim6.Init.Prescaler         = 83;     /* 84MHz/84 = 1MHz */
    htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6.Init.Period            = 999;    /* 1MHz/1000 = 1kHz */
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim6);

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    HAL_TIM_Base_Start_IT(&htim6);
}

/* ===== 延时事件调度 ===== */
// 注：schedule_after 可能在中断上下文(如 EXTI 回调)被调用，而 sched_tick 在主
// 循环上下文遍历事件表。这里用临界区保护"找空槽并占用"，避免两个上下文抢同一槽位；
// 并把 used 标志最后置位，确保 sched_tick 不会读到半初始化的事件。
int schedule_after(uint32_t delay_ms, evt_cb_t cb)
{
    int ret = -1;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();                       /* 进入临界区 */
    for (int i = 0; i < SCHED_MAX_EVENTS; i++) {
        if (!s_evts[i].used) {
            s_evts[i].due  = g_tick + delay_ms;
            s_evts[i].cb   = cb;
            s_evts[i].used = 1;            /* 最后置位 */
            ret = 0;
            break;
        }
    }
    __set_PRIMASK(primask);                /* 退出临界区，恢复原中断使能状态 */
    return ret;
}

// 每个节拍在主循环上下文调用，扫描并触发到期事件
void sched_tick(uint32_t now)
{
    for (int i = 0; i < SCHED_MAX_EVENTS; i++) {
        /* (int32_t) 差值比较，可正确处理 g_tick 回绕 */
        if (s_evts[i].used && (int32_t)(now - s_evts[i].due) >= 0) {
            evt_cb_t cb = s_evts[i].cb;
            s_evts[i].used = 0;     /* 先释放槽位，回调里可再次 schedule */
            if (cb) cb();
        }
    }
}

/* ===== 中断 ===== */
void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim6);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        g_tick++;
        g_tick_flag = 1;
    }
}
