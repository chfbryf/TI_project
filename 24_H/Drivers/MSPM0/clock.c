#include "ti_msp_dl_config.h"
#include "clock.h"


volatile unsigned long tick_ms;
static uint32_t start_time;

/**
 * @brief 阻塞延时函数（毫秒级）
 * 
 * @param num_ms 延时时间，单位ms
 * @return int 0表示成功，1表示失败
 */
int mspm0_delay_ms(unsigned long num_ms)
{
    start_time = tick_ms;
    while (tick_ms - start_time < num_ms);
    return 0;
}

/**
 * @brief 获取当前时间（毫秒级）
 * 
 * @param count 当前时间指针，返回毫秒值
 * @return int 0表示成功，1表示失败
 */
int mspm0_get_clock_ms(unsigned long *count)
{
    if (!count)
        return 1;
    count[0] = tick_ms;
    return 0;
}


/**
 * @brief 获取当前时间（微秒级）
 * 
 * @return uint64_t 当前时间，单位us
 */
uint64_t mspm0_get_clock_us_now(void)
{
    uint32_t ms, ticks;
    do {
        ms = tick_ms;
        ticks = SysTick->LOAD - SysTick->VAL;
    } while (ms != tick_ms);
    return (uint64_t) ms * 1000 + (uint64_t) ticks * 1000 / (CPUCLK_FREQ / 1000);
}

/**
 * @brief 阻塞延时函数（微秒级）
 *
 * @param num_us 延时时间，单位us
 */
void mspm0_delay_us(unsigned long num_us)
{
    uint64_t start = mspm0_get_clock_us_now();
    while (mspm0_get_clock_us_now() - start < num_us);
}

/**
 * @brief 初始化SysTick定时器
 * 
 * 配置SysTick每1ms产生一次中断
 */
void SysTick_Init(void)
{
    DL_SYSTICK_config(CPUCLK_FREQ/1000);
    NVIC_SetPriority(SysTick_IRQn, 0);
}
