/**
 * @file clock.h
 * @brief 时钟和延时相关函数声明
 */

#ifndef _CLOCK_H_
#define _CLOCK_H_

#include <stdint.h>
extern volatile unsigned long tick_ms;

int mspm0_delay_ms(unsigned long num_ms);
int mspm0_get_clock_ms(unsigned long *count);
int mspm0_get_clock_us(unsigned long *count);
uint64_t mspm0_get_clock_us_now(void);
void mspm0_delay_us(unsigned long num_us);
void SysTick_Init(void);


#endif  /* #ifndef _CLOCK_H_ */