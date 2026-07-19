/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "step_motor.h"
#include "uart.h"
#include "gimbal_tracker.h"
#include <stdio.h>

/* 系统毫秒计数器，由 SysTick 中断维护，供 gimbal_tracker 做动态 dt 测量 */
volatile uint32_t g_sys_tick_ms = 0;

void SysTick_Handler(void)
{
    g_sys_tick_ms++;
}

static void on_aim(uint8_t valid, const char *mode,
                   int point_index, int point_count, int point_locked,
                   float error_x, float error_y,
                   float target_x, float target_y,
                   float laser_x, float laser_y,
                   float rect_center_x, float rect_center_y,
                   float rect_confidence)
{
    (void)mode;
    (void)point_index;
    (void)point_count;
    (void)point_locked;
    (void)target_x;
    (void)target_y;
    (void)laser_x;
    (void)laser_y;
    (void)rect_center_x;
    (void)rect_center_y;
    (void)rect_confidence;

    GimbalTracker_Update(error_x, error_y, valid);
}

static void on_circle_done(int point_count)
{
    (void)point_count;
}

int main(void)
{
    SYSCFG_DL_init();

    step_motor_Init();
    GimbalTracker_Init(0.05f);
    UartParser_Init(on_aim, on_circle_done);

    /* 配置 SysTick 产生 1ms 中断，用于动态测量帧间隔 dt */
    SysTick_Config(32000000 / 1000);

    /* 发送指令给摄像头 */
    UartParser_SendString("MODE,CENTER\n");

    while (1) {
        UartParser_Process();
    }
}
/**
 * @brief printf重定向函数
 */
int __io_putchar(int ch)
{
    uint32_t t = 100000;
    while (DL_UART_isTXFIFOFull(UART_1_INST) && --t);
    if (t == 0) return ch;
    DL_UART_transmitData(UART_1_INST, (uint8_t)ch);
    for (volatile uint32_t d = 0; d < 8000; d++);
    return ch;
}

int _write(int fd, const char *ptr, int len)
{
    (void)fd;
    for (int i = 0; i < len; i++) {
        __io_putchar(ptr[i]);
    }
    return len;
}

int fputc(int ch, FILE *stream)
{
    (void)stream;
    return __io_putchar(ch);
}

int fputs(const char *restrict s, FILE *restrict stream)
{
    (void)stream;
    int char_len = 0;
    while (*s != 0) {
        __io_putchar(*s++);
        char_len++;
    }
    return char_len;
}

int puts(const char *ptr)
{
    int len = fputs(ptr, stdout);
    __io_putchar('\n');
    return len + 1;
}
