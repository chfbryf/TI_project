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

#include "sys.h"
#include <stdlib.h>

/* ================================================================
 * 全局变量
 * ================================================================ */
static uint8_t  time_prev_start = 0;  /* key.start 上升沿检测 */
static uint32_t track_start_ms  = 0;  /* 行驶起始时间 */
volatile uint32_t test_ms = 0;       /* 1ms 计数器 */
static char oled_buf[16];

/* ================================================================
 * OLED 显示
 * ================================================================ */
static void OLED_UpdateStatus(void)
{
    /* 循迹时间：按键启动后计时 */
    if (!time_prev_start && key.start) track_start_ms = test_ms;
    time_prev_start = key.start;
    float sec = key.start ? (test_ms - track_start_ms) / 1000.0f : 0.0f;

    sprintf(oled_buf, "%.1fs", sec);
    OLED_ShowString(0, 0, (uint8_t *)oled_buf, 16);

    /* Digtal：红外传感器位图 */
    sprintf(oled_buf, "%02X", Digtal);
    OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);

    /* 按键1：速度档位 */
    sprintf(oled_buf, "Spd:%d", key.keyspeed);
    OLED_ShowString(0, 4, (uint8_t *)oled_buf, 16);

    /* 按键3：圈数 */
    sprintf(oled_buf, "Lap:%d", key.quan);
    OLED_ShowString(0, 6, (uint8_t *)oled_buf, 16);
}

/* ================================================================
 * 传感器读取
 * ================================================================ */
static uint8_t SensorUpdate(void)
{
    uint8_t ir[8] = {1, 1, 1, 1, 1, 1, 1, 1};  /* 默认全白，防止首帧未到 */
    IR_Read(ir);
    Digtal = (ir[0]<<7) | (ir[1]<<6) | (ir[2]<<5) | (ir[3]<<4)
           | (ir[4]<<3) | (ir[5]<<2) | (ir[6]<<1) | (ir[7]<<0);
    Get_err2();
    return Digtal;
}

/* ================================================================
 * 十字停车检测
 * ================================================================ */
static uint8_t CheckStop(uint8_t d)
{
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (!(d & (1 << i))) cnt++;
    }
    if (cnt >= 7 && abs(Err2()) <= 1) {
        StepTrack_Stop();
        key.start = 0;
        return 1;
    }
    return 0;
}

/* ================================================================
 * main
 * ================================================================ */

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();
    OLED_Init();

    /* 步进电机 + 循迹 */
    step_motor_Init();
    StepTrack_Init();

    /* ICM42688 姿态传感器（未接硬件时注释掉） */
    //Init_ICM42688();

    /* LED */
    LED4_High;
    LED3_High;

    Interrupt_Init();

    /* 1ms 定时器 */
    NVIC_EnableIRQ(TIMER_xunji_pid_INST_INT_IRQN);

    /* 红外循迹传感器 */
    IR_Init();

    while (1)
    {
        key_work();                     /* 1. 按键扫描 */

        uint8_t d = SensorUpdate();     /* 2. IR 传感器 + 误差 */

        //ICM42688_ReadAndCompute();    /* 3. ICM42688 姿态解算（未接时注释） */

        OLED_UpdateStatus();            /* 4. OLED: 时间 / Digtal / Spd / Lap */

        if (CheckStop(d)) continue;     /* 5. 十字停车 */

        StepTrack_Run();                /* 6. 步进电机循迹 */
    }
}

/* ================================================================
 * 1ms 定时器 ISR
 * ================================================================ */
void TIMER_xunji_pid_INST_IRQHandler(void)
{
    delay_flag++;
    test_ms++;
}
