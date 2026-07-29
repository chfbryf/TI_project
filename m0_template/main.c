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


static uint8_t  time_prev_start = 0;  /* 用于检测 key.start 上升沿 */
static uint32_t track_start_ms  = 0;  /* 行驶起始时间 */
volatile uint32_t test_ms = 0;       /* 测试用1ms计数器，在TIMER_xunji_pid ISR中自增 */
uint8_t oled_buffer[32];

/* ================================================================
 * 辅助函数（封装主循环各环节）
 * ================================================================ */

/**
 * @brief 更新 OLED 显示：行驶时间 / Digtal / 圈数 / 档位
 */
static void OLED_UpdateStatus(void)
{
    /* 行驶时间：按键2按下时计时，松开归零 */
    if (!time_prev_start && key.start) track_start_ms = test_ms;
    time_prev_start = key.start;
    sprintf((char *)oled_buffer, "%.1f", key.start ? (test_ms - track_start_ms) / 1000.0f : 0.0f);
    OLED_ShowString(5*8, 0, oled_buffer, 16);

    /* Digtal（由 SensorUpdate 更新） */
    sprintf((char *)oled_buffer, "%02X", Digtal);
    OLED_ShowString(5*8, 2, oled_buffer, 16);

    /* 圈数 */
    sprintf((char *)oled_buffer, "%d", key.quan);
    OLED_ShowString(5*8, 4, oled_buffer, 16);

    /* 档位 */
    sprintf((char *)oled_buffer, "%d", key.keyspeed);
    OLED_ShowString(5*8, 6, oled_buffer, 16);
}

/**
 * @brief 读取红外传感器，更新 Digtal 和 err2
 * @return Digtal 值
 */
static uint8_t SensorUpdate(void)
{
    uint8_t ir[8];
    IR_Read(ir);
    Digtal = (ir[0]<<7) | (ir[1]<<6) | (ir[2]<<5) | (ir[3]<<4)
           | (ir[4]<<3) | (ir[5]<<2) | (ir[6]<<1) | (ir[7]<<0);
    Get_err2();
    return Digtal;
}

/**
 * @brief 检测十字停车条件（多路见黑 + 黑线居中）
 * @return 1=已停车, 0=未触发
 */
static uint8_t CheckStop(uint8_t d)
{
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (!(d & (1 << i))) cnt++;
    }
    if (cnt >= 7 && abs(Err2()) <= 1) {
        motor_stop();
        key.start = 0;
        return 1;
    }
    return 0;
}

/**
 * @brief 循迹控制：按键2启停，PID 输出左右目标速度
 */
static void TrackingRun(void)
{
    if (key.start) {
        speed(key.keyspeed);
        Tracking_SpeedLoop(Err2(), (float)base_speed);
    } else {
        g_target_speed_L = 0.0f;
        g_target_speed_R = 0.0f;
    }
}

/* ================================================================
 * main
 * ================================================================ */

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();
    MPU6050_Init();
    OLED_Init();
    //Init_ICM42688();
    Encoder_Init();
    SpeedCtrl_Init();
    Servo_Init();

    /* 启动 PWM 定时器（TIMG8）与速度环定时器（SPEED_PID: TIMG6, 50ms） */
    DL_TimerG_startCounter(PWM_0_INST);
    DL_TimerG_startCounter(SPEED_PID_INST);
    NVIC_EnableIRQ(SPEED_PID_INST_INT_IRQN);

    /* 初始化 LED */
    LED4_High;
    LED3_High;

    /* Don't remove this! */
    Interrupt_Init();

    /* 使能循迹PID定时器中断 */
    NVIC_EnableIRQ(TIMER_xunji_pid_INST_INT_IRQN);

    /* 初始化红外循迹传感器（UART1, 115200） */
    IR_Init();

    OLED_ShowString(0,0,(uint8_t *)"time",8);
    OLED_ShowString(0,2,(uint8_t *)"digtal",8);
    OLED_ShowString(0,4,(uint8_t *)"quanshu",8);
    OLED_ShowString(0,6,(uint8_t *)"speed",8);

    while (1)
    {
        key_work();                     /* 1. 按键扫描 */
        {
            uint8_t d = SensorUpdate(); /* 2. 传感器读取 + 误差计算 */
            OLED_UpdateStatus();        /* 3. OLED 刷新 */
            if (CheckStop(d)) continue; /* 4. 十字停车检测 */
            TrackingRun();              /* 5. 循迹 PID */
        }
        //mspm0_delay_ms(1000);
    }
}

/**
 * @brief 定时器中断回调函数（1ms周期）
 */
void TIMER_xunji_pid_INST_IRQHandler(void)
{
    delay_flag++;
    test_ms++;
}
