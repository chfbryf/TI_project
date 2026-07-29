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


static uint8_t last_start = 0;  /* 用于检测 key.start 上升沿 */
volatile uint32_t test_ms = 0;  /* 测试用1ms计数器，在TIMER_xunji_pid ISR中自增 */
uint8_t oled_buffer[32];

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

    /* 启动 PWM 定时器（TIMG8）与速度环定时器（SPEED_PID: TIMG6, 50ms），对标参考工程 motor_init */
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

        /* 行驶时间计时 */
        {
            static uint32_t track_start_ms = 0;
            static uint8_t  time_prev_start = 0;
            if (!time_prev_start && key.start) track_start_ms = test_ms;  /* 上升沿记录起始时间 */
            time_prev_start = key.start;
            if (key.start)
                sprintf((char *)oled_buffer, "%.1f", (test_ms - track_start_ms) / 1000.0f);
            else
                sprintf((char *)oled_buffer, "0.0");
            OLED_ShowString(5*8,0,oled_buffer,16);
        }
        
        sprintf((char *)oled_buffer, "%d", key.quan);
        OLED_ShowString(5*8,4,oled_buffer,16);
        
        sprintf((char *)oled_buffer, "%d", key.keyspeed);
        OLED_ShowString(5*8,6,oled_buffer,16);

        key_work();



        /* 读取红外循迹传感器，构建 Digtal（0=黑线, 1=白） */
        {
            uint8_t ir[8];
            IR_Read(ir);
            Digtal = (ir[0]<<7) | (ir[1]<<6) | (ir[2]<<5) | (ir[3]<<4)
                   | (ir[4]<<3) | (ir[5]<<2) | (ir[6]<<1) | (ir[7]<<0);
        }

        Get_err2();   /* 更新 err2，供 Err2() 返回 */

        sprintf((char *)oled_buffer, "%02X", Digtal);
        OLED_ShowString(5*8,2,oled_buffer,16);

        /* 循迹环：按键2（start）控制启停 */
        if (key.start) {
            speed(key.keyspeed);                                  /* 按键调速 → base_speed */
            Tracking_SpeedLoop(Err2(), (float)base_speed);        /* 误差 → 左右目标速度 */
        } else {
            g_target_speed_L = 0.0f;
            g_target_speed_R = 0.0f;
        }

        /* 权重法停车：多路见黑 + 误差接近0（黑线居中） */
        {
            uint8_t cnt = 0;
            for (uint8_t i = 0; i < 8; i++) {
                if (!(Digtal & (1 << i))) cnt++;
            }
            if (cnt >= 7 && abs(Err2()) <= 1) {
                motor_stop();       /* 急刹 + 清零积分 */
                key.start = 0;      /* 关闭总开关，防止循迹环重启 */
            }
        }

        last_start = key.start;  /* 保存本次状态，用于上升沿检测 */


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
