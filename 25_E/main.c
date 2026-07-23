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
#include "main.h"
#include "stdio.h"
#include "encoder.h"
#include "motor.h"
#include "key.h"
#include "sys.h"
#include "stdint.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "sensor2.h"
#include "speed_ctrl.h"
#include <math.h>


unsigned short Anolog[8]={0};
unsigned short white[8]={3129,2516,2376,2634,2745,2947,2290,2247};
unsigned short black[8]={730,465,358,402,370,463,279,291};
unsigned short Normal[8];


/* 直角转弯状态机 */
typedef enum {
    TURN_IDLE = 0,      // 无转弯，正常循迹
    TURN_FORWARD,       // 前进0.3s（停止循迹直行）
    TURN_SPIN,          // 原地旋转，等待中间灰度检测到黑线
    TURN_RECOVER        // 1s内加速恢复到目标速度
} TurnState;

static volatile TurnState turn_state = TURN_IDLE;
static volatile uint32_t turn_timer;     // 转弯阶段计时(ms)
static volatile float save_base_speed;   // 转弯前速度，用于恢复

volatile uint8_t quanshu;
static volatile uint8_t m0; //转弯计数
static uint8_t last_start = 0;  // 用于检测 key.start 上升沿
volatile uint32_t test_ms = 0;  // 测试用1ms计数器，在TIMER_xunji_pid ISR中自增
uint8_t oled_buffer[32];
No_MCU_Sensor sensor;

void speed(uint8_t keyspeed) //任务函数，mm/s（对标10_DC_MOTOR_PID_3工程）
{
        if(keyspeed == 5){
            base_speed = 1000;          /* 1.00 m/s（全速） */
        }
        else if(keyspeed == 4){
            base_speed = 800;           /* 0.80 m/s */
        }
        else if (keyspeed == 3) {
            base_speed = 600;           /* 0.60 m/s */
        } 
        else if (keyspeed == 2) {
            base_speed = 400;           /* 0.40 m/s */
        } 
        else if(keyspeed == 1){
            base_speed = 200;           /* 0.20 m/s */
        }
        else {
            base_speed = 0;
        }
}

void renwu(void) //任务函数
{
    uint8_t threshold = 0;
    if (quanshu == 0)
        return;
        
    threshold = quanshu * 4;
    if (m0 >= threshold) {
        key.start = 0;      /* 速度环 else 分支以目标 0 停车 */
        base_speed = 0;
        quanshu = 0;
        key.keyspeed = 0;
        key.quan = 0;
        m0 = 0;
        turn_state = TURN_IDLE;
        turn_timer = 0;
        SpeedCtrl_Reset();        /* 清积分，快速制动 */
    }
}

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();
    MPU6050_Init();
    OLED_Init();
    Encoder_Init();
    SpeedCtrl_Init();

    /* 启动 PWM 定时器（TIMG8）与速度环定时器（SPEED_PID: TIMG6, 50ms），对标参考工程 motor_init */
    DL_TimerG_startCounter(PWM_0_INST);
    DL_TimerG_startCounter(SPEED_PID_INST);
    NVIC_EnableIRQ(SPEED_PID_INST_INT_IRQN);

    //初始化led
    LED4_High;

    /* Don't remove this! */
    Interrupt_Init();

    /* 使能循迹PID定时器中断 */
    NVIC_EnableIRQ(TIMER_xunji_pid_INST_INT_IRQN);

    //根据黑白校准值初始化传感器
    No_MCU_Ganv_Sensor_Init(&sensor,white,black);

    //设置DMA搬运的起始地址
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t) &ADC0->ULLMEM.MEMRES[0]);

    //设置DMA搬运的目的地址
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t) &ADC_VALUE[0]);

    //开启DMA
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);

    //开启ADC转换
    DL_ADC12_startConversion(ADC12_0_INST);

    OLED_ShowString(0,0,(uint8_t *)"m0",8);
    OLED_ShowString(0,2,(uint8_t *)"digtal",8);
    OLED_ShowString(0,4,(uint8_t *)"quanshu",8);
    OLED_ShowString(0,6,(uint8_t *)"speed",8);

    while (1) 
    {
        key_work();

        if (turn_state == TURN_IDLE) {
            speed(key.keyspeed);
        }

        //oled显示圈数
        sprintf((char *)oled_buffer, "%d", m0);
        OLED_ShowString(5*8,0,oled_buffer,16);
        
        sprintf((char *)oled_buffer, "%x", Digtal);
        OLED_ShowString(5*8,2,oled_buffer,16);
        
        sprintf((char *)oled_buffer, "%d", key.quan);
        OLED_ShowString(5*8,4,oled_buffer,16);
        
        sprintf((char *)oled_buffer, "%d", key.keyspeed);
        OLED_ShowString(5*8,6,oled_buffer,16);

        No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);

        /* 获取传感器数字量结果(只有当有黑白值传入进去了之后才会有这个值！) */
        Digtal=Get_Digtal_For_User(&sensor);

        Get_err2();   /* 更新 err2，供 Err2() 返回 */

        /*---- 直角检测（最左3个传感器全部丢线且持续50ms，含冷却保护）----*/
        if (key.start == 1 && turn_state == TURN_IDLE) {
            static uint32_t left_lost_start = 0;
            static uint8_t  left_lost_flag  = 0;

            if (Digtal == 0xff || (Digtal & 0x07) == 0 ) {     /* bit2/1/0 = 最左3个传感器全部看到黑色（丢线） */
                if (!left_lost_flag) {
                    left_lost_start = test_ms;
                    left_lost_flag  = 1;
                } else if (test_ms - left_lost_start >= 50) {
                    /* 冷却计时：上次触发后 2500ms 内不重复触发转弯 */
                    {
                        static uint32_t last_trigger_time = 0;
                        static uint8_t  cool_down = 0;

                        if (!cool_down) {
                            /* 非冷却期：正常触发转弯 */
                            turn_state = TURN_FORWARD;
                            turn_timer = 0;
                            save_base_speed = base_speed;
                            SpeedCtrl_Reset();
                            Tracking_SpeedLoop_Reset();
                            last_trigger_time = test_ms;
                            cool_down = 1;
                        } else if (test_ms - last_trigger_time >= 2500) {
                            /* 冷却结束，允许再次触发 */
                            turn_state = TURN_FORWARD;
                            turn_timer = 0;
                            save_base_speed = base_speed;
                            SpeedCtrl_Reset();
                            Tracking_SpeedLoop_Reset();
                            last_trigger_time = test_ms;
                            cool_down = 1;  /* 重新进入冷却 */
                        }
                        /* 无论是否冷却，保护期间也计数 */
                        m0++;
                    }
                    left_lost_flag = 0;
                }
            } else {
                left_lost_flag = 0;
            }
        }

        /*---- 圈数任务（仅在正常循迹时检查，避免打断转弯）----*/
        if (key.start == 1 && turn_state == TURN_IDLE) {
            quanshu = key.quan;
            renwu();
        }

        static uint32_t last_track_ms = 0;

        /*---- 直角转弯状态机 ----*/
        switch (turn_state) {
        case TURN_FORWARD:
            if (turn_timer < 400) {
                /* 0~300ms 直行，两轮同速 */
            } else {
                /* 刹车至停稳或超时 1.2s 兜底 */
                if ((fabsf(GetSpeed_L()) < 0.02f && fabsf(GetSpeed_R()) < 0.02f)
                    || turn_timer > 1500) {
                    turn_state = TURN_SPIN;
                    turn_timer = 0;
                }
            }
            break;

        case TURN_SPIN:
            /* 原地旋转，中间两个灰度（bit4、bit3）连续确认 → 停止旋转 */
            {
                static uint8_t spin_confirm = 0;
                if ((Digtal & 0x18) != 0x18) {
                    if (++spin_confirm >= 4) {
                        turn_state = TURN_RECOVER;
                        turn_timer = 0;
                        SpeedCtrl_Reset();
                        Tracking_SpeedLoop_Reset();
                        spin_confirm = 0;
                    }
                } else {
                    spin_confirm = 0;
                }
            }
            break;

        case TURN_RECOVER:
            /* 每50ms计算一次目标速度，1s内从0线性加速 */
            if (test_ms - last_track_ms >= 50) {
                last_track_ms = test_ms;
                float progress = (float)turn_timer / 1000.0f;
                if (progress > 1.0f) progress = 1.0f;
                float recover_speed = save_base_speed * progress;
                Tracking_SpeedLoop(Err2(), recover_speed);
            }
            if (turn_timer >= 1000) {
                turn_state = TURN_IDLE;
                turn_timer = 0;
            }
            break;

        case TURN_IDLE:
        default:
            break;
        }

        /*==============================================================
         * 正常模式：每50ms调用速度环
         *==============================================================*/
        {
            static uint32_t last_speed_ms = 0;

            if (test_ms - last_speed_ms >= 50) {
                last_speed_ms = test_ms;

                if (key.start == 1 && turn_state == TURN_IDLE) {
                    /* 检测 key.start 上升沿：重置速度环 */
                    if (last_start == 0) {
                        SpeedCtrl_Reset();
                        Tracking_SpeedLoop_Reset();
                        last_start = 1;
                    }
                    SpeedCtrl_Update(g_target_speed_L, g_target_speed_R);
                } else if (turn_state == TURN_RECOVER) {
                    /* 恢复期：速度环按循迹目标运行 */
                    SpeedCtrl_Update(g_target_speed_L, g_target_speed_R);
                } else if (turn_state == TURN_FORWARD && turn_timer < 300) {
                    /* 直行 300ms，两轮同速，不循迹 */
                    SpeedCtrl_Update(base_speed / 1000.0f, base_speed / 1000.0f);
                } else if (turn_state == TURN_FORWARD) {
                    /* 300ms 后用速度环以目标 0 刹车 */
                    SpeedCtrl_Update(0.0f, 0.0f);
                } else if (turn_state == TURN_SPIN) {
                    /* 原地旋转：左轮反转 0.3 m/s，右轮正转 0.3 m/s */
                    SpeedCtrl_Update(0.07f, -0.07f);
                } else {
                    SpeedCtrl_Update(0.0f, 0.0f);
                }
            }
        }

        /*---- 正常循迹（每50ms计算一次目标速度，与速度环同步）----*/
        if (key.start == 1 && turn_state == TURN_IDLE) {
            if (test_ms - last_track_ms >= 50) {
                last_track_ms = test_ms;
                Tracking_SpeedLoop(Err2(), base_speed);
            }
        }
        if (key.start == 0) {
            last_start = 0;
        }
    }
}

/**
 * @brief 定时器中断回调函数（1ms周期）
 */
void TIMER_xunji_pid_INST_IRQHandler(void)
{
    /* 转弯计时 */
    if (turn_state != TURN_IDLE) {
        turn_timer++;
    }
    if (key.start == 1) {
        delay_flag++;
    }
    test_ms++;
}
