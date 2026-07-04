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
#include "sensor.h"
#include "control.h"
#include "key.h"
#include "pid.h"
#include "sys.h"
#include "gyro.h"

static volatile uint32_t biansu_time; 
static volatile uint32_t yunsu_time; 
static volatile uint32_t baohu_time;

static volatile uint8_t Tick_angle_pid;  //角度环计算标志位

static volatile uint8_t  pid_calc_flag;
static volatile uint8_t  trace_flag;

static volatile uint8_t baohu_flag; 
static volatile uint8_t biansu_flag; 
static volatile uint8_t yunsu_flag; 
static volatile uint8_t xia_flag; 
static volatile uint8_t quanshu;
static volatile uint8_t m0; //转弯标志

uint8_t oled_buffer[32];
static volatile float speed_L = 0.0f;
static volatile float speed_R = 0.0f;

static volatile int8_t sensor;

void renwu(void) //任务函数
{
    uint8_t threshold = 0;
    uint8_t target_speed = 0;

    if (quanshu == 0)
        return;

    if (quanshu == 1) {
        if (keyspeed == 3) {
            target_speed = 30;
        }
         else if (keyspeed == 2) {
            target_speed = 20;
        }
         else if(keyspeed == 1){
            target_speed = 10;
        }
        else {
            target_speed = 0;
        }
    }
        threshold = quanshu * 4;

    if (m0 >= threshold) {
        pid_calc_flag = 0;
        trace_flag = 0;
        start_flag = 0;
        App_PWM_Set_L(0);
        App_PWM_Set_R(0);
        base_speed = 0;
        quanshu = 0;
        keyspeed = 0;
        keyquan = 0;
        m0 = 0;
    }
    else{
        base_speed = target_speed;
    }
}

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();

    //MPU6050_Init();
    OLED_Init();
    Encoder_Init();
    App_Motor_Init();

    //初始化led
    LED4_High;

    /* Don't remove this! */
    Interrupt_Init();

    /* 使能循迹PID定时器中断 */
    NVIC_EnableIRQ(TIMER_xunji_pid_INST_INT_IRQN);


    /*OLED_ShowString(0,0,(uint8_t *)"Pitch",8);
    OLED_ShowString(0,2,(uint8_t *)" Roll",8);
    OLED_ShowString(0,4,(uint8_t *)"  Yaw",8);*/

    OLED_ShowString(0,0,(uint8_t *)"m0",8);
    OLED_ShowString(0,2,(uint8_t *)"base",8);
    OLED_ShowString(0,4,(uint8_t *)"quan",8);
    OLED_ShowString(0,6,(uint8_t *)"speed",8);

    while (1) 
    {

        key_work();

        //oled显示mpu6050数据
        /*sprintf((char *)oled_buffer, "%-6.1f", pitch);
        OLED_ShowString(5*8,0,oled_buffer,16);
        sprintf((char *)oled_buffer, "%-6.1f", roll);
        OLED_ShowString(5*8,2,oled_buffer,16);
        sprintf((char *)oled_buffer, "%-6.1f", yaw);
        OLED_ShowString(5*8,4,oled_buffer,16);*/

        //oled显示圈数
        sprintf((char *)oled_buffer, "%d", m0);
        OLED_ShowString(5*8,0,oled_buffer,16);
        sprintf((char *)oled_buffer, "%d", base_speed);
        OLED_ShowString(5*8,2,oled_buffer,16);
        sprintf((char *)oled_buffer, "%d", keyquan);
        OLED_ShowString(5*8,4,oled_buffer,16);
        sprintf((char *)oled_buffer, "%d", keyspeed);
        OLED_ShowString(5*8,6,oled_buffer,16);


        //串口显示速度
        speed_L = GetSpeed_L();
        speed_R = GetSpeed_R();
        printf("%3f, %3f\n", speed_L, speed_R);
                

        //防止重复
        if (baohu_flag == 0){
                switch (xia_flag) {
            case 0:
                if (L2 != 0) {
                    xia_flag = 1;
                }
                break;
            case 1:
                if (L2 == 0) {
                    baohu_flag = 1;
                    m0++;
                    if (keyquan != 0)
                        biansu_flag = 1;
                }
                break;
        }
        }


        // 保护期计时，用于避免线路检测抖动 
        if (baohu_flag == 1)
        {    
            if (baohu_time >= 3000)
            {
                baohu_flag = 0;
                baohu_time = 0;
                xia_flag = 0;
            }
        }


        //运行圈数

            if (start_flag == 1)
            {
                    pid_calc_flag = 1; // 开始计算 PID
                    quanshu = keyquan;
                    trace_flag = 1;
                    renwu(); // 执行跑圈任务
            }
        

        // 变速阶段处理 
    if (biansu_flag == 1)
    {
        if (biansu_time < 3000)
        {
            base_speed -= 10;
            if(base_speed < 0)
            {
                base_speed = 0;
            }
        }
        else
        {
            biansu_time = 0;
            biansu_flag = 0;
            yunsu_flag = 1;
        }
    }

    // 匀速恢复阶段 
    if (yunsu_flag == 1)
    {
        if (yunsu_time >= 500)
        {
            if (base_speed < 20)
            {
                base_speed++;
                yunsu_time = 0;
            }
            else
            {
                yunsu_flag = 0;
            }
        }
    }
    App_Motor_Proc(trace_flag);

        //mspm0_delay_ms(500);

    }
}


/**
 * @brief 定时器中断回调函数（10ms周期）
 */
void TIMER_xunji_pid_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_xunji_pid_INST)){
        case DL_TIMER_IIDX_ZERO:
        {

            //三秒保护
            if(baohu_flag == 1)
            {
                baohu_time++;
            }
            if(biansu_flag == 1)
            {
                biansu_time++;
            }
            if(yunsu_flag == 1)
            {
            yunsu_time++;
            }

        //陀螺仪控制pid
        //GYRO_Proc(float target_yaw);

        //循迹pid运算
        if(pid_calc_flag == 1)
        {
                if(Tick_angle_pid++ >= 6)  //6ms执行一次
                {
                    xunji_Proc();
                    Tick_angle_pid = 0;
                }
            
        }

        }
            break;
        default:
            break;
    }
}
