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


static volatile uint8_t Tick_angle_pid;  //循迹环时间计算标志位
static volatile uint8_t Tick_gyro_pid;  //转向环时间计算标志位

static volatile uint8_t  gyro_flag; //陀螺仪标志位
static volatile uint8_t  pid_calc_flag; //速度环标志位
static volatile uint8_t  trace_flag;    //循迹环标志位
static volatile float target_omega; //目标yaw角

uint8_t oled_buffer[32];
static volatile float speed_L = 0.0f;
static volatile float speed_R = 0.0f;


void speed(uint8_t keyspeed) //任务函数
{

        if (keyspeed == 3) {
            base_speed = 30;
        }
         else if (keyspeed == 2) {
            base_speed = 20;
        }
         else if(keyspeed == 1){
            base_speed = 10;
        }
        else {
            base_speed = 0;
        }
    
    
}

void renwu(uint8_t keyquan)
{
    switch(key.keyquan)
    {
        case 0:
        renwu1();
            break;
        case 1:
         renwu2();
            break;
        case 2:
         renwu3();
            break;
        case 3:
         renwu4();
            break;
        default:
            break;
    }
        switch(mode)
        {
            case 0:
            {
                pid_calc_flag = 0;
                gyro_flag = 0;
                trace_flag = 0;
                key.start_flag = 0;
                Motor_Reset_R();
                Motor_Reset_L();
                App_PWM_Set_L(0);
                App_PWM_Set_R(0);
                base_speed = 0;
                key.keyspeed = 0;
                key.keyquan = 0;

            }
                break;
            case 1:
            {
                trace_flag = 0;
                gyro_flag = 0;
            }
                break;

            case 2:
            {
                gyro_flag = 0;
                trace_flag = 1;
            }
                break;
            case 3:
            {
                
                if(omega_flag == 0)target_omega = 37.0;
                if(omega_flag == 1)target_omega = 142.0;
                gyro_flag = 1;

            }
                break;
            default:
                break;

        }
}

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();

    MPU6050_Init();
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

    OLED_ShowString(0,0,(uint8_t *)"yaw",8);
    OLED_ShowString(0,2,(uint8_t *)"base",8);
    OLED_ShowString(0,4,(uint8_t *)"quan",8);
    OLED_ShowString(0,6,(uint8_t *)"speed",8);

    while (1) 
    {

        key_work();
        speed(key.keyspeed);

        //oled显示mpu6050数据
        /*sprintf((char *)oled_buffer, "%-6.1f", pitch);
        OLED_ShowString(5*8,0,oled_buffer,16);
        sprintf((char *)oled_buffer, "%-6.1f", roll);
        OLED_ShowString(5*8,2,oled_buffer,16);
        sprintf((char *)oled_buffer, "%-6.1f", yaw);
        OLED_ShowString(5*8,4,oled_buffer,16);*/

        //oled显示圈数
        sprintf((char *)oled_buffer, "%f", yaw);
        OLED_ShowString(5*8,0,oled_buffer,16);
        sprintf((char *)oled_buffer, "%d", base_speed);
        OLED_ShowString(5*8,2,oled_buffer,16);
        sprintf((char *)oled_buffer, "%d", key.keyquan);
        OLED_ShowString(5*8,4,oled_buffer,16);
        sprintf((char *)oled_buffer, "%d", key.keyspeed);
        OLED_ShowString(5*8,6,oled_buffer,16);


        //串口显示速度
        speed_L = GetSpeed_L();
        speed_R = GetSpeed_R();
        printf("%3f, %3f\n", speed_L, speed_R);
                

        //任务代码
        /*renwu(key.keyquan);
        if(key.start_flag == 1)
        {
            pid_calc_flag = 1;
        }*/

        //测试代码
        if(key.keynum == 1)
        {
            //mode  = ;
        }
        
        switch(mode)
        {
            case 0:
            {
                pid_calc_flag = 0;
                gyro_flag = 0;
                trace_flag = 0;
                key.start_flag = 0;
                Motor_Reset_R();
                Motor_Reset_L();
                App_PWM_Set_L(0);
                App_PWM_Set_R(0);
                base_speed = 0;
                key.keyspeed = 0;
                key.keyquan = 0;

            }
                break;
            case 1:
            {
                trace_flag = 0;
                gyro_flag = 0;
            }
                break;

            case 2:
            {
                gyro_flag = 0;
                trace_flag = 1;
            }
                break;
            case 3:
            {
                
                if(omega_flag == 0)target_omega = 37.0;
                if(omega_flag == 1)target_omega = 142.0;
                gyro_flag = 1;

            }
                break;
            default:
                break;

        }


        App_Motor_Proc(pid_calc_flag);

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

        if(gyro_flag == 1)
        {
            if(Tick_gyro_pid++ >=5)
            {
            //陀螺仪控制pid
            GYRO_Proc(target_omega);
            }
        }

        //循迹pid运算
        if(trace_flag == 1)
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
