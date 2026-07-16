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
#include "stdint.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "sensor2.h"



unsigned short Anolog[8]={0};
unsigned short white[8]={3221,2584,2500,2763,2716,2997,2267,2160};
unsigned short black[8]={1311,883,814,860,929,1042,707,642};
unsigned short Normal[8];

static volatile uint8_t Tick_angle_pid;  //循迹环时间计算标志位
static volatile uint8_t Tick_gyro_pid;  //转向环时间计算标志位
static volatile uint8_t Tick_motor_pid; //电机环时间计算标志位

static volatile uint8_t  gyro_flag; //陀螺仪标志位
static volatile uint8_t  pid_calc_flag; //速度环标志位
static volatile uint8_t  trace_flag;    //循迹环标志位
static volatile float target_omega; //目标yaw角

uint8_t oled_buffer[32];
static volatile float speed_L = 0.0f;
static volatile float speed_R = 0.0f;

volatile int16_t poss;

No_MCU_Sensor sensor;


void speed(uint8_t keyspeed) //任务函数
{

        if(keyspeed == 5){
            base_speed = 70;
        }
        else if(keyspeed == 4){
            base_speed = 50;
        }
        else if (keyspeed == 3) {
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

void renwu(uint8_t keymode)
{
    static uint8_t last_keymode = 0;
    if(keymode != last_keymode)
    {
        renwu_reset();
        last_keymode = keymode;
    }

    switch(keymode)
    {
        case 1:
        renwu1();
            break;
        case 2:
         renwu2();
            break;
        case 3:
         renwu3();
            break;
        case 4:
         renwu4();
            break;
        default:
            break;
    }
}

void change_mode(uint8_t renwu_mode)
{
        switch(renwu_mode)
        {
            case 1:
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
                key.keymode = 0;
                key.keyspeed = 0;
                mode = 0;
            }
                break;

            case 2:
            {
                gyro_flag = 1;
                target_omega = 0;
                trace_flag = 0;
            }
                break;

            case 3:
            {
                gyro_flag = 0;
                trace_flag = 1;
            }
                break;
            case 4:
            {
                
                if(omega_flag == 0)target_omega = 32.0;
                if(omega_flag == 1)target_omega = 152.0;
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
    /*OLED_ShowString(0,0,(uint8_t *)"Pitch",8);
    OLED_ShowString(0,2,(uint8_t *)" Roll",8);
    OLED_ShowString(0,4,(uint8_t *)"  Yaw",8);*/


    OLED_ShowString(0,0,(uint8_t *)"yaw",8);
    OLED_ShowString(0,2,(uint8_t *)"digtal",8);
    OLED_ShowString(0,4,(uint8_t *)"renwu",8);
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
        sprintf((char *)oled_buffer, "%x", Digtal);
        OLED_ShowString(5*8,2,oled_buffer,16);
        sprintf((char *)oled_buffer, "%d", key.keymode);
        OLED_ShowString(5*8,4,oled_buffer,16);
        sprintf((char *)oled_buffer, "%d", key.keyspeed);
        OLED_ShowString(5*8,6,oled_buffer,16);


        //串口显示速度
        speed_L = GetSpeed_L();
        speed_R = GetSpeed_R();
        //printf("%3f, %3f, 20\n", speed_L, speed_R);
        
            No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
		    //获取传感器数字量结果(只有当有黑白值传入进去了之后才会有这个值！！)
		    Digtal=Get_Digtal_For_User(&sensor);
            /*printf("Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n",(Digtal>>0)&0x01,(Digtal>>1)&0x01,(Digtal>>2)&0x01,(Digtal>>3)&0x01,(Digtal>>4)&0x01,(Digtal>>5)&0x01,(Digtal>>6)&0x01,(Digtal>>7)&0x01);
			if(Get_Anolog_Value(&sensor,Anolog)){
			printf("Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",Anolog[0],Anolog[1],Anolog[2],Anolog[3],Anolog[4],Anolog[5],Anolog[6],Anolog[7]);
			}
			
			//获取传感器归一化结果(只有当有黑白值传入进去了之后才会有这个值！！有黑白值初始化后返回1 没有返回 0)
			if(Get_Normalize_For_User(&sensor,Normal)){
			printf("Normalize %d-%d-%d-%d-%d-%d-%d-%d\r\n",Normal[0],Normal[1],Normal[2],Normal[3],Normal[4],Normal[5],Normal[6],Normal[7]);
			}*/

            /*Get_err2();
            poss = Err2();
            printf("%d", poss);*/
  

        //任务代码
        /*renwu(key.keymode);
        change_mode(mode);*/
        /*if(key.start_flag == 1)
        {
            pid_calc_flag = 1;
        }*/

        if(key.start_flag == 1)
        {
            trace_flag = 1;
        }

        //延时执行
        if(Tick_motor_pid >= 4)
        {
            App_Motor_Proc(pid_calc_flag);
            Tick_motor_pid = 0;
        }

        if(gyro_flag == 1)
        {
            if(Tick_gyro_pid >= 5)
            {
                GYRO_Proc(target_omega);
                Tick_gyro_pid = 0;
            }
        }

        if(trace_flag == 1)
        {
            if(Tick_angle_pid >= 6)
            {
                xunji_Proc();
                Tick_angle_pid = 0;
            }
        }

        //mspm0_delay_ms(50);

    }
}


/**
 * @brief 定时器中断回调函数（1ms周期）
 */
void TIMER_xunji_pid_INST_IRQHandler(void)
{
    if(trace_flag == 0)
    {
        delay_flag++;
    }

    Tick_gyro_pid++;
    Tick_angle_pid++;
    Tick_motor_pid++;
}
