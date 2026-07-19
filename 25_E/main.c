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
#include "key.h"
#include "pid.h"
#include "sys.h"
#include "stdint.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "sensor2.h"



unsigned short Anolog[8]={0};
unsigned short white[8]={3221,2584,2500,2763,2716,2997,2267,2160};
unsigned short black[8]={1311,883,814,860,929,1042,707,642};
unsigned short Normal[8];

static volatile uint8_t Tick_angle_pid;  //循迹环时间计算标志位

static volatile uint32_t biansu_time; 
static volatile uint32_t yunsu_time; 
static volatile uint32_t baohu_time;
static volatile float save_base_speed;   // 转弯前速度，用于减速过渡
static volatile uint32_t yunsu_recovery_step;  // 恢复阶段步数



static volatile uint8_t baohu_flag; 
static volatile uint8_t biansu_flag; 
static volatile uint8_t yunsu_flag; 
static volatile uint8_t xia_flag; 
volatile uint8_t quanshu;
static volatile uint8_t m0; //转弯标志



uint8_t oled_buffer[32];


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

void renwu(void) //任务函数
{
    uint8_t threshold = 0;

    if (quanshu == 0)
        return;

        threshold = quanshu * 4;

    if (m0 >= threshold) {
        key.start = 0;
        App_PWM_Set_L(0);
        App_PWM_Set_R(0);
        base_speed = 0;
        quanshu = 0;
        key.keyspeed = 0;
        key.quan = 0;
        m0 = 0;
        biansu_flag = 0;
        yunsu_flag = 0;
        xia_flag = 0;
        baohu_flag = 0;
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


    OLED_ShowString(0,0,(uint8_t *)"m0",8);
    OLED_ShowString(0,2,(uint8_t *)"digtal",8);
    OLED_ShowString(0,4,(uint8_t *)"quanshu",8);
    OLED_ShowString(0,6,(uint8_t *)"speed",8);

    while (1) 
    {
        key_work();
        if (!biansu_flag && !yunsu_flag) {
            speed(key.keyspeed);
        }

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
        sprintf((char *)oled_buffer, "%x", Digtal);
        OLED_ShowString(5*8,2,oled_buffer,16);
        sprintf((char *)oled_buffer, "%d", key.quan);
        OLED_ShowString(5*8,4,oled_buffer,16);
        sprintf((char *)oled_buffer, "%d", key.keyspeed);
        OLED_ShowString(5*8,6,oled_buffer,16);

        
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


        if (baohu_flag == 0){
                switch (xia_flag) {
            case 0:
                if ((Digtal & 0xe0) == 0) {
                    xia_flag = 1;
                    // 立即清除PID残留，避免保持上次的强转向值
                    App_PWM_Set_L(base_speed);
                    App_PWM_Set_R(base_speed);
                }
                break;
            case 1:
                if ((Digtal & 0xe0) != 0) {
                    baohu_flag = 1;
                    m0++;
                    biansu_time = 0;
                    if (key.quan != 0)
                        biansu_flag = 1;
                }
                break;
        }
        }


        // 保护期计时，用于避免线路检测抖动 
        if (baohu_flag == 1)
        {    
            if (baohu_time >= 4000)
            {
                baohu_flag = 0;
                baohu_time = 0;
                xia_flag = 0;
            }
        }


        //运行圈数

            if (key.start == 1)
            {
                    quanshu = key.quan;
                    renwu(); // 执行跑圈任务
            }
        

        // 直角转弯处理 
    if (biansu_flag == 1)
    {
        if (biansu_time < 300)    // 强制转弯300ms后退出
        {
            App_PWM_Set_L(18);
            App_PWM_Set_R(5);
            key.start = 0;
        }
        else
        {
            biansu_time = 0;
            biansu_flag = 0;
            yunsu_flag = 1;
            yunsu_time = 0;
            xia_flag = 0;
            save_base_speed = base_speed;
            yunsu_recovery_step = 0;
            Motor_PID_Reset();  // 清除转弯期间PID积分残留
            key.start = 1;
        }
    }

    // 匀速恢复阶段 
    if (yunsu_flag == 1)
    {
        if (yunsu_time < 300)
        {
            // 阶段1：0.3s内从当前速度线性减速到5
            float progress = (float)yunsu_time / 300.0f;
            base_speed = save_base_speed + (5.0f - save_base_speed) * progress;
        }
        else
        {
            // 阶段2：每25ms加1，快速恢复到目标速度
            uint32_t target_step = (yunsu_time - 300) / 25;
            while (yunsu_recovery_step < target_step)
            {
                yunsu_recovery_step++;
                if (base_speed < 20)
                {
                    base_speed++;
                }
                else
                {
                    yunsu_flag = 0;
                    break;
                }
            }
        }
    }


        if(key.start == 1)
        {
            if(Tick_angle_pid >= 6)
            {
                if (biansu_flag == 0)
                {
                    if (xia_flag == 1)
                    {
                        // 检测到直角入口，暂停循迹修正，保持直行
                        App_PWM_Set_L(base_speed);
                        App_PWM_Set_R(base_speed);
                    }
                    else
                    {
                        xunji_Proc();
                    }
                }
                Tick_angle_pid = 0;
            }
        }


    }
}


/**
 * @brief 定时器中断回调函数（1ms周期）
 */
void TIMER_xunji_pid_INST_IRQHandler(void)
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

    if(key.start == 1)
    {
       delay_flag++;
    }
    

    Tick_angle_pid++;
}
