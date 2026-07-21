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



unsigned short Anolog[8]={0};
unsigned short white[8]={3221,2584,2500,2763,2716,2997,2267,2160};
unsigned short black[8]={1311,883,814,860,929,1042,707,642};
unsigned short Normal[8];

static volatile uint8_t Tick_angle_pid;  //循迹环时间计算标志位

/* 直角转弯状态机 */
typedef enum {
    TURN_IDLE = 0,      // 无转弯，正常循迹
    TURN_FORWARD,       // 前进0.3s（停止循迹直行）
    TURN_SPIN,          // 原地旋转，等待中间灰度检测到黑线
    TURN_RECOVER        // 1s内加速恢复到目标速度
} TurnState;

static volatile TurnState turn_state = TURN_IDLE;
static volatile uint32_t turn_timer;     // 转弯阶段计时(ms)
static volatile uint32_t baohu_time;     // 转弯保护计时(ms)
static volatile float save_base_speed;   // 转弯前速度，用于恢复

static volatile uint8_t baohu_flag; 
volatile uint8_t quanshu;
static volatile uint8_t m0; //转弯计数



uint8_t oled_buffer[32];


No_MCU_Sensor sensor;


void speed(uint8_t keyspeed) //任务函数，mm/s（对标10_DC_MOTOR_PID_3）
{
        if(keyspeed == 5){
            base_speed = 1500;
        }
        else if(keyspeed == 4){
            base_speed = 1000;
        }
        else if (keyspeed == 3) {
            base_speed = 700;
        }
         else if (keyspeed == 2) {
            base_speed = 500;
        }
         else if(keyspeed == 1){
            base_speed = 300;   /* 与参考工程 target_speed_1 = 300 mm/s 一致 */
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
        turn_state = TURN_IDLE;
        turn_timer = 0;
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
        if (turn_state == TURN_IDLE) {
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
            Get_err2();   /* 更新 err2，供 Err2() 返回 */
            /*printf("Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n",(Digtal>>0)&0x01,(Digtal>>1)&0x01,(Digtal>>2)&0x01,(Digtal>>3)&0x01,(Digtal>>4)&0x01,(Digtal>>5)&0x01,(Digtal>>6)&0x01,(Digtal>>7)&0x01);
			if(Get_Anolog_Value(&sensor,Anolog)){
			printf("Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",Anolog[0],Anolog[1],Anolog[2],Anolog[3],Anolog[4],Anolog[5],Anolog[6],Anolog[7]);
			}
			
			//获取传感器归一化结果(只有当有黑白值传入进去了之后才会有这个值！！有黑白值初始化后返回1 没有返回 0)
			if(Get_Normalize_For_User(&sensor,Normal)){
			printf("Normalize %d-%d-%d-%d-%d-%d-%d-%d\r\n",Normal[0],Normal[1],Normal[2],Normal[3],Normal[4],Normal[5],Normal[6],Normal[7]);
			}*/


        /*---- 直角检测（使用 Err2()，sensor2.c 已做噪点过滤）----*/
        if (baohu_flag == 0 && turn_state == TURN_IDLE) {
            if (Err2() == -5) {   // 左三全黑 = 右直角
                turn_state = TURN_FORWARD;
                turn_timer = 0;
                g_speed_ctrl_enabled = 0;   /* 关闭速度环，改用手动 PWM 减速 */
                /* m0++; */  // TODO: 测试完后恢复
            }
        }

        /*---- 转弯保护期（避免重复检测）----*/
        if (baohu_flag == 1) {
            if (baohu_time >= 4000) {
                baohu_flag = 0;
                baohu_time = 0;
            }
        }

        /*---- 圈数任务 ----*/
        if (key.start == 1) {
            quanshu = key.quan;
            renwu();
        }

        /*---- 直角转弯状态机 ----*/
        switch (turn_state) {
        case TURN_FORWARD:
            /* 【测试】保持当前速度直行 0.3s 后停车 */
            if (turn_timer >= 300) {
                App_PWM_Set_L(0);
                App_PWM_Set_R(0);
                turn_state = TURN_IDLE;
                turn_timer = 0;
            }
            break;

        case TURN_SPIN:
            /* 原地旋转：左轮反转，右轮正转 */
            App_PWM_Set_L(-10);
            App_PWM_Set_R(10);
            /* 中间两个灰度（bit4、bit3）任一检测到黑线 → 停止 */
            if ((Digtal & 0x18) != 0x18) {
                App_PWM_Set_L(0);
                App_PWM_Set_R(0);
                turn_state = TURN_RECOVER;
                turn_timer = 0;
                g_speed_ctrl_enabled = 1;   /* 恢复速度环 */
                SpeedCtrl_Reset();
                key.start = 1;
            }
            break;

        case TURN_RECOVER:
            /* 1s内从0线性加速到目标速度，使用串级PID循迹 */
            if (Tick_angle_pid >= 6) {
                Tick_angle_pid = 0;
                float progress = (float)turn_timer / 1000.0f;
                if (progress > 1.0f) progress = 1.0f;
                float recover_speed = save_base_speed * progress;
                Tracking_SpeedLoop(Err2(), recover_speed);
            }
            if (turn_timer >= 1000) {
                turn_state = TURN_IDLE;
                turn_timer = 0;
                baohu_flag = 1;   // 进入保护期
                baohu_time = 0;
            }
            break;

        case TURN_IDLE:
        default:
            break;
        }

        /*---- 正常循迹（每6ms执行一次）----*/
        if (key.start == 1 && turn_state == TURN_IDLE) {
            if (Tick_angle_pid >= 6) {
                Tracking_SpeedLoop(Err2(), base_speed);
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
    /* 转弯计时 */
    if (turn_state != TURN_IDLE) {
        turn_timer++;
    }

    /* 保护期计时 */
    if (baohu_flag == 1) {
        baohu_time++;
    }

    if (key.start == 1) {
        delay_flag++;
    }

    Tick_angle_pid++;
}
