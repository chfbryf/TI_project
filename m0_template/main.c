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
#include "vision.h"


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

static uint8_t last_start = 0;  // 用于检测 key.start 上升沿
volatile uint32_t test_ms = 0;  // 测试用1ms计数器，在TIMER_xunji_pid ISR中自增
uint8_t oled_buffer[32];
No_MCU_Sensor sensor;

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();
    //MPU6050_Init();
    //Init_ICM42688();
    BLE_Init();
    //OLED_Init();
    Encoder_Init();
    SpeedCtrl_Init();

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

    // OLED_ShowString(0,0,(uint8_t *)"yaw",8);
    // OLED_ShowString(0,2,(uint8_t *)"digtal",8);
    // OLED_ShowString(0,4,(uint8_t *)"quanshu",8);
    // OLED_ShowString(0,6,(uint8_t *)"speed",8);

    while (1) 
    {
        key_work();

        //视觉颜色识别
        Vision_Poll();
        Vision_Apply();

        //每500ms发送一次BLE前进指令
        {
            static uint32_t last_send = 0;
            if (delay_flag - last_send >= 500) {
                last_send = delay_flag;
                BLE_SendFrame((uint8_t[]){BLE_CMD_BACKWARD, 3}, 2);
            }
        }

        //处理BLE数据
        // BLE_Poll();
        // if (BLE_FrameAvailable()) {
        //     uint8_t buf[128];
        //     BLE_ReadFrame(buf, sizeof(buf));

        //     switch (buf[0]) {
        //     case BLE_CMD_FORWARD:
        //         speed(buf[1]);
        //         g_target_speed_L =  (float)base_speed / 1000.0f;
        //         g_target_speed_R =  (float)base_speed / 1000.0f;
        //         break;
        //     case BLE_CMD_BACKWARD:
        //         speed(buf[1]);
        //         g_target_speed_L = -(float)base_speed / 1000.0f;
        //         g_target_speed_R = -(float)base_speed / 1000.0f;
        //         break;
        //     case BLE_CMD_LEFT:
        //         speed(buf[1]);
        //         g_target_speed_L = -(float)base_speed / 1000.0f;
        //         g_target_speed_R =  (float)base_speed / 1000.0f;
        //         break;
        //     case BLE_CMD_RIGHT:
        //         speed(buf[1]);
        //         g_target_speed_L =  (float)base_speed / 1000.0f;
        //         g_target_speed_R = -(float)base_speed / 1000.0f;
        //         break;
        //     case BLE_CMD_STOP:
        //         motor_stop();
        //         break;
        //     default:
        //         break;
        //     }
        // }

        //oled显示 ICM42688 yaw
        // ICM42688_ReadAndCompute();
        // sprintf((char *)oled_buffer, "%f", yaw);
        // OLED_ShowString(5*8,0,oled_buffer,16);
        
        // sprintf((char *)oled_buffer, "%x", Digtal);
        // OLED_ShowString(5*8,2,oled_buffer,16);
        
        // sprintf((char *)oled_buffer, "%d", key.quan);
        // OLED_ShowString(5*8,4,oled_buffer,16);
        
        // sprintf((char *)oled_buffer, "%d", key.keyspeed);
        // OLED_ShowString(5*8,6,oled_buffer,16);

        No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
		//获取传感器数字量结果(只有当有黑白值传入进去了之后才会有这个值！！)
		Digtal=Get_Digtal_For_User(&sensor);
        Get_err2();   /* 更新 err2，供 Err2() 返回 */


        if (turn_state == TURN_IDLE) {
            speed(key.keyspeed);
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
