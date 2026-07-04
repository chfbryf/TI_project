#ifndef __ALLDATA_H__
#define __ALLDATA_H__

#include "main.h"
#include "ti_msp_dl_config.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#include  "Delay.h"
#include "Key.h"
#include "Motor.h"
#include "LED.h"
#include "mpu6050.h"
#include "mspm0_i2c.h"
#include "PID.h"
#include "Uart.h"
#include "adc.h"
#include "CCD.h"
#include "work.h"
#include "StepMotor.h"
float new_yaw;
uint8_t oled_buffer[32];

/* 按键输入状态 */
uint8_t keynum;      // 当前按键值
uint8_t keycnt;      // 按键次数/模式选择
uint8_t keyquan;     // 按键模式选择值
uint8_t start_flag;  // 启动按键触发

/* 小车循迹/任务状态 */
uint8_t xunji_flag;  // 是否进入循迹控制
uint8_t kaishi_flag; // 是否开始运行
uint8_t quanshu;     // 选定圈数
uint8_t m0;          // 当前转弯计数或黑白线计数
uint8_t xia_flag;    // 线路检测阶段标志
uint8_t baohu_flag;  // 保护期标志，避免抖动重复计数
uint8_t biansu_flag; // 变速/减速阶段标志
uint8_t yizhi_flag;  // 匀速恢复阶段标志

/* 备用/未使用标志 */
uint8_t clear_flag;
uint8_t work_1;
uint8_t work_flag;
uint8_t far_flag;

int16_t Tick_work;
_st_tick Tick;
_st_encoder Encoder;
_st_motor Motor;
_st_mpu Mpu;
tPid EncoderLPid,EncoderRPid,GxPid,TurnErrorPid,jiaoduPid;
int data;
void Timer_work(void);
int32_t L_PWM,R_PWM,jiaodu,tick_jiaodu,time_2s,t_200ms,t_3000ms,time_3s;
unsigned char L2,L1,M0,R1,R2;
#endif
