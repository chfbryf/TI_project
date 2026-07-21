/**
 * @file    encoder.h
 * @brief   编码器脉冲计数与速度计算（简化版，对标10_DC_MOTOR_PID_3工程）
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "ti_msp_dl_config.h"

/* ---------- 电机机械参数（与参考工程一致） ---------- */
#define MOTOR_BIANMAQI  360U    /* 编码器线数（每圈脉冲数） */
#define MOTOR_WHEEL_D     65U   /* 轮子直径，单位 mm          */
#define PI              3.14f

/* ---------- 脉冲计数器（GPIO中断累加，定时器ISR清零） ---------- */
extern volatile uint32_t counter_R_A;   /* 右轮编码器 A 相脉冲计数 */
extern volatile uint32_t counter_L_A;   /* 左轮编码器 A 相脉冲计数 */

/* ---------- API ---------- */
void Encoder_Init(void);            /* 编码器初始化（GPIO已由SysConfig配置） */
void Encodering(void);              /* GPIO 中断中调用：仅做脉冲累加         */
void Encoder_UpdateSpeeds(void);    /* 50ms 定时器 ISR 中调用：计算速度并清零计数器 */
float GetSpeed_L(void);             /* 获取左轮当前速度，mm/s                */
float GetSpeed_R(void);             /* 获取右轮当前速度，mm/s                */

#endif /* ENCODER_H */
