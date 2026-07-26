/**
 * @file    gyro.h
 * @brief   基于 MPU6050 yaw 的角度 PID 控制器
 *
 * 用途：控制小车精确旋转到指定角度（用于直角转弯）
 *
 * 输出为左右轮差速值（m/s），正值 = 右转，负值 = 左转。
 * 调用方自行将差速叠加到目标速度上：
 *   g_target_speed_L = base_speed + output;
 *   g_target_speed_R = base_speed - output;
 */

#ifndef GYRO_H
#define GYRO_H

#include <stdint.h>

/* ---------- 角度 PID 参数 ---------- */
#define GYRO_KP   1.5f      /* 比例系数 */
#define GYRO_KI   0.02f     /* 积分系数 */
#define GYRO_KD   0.1f      /* 微分系数 */

/* ---------- 输出限幅 ---------- */
#define GYRO_OUTPUT_MAX   0.5f    /* 最大差速输出（m/s） */

/* ---------- API ---------- */
void    Gyro_Init(void);                           /* 初始化 PID 状态 */
void    Gyro_SetTarget(float target_yaw_deg);      /* 设置目标角度（度） */
float   Gyro_GetCurrent(void);                     /* 获取当前 yaw 角（度） */
float   Gyro_Update(void);                         /* PID 更新，返回差速值（m/s）*/
uint8_t Gyro_IsDone(float tolerance_deg);          /* 是否到达目标（容差内） */
void    Gyro_Reset(void);                          /* 重置 PID 状态 */

#endif /* GYRO_H */
