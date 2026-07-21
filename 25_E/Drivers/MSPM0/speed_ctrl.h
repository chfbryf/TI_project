/**
 * @file    speed_ctrl.h
 * @brief   双路增量式PI速度控制器（对标10_DC_MOTOR_PID_3工程）
 *
 * 与参考工程差异：
 *   - 从单路扩展为双路（左/右独立 PI）
 *   - PWM 占空比使用 [-100, +100] 浮点（适配本工程 motor.c）
 *   - 增加循迹差速接口 Tracking_SpeedLoop()
 */

#ifndef SPEED_CTRL_H
#define SPEED_CTRL_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ---------- 速度环 PI 参数 ----------
 * 
 * 参考工程 kp=0.5, ki=0.4（duty 0~4000），
 * 本工程 duty 单位是百分比（0~100），且速度单位为 m/s。
 * ----------------------------------- */
#define SPD_KP  150.0f    /* 比例系数 */
#define SPD_KI  5.0f     /* 积分系数 */

/* ---------- PWM 占空比限幅 ---------- */
#define PWM_DUTY_MAX   100.0f
#define PWM_DUTY_MIN  -100.0f

/* ---------- 循迹差速 P 增益 ---------- */
#define TRACK_KP  0.05f    /* 传感器误差 → 速度差（m/s） */

/* ---------- 左右目标速度（循迹环设置，速度环 ISR 消费） ---------- */
extern volatile float g_target_speed_L;   /* m/s */
extern volatile float g_target_speed_R;   /* m/s */

/* ---------- 速度环使能标志（TURN_SPIN 期间关闭，避免与直接 PWM 控制冲突） ---------- */
extern volatile uint8_t g_speed_ctrl_enabled;

/* ---------- API ---------- */
void SpeedCtrl_Init(void);
void SpeedCtrl_Reset(void);

/* 速度环更新（50ms 定时器 ISR 中调用） */
void SpeedCtrl_Update(void);

/* 循迹环接口：设置差速目标 */
void Tracking_SpeedLoop(int16_t sensor_error, float base_speed_mmps);

#endif /* SPEED_CTRL_H */
