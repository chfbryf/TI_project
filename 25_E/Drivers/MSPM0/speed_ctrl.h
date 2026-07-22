/**
 * @file    speed_ctrl.h
 * @brief   双路位置式PI速度控制器
 *
 * 速度环：位置式 PI  →  duty = Kp * error + Ki * integral
 * 循迹环：PID 控制器  →  差速目标 = base ± (Kp*error + Ki*integral + Kd*Δerror)
 */

#ifndef SPEED_CTRL_H
#define SPEED_CTRL_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ---------- 速度环 PI 参数 ----------
 * 
 * 位置式 PI：duty = Kp * error + Ki * integral
 * - Kp：误差→占空比直接映射，1m/s误差 ≈ Kp% 占空比
 * - Ki：稳态误差收敛速度
 * duty 单位：百分比 [-100, +100]，速度单位：m/s
 * ----------------------------------- */
#define SPD_KP  20.0f     /* 比例系数 */
#define SPD_KI  5.0f     /* 积分系数 */

/* ---------- 积分限幅（防饱和回弹） ---------- */
#define INTEGRAL_MAX  10.0f

/* ---------- PWM 占空比限幅 ---------- */
#define PWM_DUTY_MAX   100.0f
#define PWM_DUTY_MIN  -100.0f

/* ---------- 循迹差速 PID 增益 ---------- */
#define TRACK_KP  0.05f    /* 传感器误差 → 速度差（m/s） */
#define TRACK_KI  0.0008f   /* 误差积分 → 速度差（m/s） */
#define TRACK_KD  0.205f   /* 误差变化率 → 速度差（m/s） */
#define TRACK_INTEGRAL_MAX  50.0f  /* 循迹积分限幅 */

/* ---------- 左右目标速度（循迹环设置，速度环 ISR 消费） ---------- */
extern volatile float g_target_speed_L;   /* m/s */
extern volatile float g_target_speed_R;   /* m/s */

/* ---------- 速度环使能标志（TURN_SPIN 期间关闭，避免与直接 PWM 控制冲突） ---------- */
extern volatile uint8_t g_speed_ctrl_enabled;

/* ---------- API ---------- */
void SpeedCtrl_Init(void);
void SpeedCtrl_Reset(void);

/* 速度环更新（50ms 定时器 ISR 中调用） */ 
void SpeedCtrl_Update(float target_L, float target_R);

/* 循迹环接口：设置差速目标 */
void Tracking_SpeedLoop(int16_t sensor_error, float base_speed_mmps);
void Tracking_SpeedLoop_Reset(void);

#endif /* SPEED_CTRL_H */
