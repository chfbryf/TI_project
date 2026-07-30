/**
 * @file    step_track.h
 * @brief   步进电机循迹控制模块
 *
 * IR 灰度传感器误差 → P 差速计算 → 步进电机直驱
 * 左轮 = 电机1, 右轮 = 电机2
 * 误差 > 0（偏右）→ 左轮加速/右轮减速，向左修正
 */

#ifndef STEP_TRACK_H
#define STEP_TRACK_H

#include "ti_msp_dl_config.h"

/* ---------- 机械参数 ---------- */
#define WHEEL_DIAMETER_MM   65.0f   /* 轮径 mm */

/* ---------- PD 增益 ---------- */
#define TRACK_KP            0.3700f    /* P 增益（误差归一化后占基础速度的比例） */

void StepTrack_Init(void);
void StepTrack_Stop(void);
void StepTrack_Run(void);

#endif /* STEP_TRACK_H */
