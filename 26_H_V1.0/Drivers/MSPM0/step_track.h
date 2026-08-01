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

/* ---------- 任务速度 ---------- */
#define TRACK_SPEED_T1           160    /* 任务1 基础速度 mm/s */
#define TRACK_SPEED_T2_EARLY     200    /* 任务2 前期速度 mm/s (15s前) */
#define TRACK_SPEED_T2_LATE       90    /* 任务2 后期速度 mm/s (15s后) */
#define TRACK_SPEED_T34          110    /* 任务3/4 基础速度 mm/s */
#define TRACK_T2_SWITCH_MS     15000U   /* 任务2 变速时间点 ms */
#define TRACK_T2_DECEL_MS       1000U   /* 任务2 减速过渡时间 ms */

/* ---------- 循迹增益 ---------- */
#define TRACK_KP            0.316038f    /* P 增益 */
#define TRACK_KD            0.0f        /* D 增益: 阻尼抑制摆动 (0=纯P) */
#define TRACK_ERROR_EMA_ALPHA   0.8f     /* 误差EMA滤波: 越小越平滑 */
#define TRACK_MAX_DPS_DELTA   30.0f      /* 每帧最大速度变化 (deg/s/50ms) */

void StepTrack_Init(void);
void StepTrack_Stop(void);
void StepTrack_Run(void);
float StepTrack_GetRampSpeed(void);   /* 获取小车当前缓启速度 mm/s，供前馈补偿 */

#endif /* STEP_TRACK_H */
