#ifndef _CASCADE_H
#define _CASCADE_H

#include "pid.h"

/*==============================================================================
 *                          串级PID 参数配置
 *============================================================================*/

/* 外环（位置环）PID 参数 */
#define POS_KP  200.0f
#define POS_KI  0.4f
#define POS_KD  365.0f

/* 外环输出限幅（差速修正量 ±3200） */
#define POS_OUT_MAX   3200
#define POS_OUT_MIN  -3200

/* 内环（速度环）PID 参数 */
#define SPD_KP  0.5f
#define SPD_KI  0.15f
#define SPD_KD  0.0f

/* 内环输出限幅（电压 V → 对应 PWM 占空比） */
#define SPD_OUT_MAX   12.0f
#define SPD_OUT_MIN  -12.0f

/*
 * 差速量 → 速度调整 映射系数
 *
 * 位置环满偏输出 ±3200 映射到 ±5.0 rad/s 的速度差，
 * 即最大转弯时左右轮速度差为 10 rad/s。
 * 可根据实际需求调整此系数。
 */
#define DIFF_TO_SPEED  (5.0f / 3200.0f)


/*==============================================================================
 *                          函数声明
 *============================================================================*/

void Cascade_PID_Init(void);
void Cascade_PID_Reset(void);
void Cascade_PID_Proc(int16_t sensor_error, float base_speed_rads);

#endif
