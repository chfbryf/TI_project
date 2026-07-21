/**
 * @file    gyro.c
 * @brief   基于 MPU6050 yaw 的角度 PID 控制器
 *
 * 算法：位置式 PID
 *   output = Kp * error + Ki * integral + Kd * derivative
 *
 * 角度误差自动处理 -180° ~ +180° 的跨越问题。
 */

#include "gyro.h"
#include "mpu6050.h"
#include <math.h>

/* ---- PID 内部状态 ---- */
static float target;         /* 目标角度（度） */
static float integral;       /* 积分累加 */
static float last_error;     /* 上一次误差，用于微分 */

/* ================================================================
 * Gyro_Init
 * ================================================================ */
void Gyro_Init(void)
{
    target     = 0.0f;
    integral   = 0.0f;
    last_error = 0.0f;
}

/* ================================================================
 * Gyro_SetTarget
 *
 * @param target_yaw_deg  目标角度（度，范围 -180 ~ +180）
 * ================================================================ */
void Gyro_SetTarget(float target_yaw_deg)
{
    target = target_yaw_deg;
}

/* ================================================================
 * Gyro_GetCurrent
 *
 * @return 当前 MPU6050 yaw 角（度）
 * ================================================================ */
float Gyro_GetCurrent(void)
{
    return yaw;   /* mpu6050.c 全局变量，由 Read_Quad() 更新 */
}

/* ================================================================
 * angle_error  内部辅助：计算最短角度误差
 *
 * 例如 target=170°, current=-170° → 误差 = -20°（不是 340°）
 * ================================================================ */
static float angle_error(float target_deg, float current_deg)
{
    float err = target_deg - current_deg;
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

/* ================================================================
 * Gyro_Update
 *
 * 每次调用执行一次位置式 PID 计算。
 * 调用频率建议与循迹环一致（如每 6ms），由调用方保证。
 *
 * @return 差速值（m/s），正值=右转，负值=左转
 * ================================================================ */
float Gyro_Update(void)
{
    float error, derivative, output;

    error = angle_error(target, yaw);

    /* 积分（带限幅防饱和） */
    integral += error;
    if (integral >  100.0f) integral =  100.0f;
    if (integral < -100.0f) integral = -100.0f;

    /* 微分 */
    derivative = error - last_error;
    last_error = error;

    /* PID 输出 */
    output = GYRO_KP * error
           + GYRO_KI * integral
           + GYRO_KD * derivative;

    /* 输出限幅 */
    if (output >  GYRO_OUTPUT_MAX) output =  GYRO_OUTPUT_MAX;
    if (output < -GYRO_OUTPUT_MAX) output = -GYRO_OUTPUT_MAX;

    return output;
}

/* ================================================================
 * Gyro_IsDone
 *
 * 返回 1 表示当前角度在目标角度的 tolerance 范围内。
 * ================================================================ */
uint8_t Gyro_IsDone(float tolerance_deg)
{
    float err = angle_error(target, yaw);
    return (fabsf(err) <= tolerance_deg) ? 1 : 0;
}

/* ================================================================
 * Gyro_Reset
 *
 * 清除积分和误差历史（切换目标时调用，避免状态残留）。
 * ================================================================ */
void Gyro_Reset(void)
{
    integral   = 0.0f;
    last_error = 0.0f;
}
