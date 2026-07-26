/**
 * @file    speed_ctrl.c
 * @brief   双路位置式PI速度控制器
 *
 * 算法（位置式 PI）：
 *   integral += error
 *   duty = Kp * error + Ki * integral    （带抗积分饱和）
 *
 * 架构：
 *   循迹环（main 主循环） → 设置 g_target_speed_L/R
 *   速度环（SPEED_PID 定时器 ISR @ 50ms） → SpeedCtrl_Update() 消费目标、输出 PWM
 */

#include "sys.h"

/* ---- 目标速度（循迹环写入，速度环 ISR 读取） ---- */
volatile float g_target_speed_L = 0.0f;
volatile float g_target_speed_R = 0.0f;

/* ---- 左右独立 PI 状态 ---- */
static float integral_L = 0.0f;
static float integral_R = 0.0f;

/* ---- 循迹环 PID 状态 ---- */
static float track_integral = 0.0f;
static int16_t prev_error = 0;

/* ---- 速度环全局使能 ---- */
volatile uint8_t g_speed_ctrl_enabled = 1;

/* ================================================================
 * PI_Update（内部辅助函数）
 *
 * 位置式 PI + 抗积分饱和 + 限幅。
 * 左右轮共用，避免代码重复。
 * ================================================================ */
static float PI_Update(float target, float actual, float *integral)
{
    float error = target - actual;
    float duty  = SPD_KP * error + SPD_KI * (*integral);

    /* 抗积分饱和：仅在未饱和时累加积分 */
    if (duty > PWM_DUTY_MAX) {
        if (error < 0.0f) *integral += error;
    } else if (duty < PWM_DUTY_MIN) {
        if (error > 0.0f) *integral += error;
    } else {
        *integral += error;
    }

    /* 输出限幅 */
    if (duty > PWM_DUTY_MAX)  duty = PWM_DUTY_MAX;
    if (duty < PWM_DUTY_MIN)  duty = PWM_DUTY_MIN;

    /* 积分限幅 */
    if (*integral >  INTEGRAL_MAX) *integral =  INTEGRAL_MAX;
    if (*integral < -INTEGRAL_MAX) *integral = -INTEGRAL_MAX;

    return duty;
}

/* ================================================================
 * SpeedCtrl_Init
 *
 * 在 main() 初始化阶段调用一次。
 * ================================================================ */
void SpeedCtrl_Init(void)
{
    integral_L = 0.0f;
    integral_R = 0.0f;
    g_speed_ctrl_enabled = 1;
}

/* ================================================================
 * SpeedCtrl_Reset
 *
 * 清零积分，转弯 / 停车 / 模式切换时调用。
 * ================================================================ */
void SpeedCtrl_Reset(void)
{
    integral_L = 0.0f;
    integral_R = 0.0f;
}

/* ================================================================
 * SpeedCtrl_Update
 *
 * 由 SPEED_PID 定时器 ISR（50ms）调用。
 * ================================================================ */
void SpeedCtrl_Update(float target_L, float target_R)
{
    if (!g_speed_ctrl_enabled) return;

    App_PWM_Set_L(PI_Update(target_L, GetSpeed_L(), &integral_L));
    App_PWM_Set_R(PI_Update(target_R, GetSpeed_R(), &integral_R));
}

/* ================================================================
 * Tracking_SpeedLoop_Reset
 *
 * 清零循迹误差历史值，转弯 / 停车 / 模式切换时调用。
 * ================================================================ */
void Tracking_SpeedLoop_Reset(void)
{
    track_integral = 0.0f;
    prev_error = 0;
}

/* ================================================================
 * Tracking_SpeedLoop
 *
 * 循迹环接口。
 *   根据灰度传感器误差（-5 ~ +5）计算左右轮速度差，
 *   写入 g_target_speed_L/R，由 50ms 定时器 ISR 消费。
 *
 * @param sensor_error  灰度传感器误差（Err2() 返回值，-5 ~ +5）
 * @param base_speed_mmps  基准速度，单位 mm/s
 * ================================================================ */
void Tracking_SpeedLoop(int16_t sensor_error, float base_speed_mmps)
{
    float diff;
    float base  = base_speed_mmps / 1000.0f;  /* mm/s → m/s */

    /* PID 循迹：P 快速响应 + I 消除稳态 + D 抑制过冲 */
    track_integral += (float)sensor_error;

    /* 积分限幅（防饱和） */
    if (track_integral >  TRACK_INTEGRAL_MAX) track_integral =  TRACK_INTEGRAL_MAX;
    if (track_integral < -TRACK_INTEGRAL_MAX) track_integral = -TRACK_INTEGRAL_MAX;

    diff = (float)sensor_error * TRACK_KP
         + track_integral * TRACK_KI
         + (float)(sensor_error - prev_error) * TRACK_KD;

    prev_error = sensor_error;

    g_target_speed_L = base + diff;
    g_target_speed_R = base - diff;

    /* 下限钳位：不输出负目标速度 */
    if (g_target_speed_L < 0.0f) g_target_speed_L = 0.0f;
    if (g_target_speed_R < 0.0f) g_target_speed_R = 0.0f;
}
