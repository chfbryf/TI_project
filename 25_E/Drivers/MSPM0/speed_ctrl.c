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

#include "speed_ctrl.h"
#include "motor.h"
#include "encoder.h"

/* ---- 目标速度（循迹环写入，速度环 ISR 读取） ---- */
volatile float g_target_speed_L = 0.0f;
volatile float g_target_speed_R = 0.0f;

/* ---- 左右独立 PI 状态 ---- */
static float integral_L = 0.0f;
static float integral_R = 0.0f;

/* ---- 速度环全局使能（TURN_SPIN 时由 main 置 0，禁止 ISR 写 PWM） ---- */
volatile uint8_t g_speed_ctrl_enabled = 1;

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
 * 位置式 PI + 抗积分饱和。
 * ================================================================ */
void SpeedCtrl_Update(float target_L, float target_R)
{
    float error, duty;

    if (!g_speed_ctrl_enabled) return;

    /* ---- 左轮位置式 PI ---- */
    error = target_L - GetSpeed_L();

    /* 抗积分饱和：仅在未饱和时累加积分 */
    if ((duty = SPD_KP * error + SPD_KI * integral_L) > PWM_DUTY_MAX) {
        if (error < 0.0f) integral_L += error;   /* 超速时允许减积分 */
    } else if (duty < PWM_DUTY_MIN) {
        if (error > 0.0f) integral_L += error;   /* 欠速时允许加积分 */
    } else {
        integral_L += error;
    }

    /* 限幅 */
    if (duty > PWM_DUTY_MAX)  duty = PWM_DUTY_MAX;
    if (duty < PWM_DUTY_MIN)  duty = PWM_DUTY_MIN;

    /* 积分限幅（防止长时间饱和后回弹过大） */
    if (integral_L >  INTEGRAL_MAX) integral_L =  INTEGRAL_MAX;
    if (integral_L < -INTEGRAL_MAX) integral_L = -INTEGRAL_MAX;

    App_PWM_Set_L(duty);

    /* ---- 右轮位置式 PI ---- */
    error = target_R - GetSpeed_R();

    if ((duty = SPD_KP * error + SPD_KI * integral_R) > PWM_DUTY_MAX) {
        if (error < 0.0f) integral_R += error;
    } else if (duty < PWM_DUTY_MIN) {
        if (error > 0.0f) integral_R += error;
    } else {
        integral_R += error;
    }

    if (duty > PWM_DUTY_MAX)  duty = PWM_DUTY_MAX;
    if (duty < PWM_DUTY_MIN)  duty = PWM_DUTY_MIN;

    if (integral_R >  INTEGRAL_MAX) integral_R =  INTEGRAL_MAX;
    if (integral_R < -INTEGRAL_MAX) integral_R = -INTEGRAL_MAX;

    App_PWM_Set_R(duty);
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
    float diff  = (float)sensor_error * TRACK_KP;
    float base  = base_speed_mmps / 1000.0f;  /* mm/s → m/s */

    g_target_speed_L = base + diff;
    g_target_speed_R = base - diff;

    /* 下限钳位：不输出负目标速度（由循迹逻辑决定倒车） */
    if (g_target_speed_L < 0.0f) g_target_speed_L = 0.0f;
    if (g_target_speed_R < 0.0f) g_target_speed_R = 0.0f;
}
