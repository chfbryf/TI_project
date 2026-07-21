/**
 * @file    speed_ctrl.c
 * @brief   双路增量式PI速度控制器（对标10_DC_MOTOR_PID_3工程）
 *
 * 算法：
 *   PWM_duty += Kp * (error - last_error) + Ki * error
 *   （增量式 PI，与参考工程 DC_MOTOR_PID() 完全一致）
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
static float last_error_L = 0.0f;
static float last_error_R = 0.0f;
static float pwm_duty_L   = 0.0f;
static float pwm_duty_R   = 0.0f;

/* ================================================================
 * SpeedCtrl_Init
 *
 * 在 main() 初始化阶段调用一次。
 * ================================================================ */
void SpeedCtrl_Init(void)
{
    last_error_L = 0.0f;
    last_error_R = 0.0f;
    pwm_duty_L   = 0.0f;
    pwm_duty_R   = 0.0f;
    g_target_speed_L = 0.0f;
    g_target_speed_R = 0.0f;
}

/* ================================================================
 * SpeedCtrl_Reset
 *
 * 重置 PI 积分（转弯等场景下清除历史误差）。
 * ================================================================ */
void SpeedCtrl_Reset(void)
{
    last_error_L = 0.0f;
    last_error_R = 0.0f;
    pwm_duty_L   = 0.0f;
    pwm_duty_R   = 0.0f;
}

/* ================================================================
 * SpeedCtrl_Update
 *
 * 由 SPEED_PID 定时器 ISR（50ms）调用。
 * 对左右电机各执行一次增量式 PI 计算并输出 PWM。
 * ================================================================ */
void SpeedCtrl_Update(void)
{
    float error, current_error;

    /* ---- 左轮增量式 PI ---- */
    error         = g_target_speed_L - GetSpeed_L();
    current_error = error;
    pwm_duty_L   += SPD_KP * (current_error - last_error_L)
                  + SPD_KI * current_error;
    last_error_L  = current_error;

    /* 限幅 */
    if (pwm_duty_L > PWM_DUTY_MAX)  pwm_duty_L = PWM_DUTY_MAX;
    if (pwm_duty_L < PWM_DUTY_MIN)  pwm_duty_L = PWM_DUTY_MIN;

    App_PWM_Set_L(pwm_duty_L);

    /* ---- 右轮增量式 PI ---- */
    error         = g_target_speed_R - GetSpeed_R();
    current_error = error;
    pwm_duty_R   += SPD_KP * (current_error - last_error_R)
                  + SPD_KI * current_error;
    last_error_R  = current_error;

    /* 限幅 */
    if (pwm_duty_R > PWM_DUTY_MAX)  pwm_duty_R = PWM_DUTY_MAX;
    if (pwm_duty_R < PWM_DUTY_MIN)  pwm_duty_R = PWM_DUTY_MIN;

    App_PWM_Set_R(pwm_duty_R);
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
    float diff = (float)sensor_error * TRACK_KP;

    g_target_speed_L = base_speed_mmps + diff;
    g_target_speed_R = base_speed_mmps - diff;

    /* 下限钳位：不输出负目标速度（由循迹逻辑决定倒车） */
    if (g_target_speed_L < 0.0f) g_target_speed_L = 0.0f;
    if (g_target_speed_R < 0.0f) g_target_speed_R = 0.0f;
}
