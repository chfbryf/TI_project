/**
 * @file    motor.c
 * @brief   电机PWM驱动
 */

#include "sys.h"

#define PERIOD 3200         /* PWM周期（ARR值） */

/* ================================================================
 * set_motor_pwm（内部辅助函数）
 *
 * 设置 PWM 占空比。左右轮共用，避免代码重复。
 * ================================================================ */
static void set_motor_pwm(float duty, uint8_t pwm_idx)
{
    if (duty >  100) duty = 100;
    if (duty < -100) duty = -100;
    DL_TimerG_setCaptureCompareValue(PWM_0_INST,
        (uint16_t)(fabsf(duty) / 100.0f * PERIOD), pwm_idx);
}

/**
 * @brief 设置左电机PWM占空比（BIN1/BIN2 + PWMB=PA1=C0）
 * @param Duty PWM占空比（-100~+100），负值表示反转
 */
void App_PWM_Set_L(float Duty)
{
    if (Duty >= 0) { BIN2_High; BIN1_Low; }
    else           { BIN1_High; BIN2_Low; }
    set_motor_pwm(Duty, GPIO_PWM_0_C0_IDX);
}

/**
 * @brief 设置右电机PWM占空比（AIN1/AIN2 + PWMA=PA0=C1）
 * @param Duty PWM占空比（-100~+100），负值表示反转
 */
void App_PWM_Set_R(float Duty)
{
    if (Duty >= 0) { AIN2_High; AIN1_Low; }
    else           { AIN1_High; AIN2_Low; }
    set_motor_pwm(Duty, GPIO_PWM_0_C1_IDX);
}

/**
 * @brief 急停：清零目标速度、清零积分、关闭 PWM 输出
 */
void motor_stop(void)
{
    base_speed = 0;
    g_target_speed_L = 0.0f;
    g_target_speed_R = 0.0f;
    SpeedCtrl_Reset();
    Tracking_SpeedLoop_Reset();
    App_PWM_Set_L(0.0f);
    App_PWM_Set_R(0.0f);
}
