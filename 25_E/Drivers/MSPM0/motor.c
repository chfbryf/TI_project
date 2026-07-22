#include "motor.h"
#include "math.h"
#include "stdint.h"
#include "sys.h"

#define PERIOD 3200         // PWM周期（ARR值）


/**
 * @brief 设置左电机PWM占空比（BIN1/BIN2 + PWMB=PA1=C0）
 * 
 * @param Duty PWM占空比（-100~+100），负值表示反转
 */
void App_PWM_Set_L(float Duty)
{
    if(Duty >  100) Duty = 100;
    if(Duty < -100) Duty = -100;

    if(Duty >= 0)
    {
        BIN2_High;
        BIN1_Low;
    }
    else
    {
        BIN1_High;
        BIN2_Low;
    }

    DL_TimerG_setCaptureCompareValue(PWM_0_INST, (uint16_t)(fabsf(Duty) / 100.0f * PERIOD), GPIO_PWM_0_C0_IDX);
}

/**
 * @brief 设置右电机PWM占空比（AIN1/AIN2 + PWMA=PA0=C1）
 * 
 * @param Duty PWM占空比（-100~+100），负值表示反转
 */
void App_PWM_Set_R(float Duty)
{
    if(Duty >  100) Duty = 100;
    if(Duty < -100) Duty = -100;

    if(Duty >= 0)
    {
        AIN2_High;
        AIN1_Low;
    }
    else
    {
        AIN1_High;
        AIN2_Low;
    }

    DL_TimerG_setCaptureCompareValue(PWM_0_INST, (uint16_t)(fabsf(Duty) / 100.0f * (PERIOD)), GPIO_PWM_0_C1_IDX);
}
