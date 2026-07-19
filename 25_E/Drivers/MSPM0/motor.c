#include "motor.h"
#include "pid.h"
#include "encoder.h"
#include "sys.h"
#include "math.h"
#include "clock.h"
#include "stdint.h"

#define PERIOD 3200         // PWM周期（ARR值）


PID_TypeDef pid_motor_L;    // 左电机速度环PID控制器
PID_TypeDef pid_motor_R;    // 右电机速度环PID控制器


/** 
 * @brief 左电机pid初始化 
 */
void Motor_Reset_L(void)
{
    PID_Reset(&pid_motor_L);
}


/** 
 * @brief 右电机pid初始化 
 */
void Motor_Reset_R(void)
{
    PID_Reset(&pid_motor_R);
}


/**
 * @brief 设置左电机PWM占空比
 * 
 * @param Duty PWM占空比（-100~+100），负值表示反转
 */
void App_PWM_Set_L(float Duty)
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

    DL_TimerG_setCaptureCompareValue(PWM_0_INST, (uint16_t)(fabsf(Duty) / 100.0f * (PERIOD)), GPIO_PWM_0_C0_IDX);

}

/**
 * @brief 设置右电机PWM占空比
 * 
 * @param Duty PWM占空比（-100~+100），负值表示反转
 */
void App_PWM_Set_R(float Duty)
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

    DL_TimerG_setCaptureCompareValue(PWM_0_INST, (uint16_t)(fabsf(Duty) / 100.0f * PERIOD), GPIO_PWM_0_C1_IDX);

}

/**
 * @brief 设置左电机目标角速度
 * 
 * @param Omega_L 目标角速度（单位：rad/s）
 */
void App_Motor_SetOmega_L(float Omega_L)
{
    PID_ChangeSP(&pid_motor_L, Omega_L);
}

/**
 * @brief 设置右电机目标角速度
 * 
 * @param Omega_R 目标角速度（单位：rad/s）
 */
void App_Motor_SetOmega_R(float Omega_R)
{
    PID_ChangeSP(&pid_motor_R, Omega_R);
}

/**
 * @brief 初始化电机调速系统
 */
void App_Motor_Init(void)
{
    PID_Init_Simple(&pid_motor_L, 1.5, 0.7, 0);
    PID_LimitConfig(&pid_motor_L, +12, -12);

    PID_Init_Simple(&pid_motor_R, 1.5, 0.7, 0);
    PID_LimitConfig(&pid_motor_R, +12, -12);

    /* 双电机共用一个定时器 PWM_0(TIMG8): C0=PA1(左), C1=PA0(右) 初始化为0%占空比 */
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, PERIOD, GPIO_PWM_0_C0_IDX);
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, PERIOD, GPIO_PWM_0_C1_IDX);
}

/**
 * @brief 电机速度闭环控制进程函数（10ms周期）
 */
void App_Motor_Proc(uint8_t flag)
{
    if (flag == 1)
    {
    float Omega_L = GetSpeed_L();
    float Omega_R = GetSpeed_R();

    float ua_L = PID_Compute(&pid_motor_L, Omega_L);
    float ua_R = PID_Compute(&pid_motor_R, Omega_R);

    float bat = 12;

    float duty_L = ua_L / bat * 100.0f;
    float duty_R = ua_R / bat * 100.0f;

    App_PWM_Set_L(duty_L);
    App_PWM_Set_R(duty_R);
    }
}
