/**
 * @file    cascade_pid.c
 * @brief   串级PID控制（位置环外环 + 速度环内环）
 *
 *  控制架构：
 *
 *   sensor_error ──→ [外环: 位置PID] ──→ 差速调整(rad/s)
 *                                               │
 *                          base_speed_rads ──→ ± ──→ 目标速度(rad/s)
 *                                                       │
 *                         GetSpeed_L/R() ──→ [内环: 速度PID] ──→ PWM → 电机
 *
 *  外环使用位置式PID，内环使用增量式PID（基于现有 pid.h 库）。
 *
 *  使用方式：
 *    1. 初始化：Cascade_PID_Init();
 *    2. 每6ms调用：Cascade_PID_Proc(err, base_speed_rads);
 */

#include "cascade.h"
#include "encoder.h"
#include "motor.h"
#include <math.h>


/*==============================================================================
 *                           内环：速度环 PID 对象
 *============================================================================*/

static PID_TypeDef pid_spd_L;   /* 左电机速度环 */
static PID_TypeDef pid_spd_R;   /* 右电机速度环 */


/*==============================================================================
 *                        外环：位置环 PID 静态变量
 *============================================================================*/

static float pos_bias       = 0.0f;   /* 本次偏差 */
static float pos_integral   = 0.0f;   /* 积分累加 */
static float pos_last_bias  = 0.0f;   /* 上次偏差 */
static float pos_output     = 0.0f;   /* 本次输出 */


/*==============================================================================
 *                           初始化
 *============================================================================*/

/**
 * @brief 串级PID初始化
 *
 * 初始化两个速度环PID控制器和位置环静态变量。
 * 在主循环开始前调用一次。
 */
void Cascade_PID_Init(void)
{
    /*-- 左电机速度环 --*/
    PID_Init_Simple(&pid_spd_L, SPD_KP, SPD_KI, SPD_KD);
    PID_LimitConfig(&pid_spd_L, SPD_OUT_MAX, SPD_OUT_MIN);

    /*-- 右电机速度环 --*/
    PID_Init_Simple(&pid_spd_R, SPD_KP, SPD_KI, SPD_KD);
    PID_LimitConfig(&pid_spd_R, SPD_OUT_MAX, SPD_OUT_MIN);

    /*-- 位置环静态变量清零 --*/
    pos_bias      = 0.0f;
    pos_integral  = 0.0f;
    pos_last_bias = 0.0f;
    pos_output    = 0.0f;
}

/**
 * @brief 复位串级PID所有积分和状态
 *
 * 在弯道退出、停车等需要清空积分残留时调用。
 */
void Cascade_PID_Reset(void)
{
    PID_Reset(&pid_spd_L);
    PID_Reset(&pid_spd_R);

    pos_bias      = 0.0f;
    pos_integral  = 0.0f;
    pos_last_bias = 0.0f;
    pos_output    = 0.0f;
}


/*==============================================================================
 *                    内环：速度PID 单次计算
 *============================================================================*/

/**
 * @brief 单次速度环PID计算 + PWM输出
 *
 * @param pid       速度PID对象指针
 * @param target    目标速度 (rad/s)
 * @param actual    实际速度 (rad/s)
 * @param set_pwm   设置PWM的回调函数指针
 */
static void Speed_Loop_Single(PID_TypeDef *pid, float target, float actual,
                              void (*set_pwm)(float))
{
    float voltage, duty;

    /* 更新目标值 */
    PID_ChangeSP(pid, target);

    /* PID计算，输出为电压 (V) */
    voltage = PID_Compute(pid, actual);

    /* 电压 → PWM占空比 (±100) */
    duty = voltage / 12.0f * 100.0f;

    /* 输出到电机 */
    set_pwm(duty);
}


/*==============================================================================
 *                    外环：位置式PID 差速计算
 *============================================================================*/

/**
 * @brief 位置式PID计算差速修正量
 *
 * 输入传感器偏差（例如 -5 ~ +5），输出差速修正量（±3200 范围）。
 * 公式：Output = Kp*Bias + Ki*∫Bias + Kd*(Bias - LastBias)
 *
 * @param error  传感器位置偏差
 * @return       差速修正量（±3200）
 */
static float Position_PID_Compute(int16_t error)
{
    pos_bias = (float)error;

    /* 积分累加，偏差为0时清零防止残留 */
    pos_integral += pos_bias;
    if (pos_bias == 0.0f) {
        pos_integral = 0.0f;
    }

    /* 位置式PID */
    pos_output = POS_KP * pos_bias
               + POS_KI * pos_integral
               + POS_KD * (pos_bias - pos_last_bias);

    pos_last_bias = pos_bias;

    /* 输出限幅 */
    if (pos_output > POS_OUT_MAX)  pos_output = POS_OUT_MAX;
    if (pos_output < POS_OUT_MIN)  pos_output = POS_OUT_MIN;

    return pos_output;
}


/*==============================================================================
 *                        主入口：串级PID 一次迭代
 *============================================================================*/

/**
 * @brief 串级PID单次迭代（每6ms调用一次）
 *
 * @param sensor_error      传感器位置偏差（Err2() 的返回值，范围约 -5 ~ +5）
 * @param base_speed_rads   直线目标速度 (rad/s)，例如 10.0f 表示 10 rad/s
 *
 * 内部流程：
 *   1. 外环位置PID → 差速修正量
 *   2. 差速量映射为 rad/s 速度调整
 *   3. 计算左右轮目标速度
 *   4. 内环速度PID → PWM输出
 */
void Cascade_PID_Proc(int16_t sensor_error, float base_speed_rads)
{
    float diff_pid;      /* 位置环差速输出 */
    float diff_rads;     /* 转换为 rad/s 的速度调整量 */
    float target_L;      /* 左轮目标速度 rad/s */
    float target_R;      /* 右轮目标速度 rad/s */
    float actual_L;      /* 左轮实际速度 rad/s */
    float actual_R;      /* 右轮实际速度 rad/s */

    /*---- 1. 外环：位置PID计算差速修正量 ----*/
    diff_pid = Position_PID_Compute(sensor_error);

    /*---- 2. 差速量 → rad/s 速度调整 ----*/
    diff_rads = diff_pid * DIFF_TO_SPEED;

    /*---- 3. 计算左右轮目标速度 ----*/
    /*
     *  左轮减速、右轮加速 → 右转
     *  左轮加速、右轮减速 → 左转
     *  差速量的符号由传感器偏差方向决定
     */
    target_L = base_speed_rads - diff_rads;
    target_R = base_speed_rads + diff_rads;

    /* 防止目标速度为负（倒车） */
    if (target_L < 0.0f) target_L = 0.0f;
    if (target_R < 0.0f) target_R = 0.0f;

    /*---- 4. 读取编码器实际速度 ----*/
    actual_L = GetSpeed_L();
    actual_R = GetSpeed_R();

    /*---- 5~6. 内环：速度PID → PWM输出 ----*/
    Speed_Loop_Single(&pid_spd_L, target_L, actual_L, App_PWM_Set_L);
    Speed_Loop_Single(&pid_spd_R, target_R, actual_R, App_PWM_Set_R);
}
