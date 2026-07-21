/**
 * @file    encoder.c
 * @brief   编码器脉冲计数与速度计算（简化版，对标10_DC_MOTOR_PID_3工程）
 *
 * 测量原理：
 *   - GPIO 中断累加 A 相上升沿脉冲数
 *   - 50ms 定时器周期性地将脉冲数换算为速度（mm/s），然后清零计数器
 *
 * 速度公式（与参考工程一致）：
 *   speed = counter / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 20
 *   其中 *20 是将 50ms 采样值换算到 1s（1 / 0.05 = 20）
 */

#include "encoder.h"

/* ---- 全局脉冲计数器（GPIO 中断累加，定时器 ISR 清零） ---- */
volatile uint32_t counter_R_A = 0;
volatile uint32_t counter_L_A = 0;

/* ---- 内部速度缓存（mm/s） ---- */
static float speed_L = 0.0f;
static float speed_R = 0.0f;

/* ================================================================
 * Encoder_Init
 *
 * GPIO 中断已在 SysConfig 生成的 SYSCFG_DL_init() 中配置完毕，
 * 这里不需要额外操作。
 * ================================================================ */
void Encoder_Init(void)
{
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(GPIO_EncoderB_INT_IRQN);
}

/* ================================================================
 * Encodering
 *
 * 由 GROUP1_IRQHandler 调用。
 * 仅做脉冲累加，不做任何速度计算。
 * ================================================================ */
void Encodering(void)
{
    uint32_t status;

    /* ---- 左编码器（PA12） ---- */
    status = DL_GPIO_getEnabledInterruptStatus(GPIO_EncoderA_PORT,
                                                GPIO_EncoderA_PIN_0_PIN);
    if (status & GPIO_EncoderA_PIN_0_PIN) {
        counter_L_A++;
    }
    DL_GPIO_clearInterruptStatus(GPIO_EncoderA_PORT,
                                  GPIO_EncoderA_PIN_0_PIN);

    /* ---- 右编码器（PB22） ---- */
    status = DL_GPIO_getEnabledInterruptStatus(GPIO_EncoderB_PORT,
                                                GPIO_EncoderB_PIN_2_PIN);
    if (status & GPIO_EncoderB_PIN_2_PIN) {
        counter_R_A++;
    }
    DL_GPIO_clearInterruptStatus(GPIO_EncoderB_PORT,
                                  GPIO_EncoderB_PIN_2_PIN);
}

/* ================================================================
 * Encoder_UpdateSpeeds
 *
 * 由 50ms 定时器 ISR 调用。
 * 将累计脉冲换算为速度（m/s），然后清零计数器。
 *
 * 速度公式：
 *   speed(m/s) = (pulses / 360) * PI * 0.065 * 20
 * ================================================================ */
void Encoder_UpdateSpeeds(void)
{
    speed_R = (float)counter_R_A / MOTOR_BIANMAQI
              * PI * MOTOR_WHEEL_D / 1000.0f * 20.0f;
    counter_R_A = 0;

    speed_L = (float)counter_L_A / MOTOR_BIANMAQI
              * PI * MOTOR_WHEEL_D / 1000.0f * 20.0f;
    counter_L_A = 0;
}

/* ================================================================
 * GetSpeed_L / GetSpeed_R
 *
 * 返回最近一次 Encoder_UpdateSpeeds() 计算的速度（mm/s）。
 * 注意：这两个函数不主动刷新速度，速度由定时器 ISR 定期更新。
 * 如果在 ISR 之外读取，得到的是上一周期的缓存值。
 * ================================================================ */
float GetSpeed_L(void)
{
    return speed_L;
}

float GetSpeed_R(void)
{
    return speed_R;
}
