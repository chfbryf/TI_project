/**
 * @file    encoder.c
 * @brief   编码器脉冲计数与速度计算（简化版，对标10_DC_MOTOR_PID_3工程）
 *
 * 测量原理：
 *   - GPIO 中断累加 A 相上升沿脉冲数
 *   - 同时读取 B 相电平判断正反转（对标 STM32 工程）
 *   - 50ms 定时器周期性地用差值法将脉冲增量换算为速度（m/s）
 *   - 差值法：delta = 本次计数器值 - 上次计数器值，避免清零造成的脉冲丢失
 *
 * 方向判断（A 相上升沿 + B 相电平）：
 *   A == B → 正转（计数器 +1）
 *   A != B → 反转（计数器 -1）
 *
 * 速度公式（与参考工程一致）：
 *   speed = counter / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D / 1000 * 20
 *   其中 *20 是将 50ms 采样值换算到 1s（1 / 0.05 = 20）
 */

#include "encoder.h"

/* ---- 全局脉冲计数器（GPIO 中断累加，定时器 ISR 清零） ---- */
volatile int32_t counter_R_A = 0;
volatile int32_t counter_L_A = 0;

/* ---- 内部速度缓存（m/s） ---- */
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

    /* ---- 左编码器（PA12: A相, PA13: B相） ---- */
    status = DL_GPIO_getEnabledInterruptStatus(GPIO_EncoderA_PORT,
                                                GPIO_EncoderA_PIN_0_PIN);
    if (status & GPIO_EncoderA_PIN_0_PIN) {
        /* 读取 A 相与 B 相当前电平，判断方向 */
        uint32_t a = DL_GPIO_readPins(GPIO_EncoderA_PORT, GPIO_EncoderA_PIN_0_PIN);
        uint32_t b = DL_GPIO_readPins(GPIO_EncoderA_PORT, GPIO_EncoderA_PIN_1_PIN);

        if ((a && !b) || (!a && b)) {
            /* A != B → 正转 */
            counter_L_A++;
        } else {
            /* A == B → 反转 */
            counter_L_A--;
        }
    }
    DL_GPIO_clearInterruptStatus(GPIO_EncoderA_PORT,
                                  GPIO_EncoderA_PIN_0_PIN);

    /* ---- 右编码器（PB22: A相, PB23: B相） ---- */
    status = DL_GPIO_getEnabledInterruptStatus(GPIO_EncoderB_PORT,
                                                GPIO_EncoderB_PIN_2_PIN);
    if (status & GPIO_EncoderB_PIN_2_PIN) {
        /* 读取 A 相与 B 相当前电平，判断方向 */
        uint32_t a = DL_GPIO_readPins(GPIO_EncoderB_PORT, GPIO_EncoderB_PIN_2_PIN);
        uint32_t b = DL_GPIO_readPins(GPIO_EncoderB_PORT, GPIO_EncoderB_PIN_3_PIN);

        if ((a && !b) || (!a && b)) {
            /* A != B → 反转 */
            counter_R_A--;
        } else {
            /* A == B → 正转 */
            counter_R_A++;
        }
    }
    DL_GPIO_clearInterruptStatus(GPIO_EncoderB_PORT,
                                  GPIO_EncoderB_PIN_2_PIN);
}

/* ================================================================
 * Encoder_UpdateSpeeds
 *
 * 由 50ms 定时器 ISR 调用。
 * 差值法：用本次计数器值 - 上次计数器值得到本周期脉冲增量。
 * counter 由 GPIO 中断只写不读，定时器 ISR 只读不写，无竞态。
 *
 * 丢脉冲保护：当 delta==0 但上一周期有明显速度（电机不应静止），
 * 沿用上次有效速度，避免因传感器 I2C 抢占导致的速度误报0。
 *
 * 速度公式：
 *   speed(m/s) = (pulses / 360) * PI * 0.065 * 20
 * ================================================================ */
void Encoder_UpdateSpeeds(void)
{
    static int32_t prev_R = 0;
    static int32_t prev_L = 0;
    static float   last_valid_speed_L = 0.0f;
    static float   last_valid_speed_R = 0.0f;

    int32_t delta_R = counter_R_A - prev_R;
    int32_t delta_L = counter_L_A - prev_L;

    prev_R = counter_R_A;
    prev_L = counter_L_A;

    speed_R = (float)delta_R / MOTOR_BIANMAQI
              * PI * MOTOR_WHEEL_D / 1000.0f * 20.0f;
    speed_L = (float)delta_L / MOTOR_BIANMAQI
              * PI * MOTOR_WHEEL_D / 1000.0f * 20.0f;

    /* 丢脉冲保护：delta==0 但上次速度 > 0.05m/s，沿用上次有效值 */
    if (delta_L == 0 && last_valid_speed_L > 0.05f) {
        speed_L = last_valid_speed_L;
    } else {
        last_valid_speed_L = speed_L;
    }

    if (delta_R == 0 && last_valid_speed_R > 0.05f) {
        speed_R = last_valid_speed_R;
    } else {
        last_valid_speed_R = speed_R;
    }
}

/* ================================================================
 * GetSpeed_L / GetSpeed_R
 *
 * 返回最近一次 Encoder_UpdateSpeeds() 计算的速度（m/s）。
 * 正值为正转，负值为反转。
 * ================================================================ */
float GetSpeed_L(void)
{
    return speed_L;
}

float GetSpeed_R(void)
{
    return speed_R;
}
