#include "servo.h"
#include "sys.h"

static unsigned int Servo_Angle[2] = {0, 0};  /* 两路舵机当前角度 */

/* CC 索引表，对应 SERVO_CH0 / SERVO_CH1 */
static const uint8_t servo_cc_idx[2] = {
    GPIO_SERVO_C0_IDX,  /* CH0: TIMA0_CCP0 */
    GPIO_SERVO_C1_IDX,  /* CH1: TIMA0_CCP1 */
};

/*
 * 270度舵机 PWM 参数：
 *   周期 = 20ms (50Hz)
 *   0.5ms 高电平 → 0°
 *   2.5ms 高电平 → 270°
 *
 * SysConfig 配置（模块名 SERVO，定时器 TIMA0）：
 *   时钟源 = BUSCLK (40MHz)
 *   Divider  = 8    → 5MHz
 *   Prescale = 250  → 20kHz（每计数 = 50us）
 *   Counter  = 400  → 50Hz 周期
 *
 *   0°  → 0.5ms / 50us = 10  counts
 *   270°→ 2.5ms / 50us = 50  counts
 *
 * 通道分配：
 *   CH0 (C0): PB14, TIMA0_CCP0
 *   CH1 (C1): PA3,  TIMA0_CCP1
 */

void Servo_Init(void)
{
    DL_TimerG_startCounter(SERVO_INST);
    Set_Servo_Angle(SERVO_CH0, 50);
    Set_Servo_Angle(SERVO_CH1, 50);
}

void Set_Servo_Angle(uint8_t ch, unsigned int angle)
{
    if (ch > 1) return;

    if (angle > 270) {
        angle = 270;
    }

    Servo_Angle[ch] = angle;

    /* 线性映射：0~270° → 10~50 counts */
    float min_count = 10.0f;
    float max_count = 50.0f;
    float range = max_count - min_count;
    float cmp = min_count + (((float)angle / 270.0f) * range);

    DL_TimerG_setCaptureCompareValue(SERVO_INST,
        (unsigned int)(cmp + 0.5f), servo_cc_idx[ch]);
}

unsigned int Get_Servo_Angle(uint8_t ch)
{
    if (ch > 1) return 0;
    return Servo_Angle[ch];
}
