#include "servo.h"
#include "sys.h"

static unsigned int Servo_Angle = 0;  // 当前舵机角度

void Servo_Init(void)
{
    DL_TimerG_startCounter(SERVO_INST);
    Set_Servo_Angle(50);
}

/*
 * SG90 舵机 PWM 参数：
 *   周期 = 20ms (50Hz)
 *   0.5ms 高电平 → 0°
 *   2.5ms 高电平 → 180°
 *
 * SysConfig 配置（模块名 SERVO，定时器 TIMA0）：
 *   时钟源 = BUSCLK (40MHz)
 *   Divider  = 8    → 5MHz
 *   Prescale = 250  → 20kHz（每计数 = 50us）
 *   Counter  = 400  → 50Hz 周期
 *
 *   0°  → 0.5ms / 50us = 10  counts
 *   180°→ 2.5ms / 50us = 50  counts
 */

void Set_Servo_Angle(unsigned int angle)
{
    if (angle > 180) {
        angle = 180;
    }

    Servo_Angle = angle;

    // 线性映射：0~180° → 10~50 counts
    float min_count = 10.0f;
    float max_count = 50.0f;
    float range = max_count - min_count;
    float cmp = min_count + (((float)angle / 180.0f) * range);

    DL_TimerG_setCaptureCompareValue(SERVO_INST,
        (unsigned int)(cmp + 0.5f), GPIO_SERVO_C0_IDX);
}

unsigned int Get_Servo_Angle(void)
{
    return Servo_Angle;
}
