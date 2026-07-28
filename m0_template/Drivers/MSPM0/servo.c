#include "servo.h"
#include "sys.h"

/*
 * 舵机脉宽校准（BUSCLK=80MHz, Divider=8, Prescale=249 → 40kHz, 每 count=25us）
 * 改这两个值即可匹配不同舵机：
 *   MIN → 0° 脉宽,  MAX → 270° 脉宽
 *   单位 us,  典型范围 200~3000
 */
#define SERVO_MIN_US    500    /* 0° 脉宽 (us) */
#define SERVO_MAX_US    2500   /* 270° 脉宽 (us) */
#define SERVO_MIN_COUNT (SERVO_MIN_US / 25)
#define SERVO_MAX_COUNT (SERVO_MAX_US / 25)

static unsigned int Servo_Angle[2] = {0, 0};  /* 两路舵机当前角度 */

/* CC 索引表，对应 SERVO_CH0 / SERVO_CH1 */
static const uint8_t servo_cc_idx[2] = {
    GPIO_SERVO_C0_IDX,  /* CH0: TIMA0_CCP0 */
    GPIO_SERVO_C1_IDX,  /* CH1: TIMA0_CCP1 */
};

/*
 * 通道：SERVO_CH0 = PB14, SERVO_CH1 = PA3，共用 TIMA0
 * 角度范围：0 ~ 270°
 */

/* 启动舵机定时器，两路归中到 50° */
void Servo_Init(void)
{
    DL_TimerG_startCounter(SERVO_INST);
    Set_Servo_Angle(SERVO_CH0, 50);
    Set_Servo_Angle(SERVO_CH1, 50);
}

/* ch: 通道 SERVO_CH0 / SERVO_CH1, angle: 0~270 */
void Set_Servo_Angle(uint8_t ch, unsigned int angle)
{
    if (ch > 1) return;

    if (angle > 270) {
        angle = 270;
    }

    Servo_Angle[ch] = angle;

    /* 内部映射：0~270° → MIN~MAX counts */
    float min_count = (float)SERVO_MIN_COUNT;
    float max_count = (float)SERVO_MAX_COUNT;
    float range = max_count - min_count;
    float cmp = min_count + (((float)angle / 270.0f) * range);

    DL_TimerG_setCaptureCompareValue(SERVO_INST,
        (unsigned int)(cmp + 0.5f), servo_cc_idx[ch]);
}

/* ch: 通道, 返回当前角度 0~270 */
unsigned int Get_Servo_Angle(uint8_t ch)
{
    if (ch > 1) return 0;
    return Servo_Angle[ch];
}
