/**
 * @file    step_track.c
 * @brief   步进电机循迹控制实现
 *
 * IR 灰度传感器误差 → PD 差速计算 → 步进电机直驱
 */

#include "step_track.h"
#include "step_motor.h"
#include "sensor2.h"
#include "key.h"
#include "sys.h"
#include <math.h>

/* ---------- 内部状态 ---------- */
static int16_t last_error;  /* 上一拍误差（D项用） */

/* ---------- 辅助函数 ---------- */

/**
 * @brief 线性速度转步进电机角速度
 */
static float mmps_to_dps(float mmps)
{
    return (mmps * 360.0f) / (3.1415926f * WHEEL_DIAMETER_MM);
}

/* ---------- 公开接口 ---------- */

void StepTrack_Init(void)
{
    last_error = 0;
}

void StepTrack_Stop(void)
{
    step_motor_stop(1);
    step_motor_stop(2);
    last_error = 0;
}

/**
 * @brief 循迹控制：IR 误差 → 步进电机差速直驱
 *
 * 每轮主循环调用一次。根据按键启停状态：
 * - 启动：读取 IR 误差 → PD 计算左右差速 → 驱动电机
 * - 停止：两电机停转
 */
void StepTrack_Run(void)
{
    if (!key.start) {
        step_motor_continuous_run(1, 0.0f);
        step_motor_continuous_run(2, 0.0f);
        last_error = 0;
        return;
    }

    speed(key.keyspeed);
    float base_dps = mmps_to_dps((float)base_speed);

    /* 直行测试 */
    step_motor_continuous_run(1, base_dps);
    step_motor_continuous_run(2, base_dps);
#if 0
    int16_t error = Err2();
    float diff = TRACK_KP * base_dps * (error / 7.0f)
               + TRACK_KD * base_dps * ((error - last_error) / 7.0f);
    last_error = error;

    step_motor_continuous_run(1, base_dps - diff);
    step_motor_continuous_run(2, base_dps + diff);
#endif
}
