/**
 * @file    step_track.c
 * @brief   步进电机循迹控制实现
 *
 * IR 灰度传感器误差 → P 差速计算 → 步进电机直驱
 */

#include "step_track.h"
#include "step_motor.h"
#include "sensor2.h"
#include "key.h"
#include "sys.h"
#include <math.h>

extern volatile uint32_t test_ms;  /* 1ms 计数器 (main.c) */

/* ---------- 内部状态 ---------- */
static float    ramp_speed;      /* 缓启动当前速度 */
static uint32_t ramp_start_ms;   /* 缓启动起始时间 */

#define RAMP_TIME_MS  2000U   /* 加速时间 2s */

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
}

void StepTrack_Stop(void)
{
    step_motor_stop(1);
    step_motor_stop(2);
}

/**
 * @brief 循迹控制：IR 误差 → 步进电机差速直驱
 *
 * 每轮主循环调用一次。根据按键启停状态：
 * - 启动：读取 IR 误差 → P 计算左右差速 → 驱动电机
 * - 停止：两电机停转
 */
void StepTrack_Run(void)
{
    if (!key.start) {
        step_motor_continuous_run(1, 0.0f);
        step_motor_continuous_run(2, 0.0f);
        ramp_speed    = 0.0f;
        ramp_start_ms = 0;
        return;
    }

    /* 缓启动：2s 内从 0 加速到 base_speed */
    if (ramp_speed < (float)base_speed) {
        if (ramp_start_ms == 0) ramp_start_ms = test_ms;
        float frac = (float)(test_ms - ramp_start_ms) / (float)RAMP_TIME_MS;
        if (frac > 1.0f) frac = 1.0f;
        ramp_speed = (float)base_speed * frac;
    }

    float base_dps = mmps_to_dps(ramp_speed);

    int16_t error = Err2();
    float err_norm = error / 7.0f;

    float diff = base_dps * TRACK_KP * err_norm;

    step_motor_continuous_run(1, base_dps + diff);
    step_motor_continuous_run(2, base_dps - diff);
}
