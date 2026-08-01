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

extern volatile uint32_t test_ms;        /* 1ms 计数器 (main.c) */
extern uint32_t track_start_ms;          /* 行驶起始时间 (main.c) */

/* ---------- 内部状态 ---------- */
static float    ramp_speed;      /* 缓启动当前速度 */
static uint32_t ramp_start_ms;   /* 缓启动起始时间 */
static float    prev_left;       /* 上一帧左轮速度，用于限幅 */
static float    prev_right;      /* 上一帧右轮速度，用于限幅 */

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
 * @brief 获取小车当前缓启速度，供 vision_control 前馈补偿
 * @return 速度 mm/s，未启动时返回 0
 */
float StepTrack_GetRampSpeed(void)
{
    return ramp_speed;
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
        ramp_speed      = 0.0f;
        ramp_start_ms   = 0;
        prev_left       = 0.0f;
        prev_right      = 0.0f;
        return;
    }

    /* 计算目标速度 */
    int16_t target_speed;
    uint8_t t2_decel = 0;  /* Task 2 是否处于减速阶段 */
    if (key.task_id == 2) {
        uint32_t elapsed = test_ms - track_start_ms;
        if (elapsed < TRACK_T2_SWITCH_MS) {
            target_speed = TRACK_SPEED_T2_EARLY;        /* 180 mm/s */
        } else if (elapsed < TRACK_T2_SWITCH_MS + TRACK_T2_DECEL_MS) {
            /* 15s~15.5s: 线性从 180 降到 120 */
            float frac = (float)(elapsed - TRACK_T2_SWITCH_MS) / (float)TRACK_T2_DECEL_MS;
            target_speed = (int16_t)(TRACK_SPEED_T2_EARLY
                           + frac * (TRACK_SPEED_T2_LATE - TRACK_SPEED_T2_EARLY));
            t2_decel = 1;
        } else {
            target_speed = TRACK_SPEED_T2_LATE;          /* 120 mm/s */
        }
    } else {
        target_speed = (key.task_id == 1) ? TRACK_SPEED_T1 : TRACK_SPEED_T34;
    }

    /* 缓启动/缓变速 */
    if (ramp_speed != (float)target_speed) {
        if (t2_decel) {
            /* Task 2 减速阶段：直接跟随目标，0.5s 内平滑过渡 */
            ramp_speed = (float)target_speed;
        } else {
            if (ramp_start_ms == 0) ramp_start_ms = test_ms;
            float frac = (float)(test_ms - ramp_start_ms) / (float)RAMP_TIME_MS;
            if (frac > 1.0f) frac = 1.0f;
            ramp_speed = ramp_speed + ((float)target_speed - ramp_speed) * frac;
        }
        if (fabsf(ramp_speed - (float)target_speed) < 0.5f) {
            ramp_speed = (float)target_speed;
        }
    }

    float base_dps = mmps_to_dps(ramp_speed);

    /* EMA滤波 + PD控制：EMA平滑传感器抖动，D抑制误差突变防摆动 */
    static float prev_error = 0.0f;
    int16_t error_raw = Err2();
    float error_ema = TRACK_ERROR_EMA_ALPHA * (float)error_raw
                    + (1.0f - TRACK_ERROR_EMA_ALPHA) * prev_error;
    float derivative = (error_ema - prev_error) / 0.05f;  /* dt≈50ms */
    prev_error = error_ema;

    float diff = base_dps * (TRACK_KP * (error_ema / 7.0f)
                           + TRACK_KD * (derivative / 7.0f));

    /* 速度限幅: 防止PD突变导致电机急加速/急减速 */
    float left  = base_dps + diff;
    float right = base_dps - diff;
    if (left  > prev_left  + TRACK_MAX_DPS_DELTA) left  = prev_left  + TRACK_MAX_DPS_DELTA;
    if (left  < prev_left  - TRACK_MAX_DPS_DELTA) left  = prev_left  - TRACK_MAX_DPS_DELTA;
    if (right > prev_right + TRACK_MAX_DPS_DELTA) right = prev_right + TRACK_MAX_DPS_DELTA;
    if (right < prev_right - TRACK_MAX_DPS_DELTA) right = prev_right - TRACK_MAX_DPS_DELTA;
    prev_left  = left;
    prev_right = right;

    step_motor_continuous_run(1, left);
    step_motor_continuous_run(2, right);
}
