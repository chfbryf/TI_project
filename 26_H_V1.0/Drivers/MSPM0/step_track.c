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
extern volatile uint32_t track_start_ms;          /* 行驶起始时间 (main.c) */
extern uint8_t  decelerating;                     /* 减速中标志 (main.c) */

/* ---------- 内部状态 ---------- */
static float    ramp_speed;      /* 缓启动当前速度 */
static uint32_t ramp_start_ms;   /* 缓启动起始时间 */

#define RAMP_TIME_MS     3000U   /* 加速时间 Task1: 3s */
#define RAMP_TIME_T34_MS 4000U   /* 加速时间 Task3/4: 4s */

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
        return;
    }

    /* --- 十字停车减速：500ms 线性降到 0，P 循迹保持 --- */
    if (decelerating) {
        static uint32_t decel_start_ms   = 0;
        static float    decel_start_speed = 0.0f;

        if (decel_start_ms == 0) {
            decel_start_ms   = test_ms;
            decel_start_speed = ramp_speed;
        }

        uint32_t elapsed = test_ms - decel_start_ms;
        if (elapsed >= DECEL_TIME_MS) {
            ramp_speed      = 0.0f;
            ramp_start_ms   = 0;
            decel_start_ms  = 0;
            decelerating    = 0;
            key.start       = 0;           /* 提交停车 */
        } else {
            float frac = (float)elapsed / (float)DECEL_TIME_MS;
            ramp_speed = decel_start_speed * (1.0f - frac);
        }

        /* 减速期间仍然跑 P 循迹 */
        float base_dps = mmps_to_dps(ramp_speed);
        int16_t error_raw = Err2();
        float diff = base_dps * TRACK_KP * ((float)error_raw / 7.0f);
        step_motor_continuous_run(1, base_dps + diff);
        step_motor_continuous_run(2, base_dps - diff);
        return;
    }

    /* 计算目标速度 */
    int16_t target_speed;
    uint8_t t2_decel = 0;  /* Task 2 是否处于减速阶段 */
    if (key.task_id == 2) {
        uint32_t elapsed = test_ms - track_start_ms;
        if (elapsed < TRACK_T2_SWITCH_MS) {
            target_speed = TRACK_SPEED_T2_EARLY;        /* 200 mm/s */
        } else if (elapsed < TRACK_T2_SWITCH_MS + TRACK_T2_DECEL_MS) {
            /* 15s~16s: 线性从 200 降到 90 */
            float frac = (float)(elapsed - TRACK_T2_SWITCH_MS) / (float)TRACK_T2_DECEL_MS;
            target_speed = (int16_t)(TRACK_SPEED_T2_EARLY
                           + frac * (TRACK_SPEED_T2_LATE - TRACK_SPEED_T2_EARLY));
            t2_decel = 1;
        } else {
            target_speed = TRACK_SPEED_T2_LATE;          /* 90 mm/s */
        }
    } else {
        target_speed = (key.task_id == 1) ? TRACK_SPEED_T1 : TRACK_SPEED_T34;
    }

    /* 缓启动/缓变速 */
    if (ramp_speed != (float)target_speed) {
        if (t2_decel) {
            /* Task 2 减速阶段：瞬时切换目标速度 */
            ramp_speed = (float)target_speed;
        } else {
            if (ramp_start_ms == 0) ramp_start_ms = test_ms;
            uint32_t ramp_ms = (key.task_id == 3 || key.task_id == 4) ? RAMP_TIME_T34_MS : RAMP_TIME_MS;
            float frac = (float)(test_ms - ramp_start_ms) / (float)ramp_ms;
            if (frac > 1.0f) frac = 1.0f;
            ramp_speed = ramp_speed + ((float)target_speed - ramp_speed) * frac;
        }
        if (fabsf(ramp_speed - (float)target_speed) < 0.5f) {
            ramp_speed = (float)target_speed;
        }
    }

    float base_dps = mmps_to_dps(ramp_speed);

    /* 纯P控制 */
    int16_t error_raw = Err2();
    float diff = base_dps * TRACK_KP * ((float)error_raw / 7.0f);

    float left  = base_dps + diff;
    float right = base_dps - diff;

    step_motor_continuous_run(1, left);
    step_motor_continuous_run(2, right);
}
