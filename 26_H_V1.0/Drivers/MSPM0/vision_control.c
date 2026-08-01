/**
 * @file    vision_control.c
 * @brief   视觉坐标 → 电机3 PD + 速度耦合 + 小车加速度前馈
 *
 * 公式: error = coupled_target - distance
 *       coupled_target = target + k_couple * ball_vel
 *       ff = k_ff * car_accel  (小车起步时预判钢珠后滚)
 *       output = P*error + D*derror/dt + ff
 *
 * 机械参数: 电机转 160° → 推杆动 52mm → 杆子摆动
 */

#include "vision_control.h"
#include "vision.h"
#include "step_motor.h"
#include "step_track.h"
#include "key.h"
#include "ti_msp_dl_config.h"
#include <math.h>

/* ═══════════════════════════════════════════════════════════════
 *  内部状态
 * ═══════════════════════════════════════════════════════════════ */
static float   target_cm         = VC_DEFAULT_TARGET_CM;
static float   last_error_cm     = 0.0f;
static float   prev_ramp_speed   = 0.0f;   /* 上一帧小车速度，用于算加速度 */
static uint32_t last_control_ms   = 0;
static bool     skip_derivative    = false;  /* SetTarget后首帧抑制D尖峰 */

/* 前馈相关 */
static float   ema_distance_cm   = 0.0f;   /* EMA滤波后的距离 */
static float   prev_ema_distance = 0.0f;   /* 上一帧滤波距离，用于算速度 */
static float   ema_ball_vel      = 0.0f;   /* EMA滤波后的钢珠速度 (cm/s) */
static bool    ema_initialized   = false;  /* EMA滤波器是否已初始化 */

/* 自动标定 */
static uint8_t calib_count       = 0;      /* 已采集帧数 */
static float   calib_sum         = 0.0f;   /* 距离累加和 */
static bool    calib_done        = false;  /* 标定是否完成 */

/* ================================================================
 *  VisionControl_Init
 * ================================================================ */
void VisionControl_Init(void)
{
    target_cm         = VC_DEFAULT_TARGET_CM;
    last_error_cm     = 0.0f;
    prev_ramp_speed   = 0.0f;
    last_control_ms   = 0;
    skip_derivative   = false;
    ema_distance_cm   = 0.0f;
    prev_ema_distance = 0.0f;
    ema_ball_vel      = 0.0f;
    ema_initialized   = false;
    calib_count       = 0;
    calib_sum         = 0.0f;
    calib_done        = true;   /* 默认跳过标定，直接用 VC_DEFAULT_TARGET_CM */
}

/* ================================================================
 *  VisionControl_Run（主循环调用）
 * ================================================================ */
void VisionControl_Run(void)
{
    extern volatile uint32_t test_ms;

    /* ── 1. 超时检测 ── */
    if (vision.last_update_ms > 0 &&
        (test_ms - vision.last_update_ms) > VC_VISION_TIMEOUT_MS)
    {
        vision.ball_detected = false;
        vision.data_ready    = true;
        vision.last_update_ms = test_ms;
    }

    /* ── 2. 钢珠丢失 → 停转，保持目标值不变 ── */
    if (!vision.ball_detected) {
        step_motor_continuous_run(3, 0.0f);
        last_error_cm     = 0.0f;
        last_control_ms   = 0;
        ema_initialized   = false;
        return;
    }

    /* ── 3. 控制间隔 ── */
    uint32_t now = test_ms;
    if (last_control_ms != 0 && (now - last_control_ms) < VC_CONTROL_INTERVAL_MS) return;

    /* ── 4. 等待新帧 ── */
    if (!vision.data_ready) return;
    vision.data_ready = false;

    /* 实际 dt：用两次控制的真实间隔，首帧默认 30ms */
    float dt = (last_control_ms != 0) ? (float)(now - last_control_ms) / 1000.0f
                                      : VC_CONTROL_INTERVAL_MS / 1000.0f;
    last_control_ms = now;

    /* ── 5. 自动标定：前N帧平均距离作为目标 ── */
    if (!calib_done) {
        calib_sum += vision.distance_cm;
        calib_count++;
        if (calib_count >= VC_AUTO_CALIB_FRAMES) {
            target_cm  = calib_sum / (float)calib_count;
            calib_done = true;
        }
        return;  /* 标定期间不控制电机 */
    }

    /* ── 6. EMA 滤波距离 + 估算钢珠速度 ── */

    if (!ema_initialized) {
        /* 首次直接赋值，不做滤波 */
        ema_distance_cm   = vision.distance_cm;
        prev_ema_distance = vision.distance_cm;
        ema_ball_vel      = 0.0f;
        ema_initialized   = true;
    } else {
        /* EMA: filtered = alpha*new + (1-alpha)*old */
        ema_distance_cm = VC_EMA_ALPHA * vision.distance_cm
                        + (1.0f - VC_EMA_ALPHA) * ema_distance_cm;

        /* 钢珠速度 = 滤波距离的差分 */
        float raw_vel = (ema_distance_cm - prev_ema_distance) / dt;
        prev_ema_distance = ema_distance_cm;

        /* EMA 滤波速度 */
        ema_ball_vel = VC_VEL_EMA_BETA * raw_vel
                     + (1.0f - VC_VEL_EMA_BETA) * ema_ball_vel;
    }

    /* ── 7. PD + 速度耦合 + 小车加速度前馈 ── */

    /* 小车加速度前馈: 仅 Task 1/3/4 小车运动时有效 */
    float ramp_now = StepTrack_GetRampSpeed();
    float ff_out = 0.0f;
    if (key.start && (key.task_id == 1 || key.task_id == 3 || key.task_id == 4)) {
        float car_accel_mmps2 = (ramp_now - prev_ramp_speed) / dt;   /* mm/s² */
        float car_accel_cmps2 = car_accel_mmps2 * 0.1f;              /* → cm/s² */
        ff_out = VC_CAR_ACCEL_FF_GAIN * car_accel_cmps2;             /* → deg/s */
        prev_ramp_speed = ramp_now;
    } else {
        prev_ramp_speed = 0.0f;    /* 非小车运动场景，清零防残留 */
    }

    /* 速度耦合: 球速修正目标位置, 提前预判刹车 */
    float coupled_target = target_cm + VC_VEL_COUPLE_GAIN * ema_ball_vel;
    float error = coupled_target - ema_distance_cm;

    /* 死区（无前馈时才完全停机） */
    if (fabsf(error) < VC_DEADBAND_CM && fabsf(ff_out) < 1.0f) {
        step_motor_continuous_run(3, 0.0f);
        last_error_cm = error;
        return;
    }

    /* PD计算 */
    float derivative = (error - last_error_cm) / dt;
    if (skip_derivative) {
        derivative = 0.0f;           /* SetTarget后首帧抑制D尖峰 */
        skip_derivative = false;
    }
    last_error_cm = error;

    float output = VC_P_GAIN * error + VC_D_GAIN * derivative + ff_out;

    /* 限幅 */
    if (output >  VC_MAX_SPEED_DPS) output =  VC_MAX_SPEED_DPS;
    if (output < -VC_MAX_SPEED_DPS) output = -VC_MAX_SPEED_DPS;

    /* 驱动电机 */
    step_motor_continuous_run(3, output * VC_DIR_SIGN);
}

/* ═══════════════════════════════════════════════════════════════
 *  接口（保留兼容）
 * ═══════════════════════════════════════════════════════════════ */

void VisionControl_SetTarget(float cm)
{
    target_cm = cm;
    if (target_cm < 0.0f) target_cm = 0.0f;
    last_error_cm    = 0.0f;
    last_control_ms  = 0;
    skip_derivative  = true;   /* 首帧抑制D尖峰 */
}

float VisionControl_GetError(void)
{
    return last_error_cm;
}

bool VisionControl_IsStable(void)
{
    return vision.ball_detected && (fabsf(last_error_cm) < VC_DEADBAND_CM);
}

float VisionControl_GetMotorAngle(void)
{
    return 0.0f;
}

void VisionControl_ResetCalib(void)
{
    calib_count = 0;
    calib_sum   = 0.0f;
    calib_done  = false;
    ema_initialized = false;  /* 重置滤波器，标定后重新初始化 */
}

bool VisionControl_IsCalibDone(void)
{
    return calib_done;
}
