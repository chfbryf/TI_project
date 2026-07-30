/**
 * @file    vision_control.c
 * @brief   视觉坐标 → 电机3 PD + 速度耦合控制
 *
 * 公式: error = coupled_target - distance
 *       coupled_target = target + k_couple * ball_vel
 *       output = P*error + D*derror/dt
 *
 * 机械参数: 电机转 160° → 推杆动 52mm → 杆子摆动
 */

#include "vision_control.h"
#include "vision.h"
#include "step_motor.h"
#include "ti_msp_dl_config.h"
#include <math.h>

/* ═══════════════════════════════════════════════════════════════
 *  内部状态
 * ═══════════════════════════════════════════════════════════════ */
static float   target_cm         = VC_DEFAULT_TARGET_CM;
static float   last_error_cm     = 0.0f;
static uint32_t last_control_ms   = 0;

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
    last_control_ms   = 0;
    ema_distance_cm   = 0.0f;
    prev_ema_distance = 0.0f;
    ema_ball_vel      = 0.0f;
    ema_initialized   = false;
    calib_count       = 0;
    calib_sum         = 0.0f;
    calib_done        = true;   /* 跳过自动标定, 固定 11.1cm */
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
    if ((test_ms - last_control_ms) < VC_CONTROL_INTERVAL_MS) return;
    last_control_ms = test_ms;

    /* ── 4. 等待新帧 ── */
    if (!vision.data_ready) return;
    vision.data_ready = false;

    /* ── 5. 自动标定：前N帧的平均距离作为目标 ── */
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
    float dt = VC_CONTROL_INTERVAL_MS / 1000.0f;

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

    /* ── 7. PD + 速度耦合控制 ── */
    /* 速度耦合: 球速修正目标位置, 提前预判刹车 */
    float coupled_target = target_cm + VC_VEL_COUPLE_GAIN * ema_ball_vel;
    float error = coupled_target - ema_distance_cm;

    /* 死区 */
    if (fabsf(error) < VC_DEADBAND_CM) {
        step_motor_continuous_run(3, 0.0f);
        last_error_cm = error;
        return;
    }

    /* PD计算 */
    float derivative = (error - last_error_cm) / dt;
    last_error_cm = error;

    float output = VC_P_GAIN * error + VC_D_GAIN * derivative;

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
    last_error_cm   = 0.0f;
    last_control_ms = 0;
}

float VisionControl_GetError(void)
{
    return last_error_cm;
}

bool VisionControl_IsStable(void)
{
    return fabsf(last_error_cm) < VC_DEADBAND_CM;
}

float VisionControl_GetMotorAngle(void)
{
    return 0.0f;
}
