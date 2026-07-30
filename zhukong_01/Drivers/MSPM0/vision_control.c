/**
 * @file    vision_control.c
 * @brief   视觉坐标 → 电机3 比例控制
 *
 * 控制链路:
 *   视觉 distance_cm  ─→  P * (target - actual)  ─→  step_motor_continuous_run(3, speed)
 *   视觉丢失          ─→  停转
 *
 * 机械参数: 电机转 180° → 推杆动 52mm → 杆子摆动
 */

#include "vision_control.h"
#include "vision.h"
#include "step_motor.h"
#include "ti_msp_dl_config.h"

/* ═══════════════════════════════════════════════════════════════
 *  内部状态
 * ═══════════════════════════════════════════════════════════════ */
static float   target_cm       = VC_DEFAULT_TARGET_CM;
static float   last_error_cm   = 0.0f;
static uint32_t last_control_ms = 0;

/* ================================================================
 *  VisionControl_Init
 * ================================================================ */
void VisionControl_Init(void)
{
    target_cm       = VC_DEFAULT_TARGET_CM;
    last_error_cm   = 0.0f;
    last_control_ms = 0;
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

    /* ── 2. 钢珠丢失 → 停转 ── */
    if (!vision.ball_detected) {
        step_motor_continuous_run(3, 0.0f);
        last_error_cm   = 0.0f;
        last_control_ms = 0;
        return;
    }

    /* ── 3. 控制间隔 ── */
    if ((test_ms - last_control_ms) < VC_CONTROL_INTERVAL_MS) return;
    last_control_ms = test_ms;

    /* ── 4. 等待新帧 ── */
    if (!vision.data_ready) return;
    vision.data_ready = false;

    /* ── 5. 计算误差 ── */
    float error = target_cm - vision.distance_cm;

    /* ── 6. 死区 ── */
    if (fabsf(error) < VC_DEADBAND_CM) {
        step_motor_continuous_run(3, 0.0f);
        return;
    }

    /* ── 7. PD 控制（比例 + 阻尼，无积分项，匹配参考例程） ── */
    /* 微分 = (当前误差 - 上次误差) / 控制间隔(秒) */
    float dt = VC_CONTROL_INTERVAL_MS / 1000.0f;
    float derivative = (error - last_error_cm) / dt;
    last_error_cm = error;

    float output = VC_P_GAIN * error + VC_D_GAIN * derivative;

    /* ── 8. 限幅 ── */
    if (output >  VC_MAX_SPEED_DPS) output =  VC_MAX_SPEED_DPS;
    if (output < -VC_MAX_SPEED_DPS) output = -VC_MAX_SPEED_DPS;

    /* ── 9. 驱动电机（方向修正用 VC_DIR_SIGN） ── */
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
