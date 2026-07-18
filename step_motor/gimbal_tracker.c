#include "gimbal_tracker.h"
#include "pid.h"
#include "step_motor.h"

/* ================================================================
 *  PID 默认参数（可运行时通过 VOFA 下发修改）
 * ================================================================ */

/* X 轴 PID */
#define PID_X_KP_DEFAULT      0.08f
#define PID_X_KI_DEFAULT      0.009f
#define PID_X_KD_DEFAULT      0.001f
#define PID_X_OUTPUT_LIMIT    15.0f

/* Y 轴 PID */
#define PID_Y_KP_DEFAULT      0.08f
#define PID_Y_KI_DEFAULT      0.001f
#define PID_Y_KD_DEFAULT      0.001f
#define PID_Y_OUTPUT_LIMIT    15.0f

/* ================================================================
 *  内部状态
 * ================================================================ */

static PID_Controller g_pid_x;
static PID_Controller g_pid_y;
static float g_dt = 0.05f;
static float g_angle_x = 135.0f;
static float g_angle_y = 90.0f;
static uint8_t g_enabled = 1;

/* 用于 VOFA 波形显示的实时数据 */
static float g_pid_output_x = 0.0f;
static float g_pid_output_y = 0.0f;
static float g_pid_error_x = 0.0f;
static float g_pid_error_y = 0.0f;

/* ================================================================
 *  公共接口
 * ================================================================ */

void GimbalTracker_Init(float dt_seconds)
{
    g_dt = dt_seconds;

    /* PID 目标设为 0："零误差"，MaixCam2 已计算好误差，直接作为反馈输入 */
    PID_Init(&g_pid_x,
             PID_X_KP_DEFAULT, PID_X_KI_DEFAULT, PID_X_KD_DEFAULT,
             0.0f,
             PID_X_OUTPUT_LIMIT);

    PID_Init(&g_pid_y,
             PID_Y_KP_DEFAULT, PID_Y_KI_DEFAULT, PID_Y_KD_DEFAULT,
             0.0f,
             PID_Y_OUTPUT_LIMIT);

    g_angle_x = 135.0f;
    g_angle_y = 90.0f;
    step_set_angle(g_angle_x, 1);
    step_set_angle(g_angle_y, 2);
}

void GimbalTracker_Update(float error_x, float error_y, uint8_t valid)
{
    if (!g_enabled) return;

    /* valid=0 表示目标丢失或本帧数据不可用，保持当前位置 */
    if (!valid) {
        return;
    }

    /**
     * PID setpoint = 0，feedback = -error
     * PID error = 0 - (-error) = +error
     * 当 error_x > 0（目标偏右），PID 输出正值，云台向右转
     * 当 error_y > 0（目标偏下），PID 输出正值，云台向下转
     */
    float delta_x = PID_Compute(&g_pid_x, -error_x, g_dt);
    float delta_y = PID_Compute(&g_pid_y, -error_y, g_dt);

    /* 保存 PID 数据供 VOFA 读取 */
    g_pid_output_x = delta_x;
    g_pid_output_y = delta_y;
    g_pid_error_x = error_x;
    g_pid_error_y = error_y;

    g_angle_x += delta_x;
    g_angle_y -= delta_y;

    if (g_angle_x < SERVO_X_MIN) g_angle_x = SERVO_X_MIN;
    if (g_angle_x > SERVO_X_MAX) g_angle_x = SERVO_X_MAX;
    if (g_angle_y < SERVO_Y_MIN) g_angle_y = SERVO_Y_MIN;
    if (g_angle_y > SERVO_Y_MAX) g_angle_y = SERVO_Y_MAX;

    step_set_angle(g_angle_x, 1);
    step_set_angle(g_angle_y, 2);
}

float GimbalTracker_GetAngleX(void)
{
    return g_angle_x;
}

float GimbalTracker_GetAngleY(void)
{
    return g_angle_y;
}

float GimbalTracker_GetPID_OutputX(void)
{
    return g_pid_output_x;
}

float GimbalTracker_GetPID_OutputY(void)
{
    return g_pid_output_y;
}

float GimbalTracker_GetPID_ErrorX(void)
{
    return g_pid_error_x;
}

float GimbalTracker_GetPID_ErrorY(void)
{
    return g_pid_error_y;
}

float GimbalTracker_GetKpX(void) { return g_pid_x.kp; }
float GimbalTracker_GetKiX(void) { return g_pid_x.ki; }
float GimbalTracker_GetKdX(void) { return g_pid_x.kd; }

void GimbalTracker_SetPID_X(float kp, float ki, float kd)
{
    g_pid_x.kp = kp;
    g_pid_x.ki = ki;
    g_pid_x.kd = kd;
    /* 重置积分和历史误差，防止切换参数时的冲击 */
    g_pid_x.integral = 0.0f;
    g_pid_x.prev_error = 0.0f;
}

float GimbalTracker_GetKpY(void) { return g_pid_y.kp; }
float GimbalTracker_GetKiY(void) { return g_pid_y.ki; }
float GimbalTracker_GetKdY(void) { return g_pid_y.kd; }

void GimbalTracker_SetPID_Y(float kp, float ki, float kd)
{
    g_pid_y.kp = kp;
    g_pid_y.ki = ki;
    g_pid_y.kd = kd;
    g_pid_y.integral = 0.0f;
    g_pid_y.prev_error = 0.0f;
}

void GimbalTracker_Enable(uint8_t enable)
{
    g_enabled = enable;
}

uint8_t GimbalTracker_IsEnabled(void)
{
    return g_enabled;
}
