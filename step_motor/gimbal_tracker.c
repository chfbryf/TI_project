#include "gimbal_tracker.h"
#include "pid.h"
#include "step_motor.h"
#include <math.h>

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
 *  速度控制参数
 * ================================================================
 *  PID 输出 delta（度/帧）→ 电机角速度 speed = delta / dt
 *  SPEED_MIN: 低于此速度认为已对准，电机停转。降到 0.5°/s，
 *             对应约 1 像素误差（含 P 项），实现精细化跟踪。
 *  SPEED_MAX: 最高转速，提高到 100°/s，增大 PID 输出的有效范围，
 *             减少"PID 算细了但速度一律封顶"的二值化问题。
 */
#define SPEED_MAX  100.0f
#define SPEED_MIN  0.5f

/*
 * valid 去抖：连续丢帧 N 次才确认目标确实丢失。
 * 避免单帧 valid=0 导致电机频繁启停，看起来像"不响应"。
 */
#define VALID_LOST_THRESHOLD  3

/* 系统毫秒计数器（由 SysTick 中断维护） */
extern volatile uint32_t g_sys_tick_ms;

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

    g_angle_x = 90.0f;
    g_angle_y = 90.0f;
}

void GimbalTracker_Update(float error_x, float error_y, uint8_t valid)
{
    static uint8_t motor1_running = 0;
    static uint8_t motor2_running = 0;
    static uint32_t last_tick = 0;
    static uint8_t valid_lost_cnt = 0;

    if (!g_enabled) {
        return;
    }

    /*
     * 动态计算帧间隔 dt，替代硬编码的 g_dt。
     * 首次调用时 last_tick=0，dt 会很大，用 g_dt 兜底。
     */
    uint32_t now = g_sys_tick_ms;
    float dt = (float)(now - last_tick) / 1000.0f;
    if (dt < 0.005f || dt > 0.5f) {
        dt = g_dt;   /* 首帧或异常间隔，用初始化默认值 */
    }
    last_tick = now;

    /*
     * valid 去抖：连续丢帧 VALID_LOST_THRESHOLD 次才确认目标丢失。
     * 避免摄像头偶发 valid=0 导致电机频繁启停。
     */
    if (!valid) {
        valid_lost_cnt++;
        if (valid_lost_cnt < VALID_LOST_THRESHOLD) {
            return;  /* 暂不停机，等待确认 */
        }
        if (motor1_running) { step_motor_stop(1); motor1_running = 0; }
        if (motor2_running) { step_motor_stop(2); motor2_running = 0; }
        return;
    }
    valid_lost_cnt = 0;

    /**
     * PID setpoint = 0，feedback = -error
     * PID error = 0 - (-error) = +error
     * 当 error_x > 0（目标偏右），PID 输出正值，云台向右转
     * 当 error_y > 0（目标偏下），PID 输出正值，云台向下转
     */
    float delta_x = PID_Compute(&g_pid_x, -error_x, dt);
    float delta_y = PID_Compute(&g_pid_y, -error_y, dt);

    /* 保存 PID 数据供 VOFA 读取 */
    g_pid_output_x = delta_x;
    g_pid_output_y = delta_y;
    g_pid_error_x = error_x;
    g_pid_error_y = error_y;

    g_angle_x += delta_x;
    g_angle_y -= delta_y;

    /* 角度限幅 */
    if (g_angle_x < SERVO_X_MIN) g_angle_x = SERVO_X_MIN;
    if (g_angle_x > SERVO_X_MAX) g_angle_x = SERVO_X_MAX;
    if (g_angle_y < SERVO_Y_MIN) g_angle_y = SERVO_Y_MIN;
    if (g_angle_y > SERVO_Y_MAX) g_angle_y = SERVO_Y_MAX;

    /*
     * 连续速度控制：将 PID 输出的角度增量转为电机转速
     * speed = delta / dt，限制在 SPEED_MIN ~ SPEED_MAX 度/秒
     */

    /* 电机1（X轴） */
    float speed_x = fabsf(delta_x) / dt;
    if (speed_x > SPEED_MAX) speed_x = SPEED_MAX;

    if (speed_x > SPEED_MIN) {
        step_motor_dir_set(delta_x > 0 ? 1 : 0, 1);
        step_remain_1 = 0;  /* 连续运行 */
        step_set_speed(speed_x, 1);
        if (!motor1_running) {
            step_motor_start(1);
            motor1_running = 1;
        }
    } else {
        if (motor1_running) {
            step_motor_stop(1);
            motor1_running = 0;
        }
    }

    /* 电机2（Y轴） */
    float speed_y = fabsf(delta_y) / dt;
    if (speed_y > SPEED_MAX) speed_y = SPEED_MAX;

    if (speed_y > SPEED_MIN) {
        step_motor_dir_set(delta_y > 0 ? 0 : 1, 2);
        step_remain_2 = 0;  /* 连续运行 */
        step_set_speed(speed_y, 2);
        if (!motor2_running) {
            step_motor_start(2);
            motor2_running = 1;
        }
    } else {
        if (motor2_running) {
            step_motor_stop(2);
            motor2_running = 0;
        }
    }
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
