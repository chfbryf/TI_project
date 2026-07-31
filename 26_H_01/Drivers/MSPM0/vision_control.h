/**
 * @file    vision_control.h
 * @brief   视觉坐标 → 电机3 PD + 速度耦合控制
 *
 * 公式: error = coupled_target - distance
 *       coupled_target = target + k_couple * ball_vel
 *       output = P*error + D*derror/dt
 *
 * 机械参数: 电机转 160° → 推杆动 52mm → 杆子摆动
 */

#ifndef VISION_CONTROL_H
#define VISION_CONTROL_H

#include <stdbool.h>

/* ═══════════════════════════════════════════════════════════════
 *  PD + 速度耦合 参数
 * ═══════════════════════════════════════════════════════════════ */

/* 比例增益: 1cm误差 → P*1 deg/s 修正速度 */
#define VC_P_GAIN              4.5f

/* 微分增益: 阻尼抑制超调 */
#define VC_D_GAIN             6.5f

/* 速度-位置耦合: 球速动态修正目标, 防冲过头 (0=关闭, 负值才正确) */
#define VC_VEL_COUPLE_GAIN     -0.20f

/* ═══════════════════════════════════════════════════════════════
 *  滤波 & 控制参数
 * ═══════════════════════════════════════════════════════════════ */

/* 输出限幅: 电机最大角速度 (deg/s) */
#define VC_MAX_SPEED_DPS       150.0f

/* 视觉距离 EMA 滤波系数 (0~1, 越小越平滑但延迟越大) */
#define VC_EMA_ALPHA           0.8f

/* 钢珠速度 EMA 滤波系数 (应 < EMA_ALPHA) */
#define VC_VEL_EMA_BETA       0.75f

/* ═══════════════════════════════════════════════════════════════
 *  控制参数
 * ═══════════════════════════════════════════════════════════════ */

/* 控制间隔: 两次计算的最短间隔 (ms) */
#define VC_CONTROL_INTERVAL_MS   30U

/* 死区: |偏差| < 此值 (cm) 电机停转, 防止抖动 */
#define VC_DEADBAND_CM           1.0f

/* 视觉超时: 超过此时长未收到有效帧视为钢珠丢失 (ms) */
#define VC_VISION_TIMEOUT_MS   500U

/* 默认目标距离 (cm) — 仅在手动模式下使用 */
#define VC_DEFAULT_TARGET_CM    11.1f

/* 自动标定帧数: 上电后用前N帧的平均距离作为目标 */
#define VC_AUTO_CALIB_FRAMES        3

/* 电机方向: 正转=距离减小, 误差为负时需正向输出 → 1.0f */
#define VC_DIR_SIGN               -1.0f

/* ═══════════════════════════════════════════════════════════════
 *  公开接口
 * ═══════════════════════════════════════════════════════════════ */

void VisionControl_Init(void);
void VisionControl_Run(void);

void VisionControl_SetTarget(float target_cm);
float VisionControl_GetError(void);
bool  VisionControl_IsStable(void);
float VisionControl_GetMotorAngle(void);

#endif /* VISION_CONTROL_H */
