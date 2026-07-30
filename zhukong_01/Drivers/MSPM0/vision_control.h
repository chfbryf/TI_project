/**
 * @file    vision_control.h
 * @brief   视觉坐标 → 电机3 PD + 前馈控制
 *
 * 读取 vision 全局结构体中的钢珠距离,
 * 通过 PD 反馈 + 速度前馈 驱动步进电机 3 实时调速。
 *
 * 公式: speed = P*error + D*derror/dt + FF*ball_velocity
 *        反馈(追偏差)              前馈(预判钢珠速度)
 *
 * 机械参数: 电机转 160° → 推杆动 52mm → 杆子摆动
 */

#ifndef VISION_CONTROL_H
#define VISION_CONTROL_H

#include <stdbool.h>

/* ═══════════════════════════════════════════════════════════════
 *  PD + 前馈 参数
 * ═══════════════════════════════════════════════════════════════ */

/* 比例增益: 1cm误差 → P*1 deg/s 修正速度，调大=响应快, 调小=平稳 */
#define VC_P_GAIN              5.0f

/* 微分增益: 阻尼预测, 抑制超调 */
#define VC_D_GAIN               0.0f

/* 前馈增益: 钢珠1cm/s → 需要多少deg/s电机速度去抵消 */
#define VC_FF_VEL_GAIN         1.0f

/* 输出限幅: 电机最大角速度 (deg/s) */
#define VC_MAX_SPEED_DPS       150.0f

/* 视觉距离 EMA 滤波系数 (0~1, 越大响应越快但噪声越大) */
#define VC_EMA_ALPHA           0.8f

/* 钢珠速度 EMA 滤波系数 */
#define VC_VEL_EMA_BETA        0.6f

/* ═══════════════════════════════════════════════════════════════
 *  控制参数
 * ═══════════════════════════════════════════════════════════════ */

/* 控制间隔: 两次 PID 计算的最短间隔 (ms)，匹配参考例程 30ms */
#define VC_CONTROL_INTERVAL_MS   30U

/* 死区: |偏差| < 此值 (cm) 电机停转, 防止抖动 */
#define VC_DEADBAND_CM           0.3f

/* 视觉超时: 超过此时长未收到有效帧视为钢珠丢失 (ms) */
#define VC_VISION_TIMEOUT_MS   500U

/* 默认目标距离 (cm) — 仅在手动模式下使用 */
#define VC_DEFAULT_TARGET_CM    11.1f

/* 自动标定帧数: 上电后用前N帧的平均距离作为目标 */
#define VC_AUTO_CALIB_FRAMES        3

/* 电机方向: 正转=距离减小, 误差为负时需正向输出 → 1.0f */
#define VC_DIR_SIGN               1.0f

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
