#ifndef __GIMBAL_TRACKER_H
#define __GIMBAL_TRACKER_H

#include <stdint.h>

/* 图像分辨率 */
#define IMAGE_WIDTH  640
#define IMAGE_HEIGHT 480

/* 图像中心（PID 目标点） */
#define IMAGE_CX     (IMAGE_WIDTH / 2.0f)
#define IMAGE_CY     (IMAGE_HEIGHT / 2.0f)

/* 舵机行程 */
#define SERVO_X_MIN  0.0f
#define SERVO_X_MAX  270.0f
#define SERVO_Y_MIN  0.0f
#define SERVO_Y_MAX  180.0f

/**
 * @brief 初始化云台追踪器
 * @param dt_seconds  每帧时间间隔（秒），例如 0.05f 对应 50ms / 20Hz
 */
void GimbalTracker_Init(float dt_seconds);

/**
 * @brief 更新目标坐标，内部完成 PID 计算并驱动舵机
 * @param target_x  目标在图像中的 X 坐标 (0~640)
 * @param target_y  目标在图像中的 Y 坐标 (0~480)
 *
 * 当 (x, y) 均为 -1 时表示目标丢失，舵机保持当前位置不动。
 */
void GimbalTracker_Update(float target_x, float target_y);

/**
 * @brief 获取当前舵机角度
 */
float GimbalTracker_GetAngleX(void);
float GimbalTracker_GetAngleY(void);

/**
 * @brief 获取当前 PID 输出（用于 VOFA 波形显示）
 */
float GimbalTracker_GetPID_OutputX(void);
float GimbalTracker_GetPID_OutputY(void);

/**
 * @brief 获取当前 PID 误差
 */
float GimbalTracker_GetPID_ErrorX(void);
float GimbalTracker_GetPID_ErrorY(void);

/**
 * @brief 获取/设置 X 轴 PID 参数
 */
float GimbalTracker_GetKpX(void);
float GimbalTracker_GetKiX(void);
float GimbalTracker_GetKdX(void);
void  GimbalTracker_SetPID_X(float kp, float ki, float kd);

/**
 * @brief 获取/设置 Y 轴 PID 参数
 */
float GimbalTracker_GetKpY(void);
float GimbalTracker_GetKiY(void);
float GimbalTracker_GetKdY(void);
void  GimbalTracker_SetPID_Y(float kp, float ki, float kd);

/**
 * @brief 设置追踪器使能/失能
 */
void GimbalTracker_Enable(uint8_t enable);
uint8_t GimbalTracker_IsEnabled(void);

#endif
