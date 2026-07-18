#ifndef __GIMBAL_TRACKER_H
#define __GIMBAL_TRACKER_H

#include <stdint.h>

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
 * @brief 更新目标误差，内部完成 PID 计算并驱动电机
 * @param error_x  MaixCam2 下发的 X 误差（像素）
 * @param error_y  MaixCam2 下发的 Y 误差（像素）
 * @param valid    1=误差有效，0=目标丢失/未识别，电机保持当前位置
 *
 * error_x > 0：目标在当前对象右侧，电机需向右转
 * error_y > 0：目标在当前对象下方，电机需向下转
 */
void GimbalTracker_Update(float error_x, float error_y, uint8_t valid);

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
