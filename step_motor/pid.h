#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct {
    float kp;                 // 比例系数
    float ki;                 // 积分系数
    float kd;                 // 微分系数
    float setpoint;           // 目标值（如图像中心 320 或 240）
    float integral;           // 积分累加
    float prev_error;         // 上次误差
    float output_limit;       // 输出限幅（绝对值，单位：度）
    uint32_t last_time;       // 上次调用时间（ms）
} PID_Controller;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float setpoint, float output_limit);
float PID_Compute(PID_Controller *pid, float feedback, float dt);   // dt 单位：秒

#endif
