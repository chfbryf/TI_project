/**
 * @file    lpf.h
 * @brief   一阶低通滤波器（PT1），用于编码器测速平滑
 */

#ifndef LPF_H
#define LPF_H

#include <stdint.h>

typedef struct {
    float alpha;        /* 滤波系数 (0~1)，越小滤波越强 */
    float last_output;  /* 上一次滤波输出 */
} LPF_t;

/**
 * @brief 用截止频率初始化低通滤波器
 * @param lpf      滤波器实例
 * @param cutoffHz 截止频率（Hz），如 2.0 表示 2Hz
 * @param sampleHz 采样频率（Hz），如 20.0 表示 20Hz（50ms周期）
 */
void LPF_Init(LPF_t *lpf, float cutoffHz, float sampleHz);

/**
 * @brief 滤波计算
 * @param lpf   滤波器实例
 * @param input 当前输入值
 * @return 滤波后的输出值
 */
float LPF_Apply(LPF_t *lpf, float input);

#endif /* LPF_H */
