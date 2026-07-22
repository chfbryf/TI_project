/**
 * @file    lpf.c
 * @brief   一阶低通滤波器（PT1）实现
 *
 * 公式：output = alpha * input + (1 - alpha) * last_output
 *       alpha = dt / (tau + dt)
 *       tau   = 1 / (2 * PI * cutoffHz)
 *
 * 首次调用自动初始化为 input 值。
 */

#include "lpf.h"
#include <math.h>

#ifndef M_PI
#define M_PI  3.1415926535f
#endif

static uint8_t _initialized = 0;

void LPF_Init(LPF_t *lpf, float cutoffHz, float sampleHz)
{
    float tau = 1.0f / (2.0f * M_PI * cutoffHz);
    float dt  = 1.0f / sampleHz;
    lpf->alpha = dt / (tau + dt);
    lpf->last_output = 0.0f;
    _initialized = 0;
}

float LPF_Apply(LPF_t *lpf, float input)
{
    if (!_initialized) {
        lpf->last_output = input;
        _initialized = 1;
        return input;
    }

    lpf->last_output = lpf->alpha * input + (1.0f - lpf->alpha) * lpf->last_output;
    return lpf->last_output;
}
