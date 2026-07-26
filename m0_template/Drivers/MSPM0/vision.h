/**
 * @file    vision.h
 * @brief   视觉颜色识别模块
 *
 * 通过 UART1 (DCC101v1_2, PA9=RX) 接收外部视觉模块的单字符颜色指令，
 * 根据颜色输出左右轮差速值，驱动小车执行原地左旋 / 直行 / 原地右旋。
 *
 * 用法：
 *   #include "vision.h"
 *   while (1) {
 *       Vision_Poll();
 *       Vision_Apply();
 *   }
 */

#ifndef VISION_H
#define VISION_H

#include <stdint.h>

/* ---- 颜色差速配置项 ---- */
typedef struct {
    float left;   /* 左轮速度 m/s，负值=反转 */
    float right;  /* 右轮速度 m/s，负值=反转 */
} color_action_t;

/* ---- 最近收到的颜色指令（0=无, 'R'/'G'/'B'=当前颜色） ---- */
extern volatile char g_last_color;

/* ---- API ---- */
void Vision_Poll(void);   /* 轮询 UART，非阻塞 */
void Vision_Apply(void);  /* 根据颜色设置 g_target_speed_L/R */

#endif /* VISION_H */
