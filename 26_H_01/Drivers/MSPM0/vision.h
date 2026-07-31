/**
 * @file    vision.h
 * @brief   MaixCam 视觉模块 UART 通讯协议
 *
 * 协议格式 (固定 12 字节帧, 115200 8N1):
 *   $B1,023.00*\n  → 检测到钢珠, 距离 = 23.00 cm
 *   $B0,000.00*\n  → 钢珠丢失
 *
 * 帧结构: $ B <status> , <DDD> . <FF> * \n
 *         0 1     2    3   4 5 6  7  8 9  10 11
 */

#ifndef VISION_H
#define VISION_H

#include <stdint.h>
#include <stdbool.h>

/* 视觉数据 */
typedef struct {
    bool     ball_detected;    /* true = 检测到钢珠, false = 丢失 */
    float    distance_cm;      /* 距离 (cm), 如 23.00 */
    bool     data_ready;       /* 新帧就绪标志 (主循环消费后清零) */
    uint32_t last_update_ms;   /* 最后一帧有效帧时刻 (test_ms) */
} vision_data_t;

extern volatile vision_data_t vision;

void Vision_Init(void);

#endif /* VISION_H */
