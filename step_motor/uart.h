#ifndef __UART_H
#define __UART_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* 一帧最大长度（AIM 报文约 80 字节，取 128 足够） */
#define UART_RX_BUF_SIZE   128

/**
 * @brief AIM 报文回调
 * @param valid           1=误差有效，0=不应使用
 * @param mode            "CENTER" / "CIRCLE" / "OFF"
 * @param point_index     圆点索引，非画圆状态为 -1
 * @param point_count     圆点总数
 * @param point_locked    当前点到达切换条件为 1
 * @param error_x         X 误差（像素）
 * @param error_y         Y 误差（像素）
 * @param target_x        目标 X，不存在时为 -1
 * @param target_y        目标 Y，不存在时为 -1
 * @param laser_x         激光 X，未识别时为 -1
 * @param laser_y         激光 Y，未识别时为 -1
 * @param rect_center_x   矩形中心 X
 * @param rect_center_y   矩形中心 Y
 * @param rect_confidence 矩形置信度 0~1
 */
typedef void (*AIMCallback)(uint8_t valid, const char *mode,
                             int point_index, int point_count, int point_locked,
                             float error_x, float error_y,
                             float target_x, float target_y,
                             float laser_x, float laser_y,
                             float rect_center_x, float rect_center_y,
                             float rect_confidence);

/**
 * @brief 画圆完成回调
 * @param point_count  完成时的圆点总数
 */
typedef void (*CircleDoneCallback)(int point_count);

void UartParser_Init(AIMCallback aim_cb, CircleDoneCallback circle_done_cb);
void UartParser_RxByte(uint8_t byte);
void UartParser_Process(void);
void UartParser_SendString(const char *str);

#endif
