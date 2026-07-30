#ifndef IR_TRACKING_H_
#define IR_TRACKING_H_

#include <stdint.h>

/**
 * @brief 初始化红外循迹传感器（UART1，115200 波特率）
 * @note  发送 $0,0,1# 配置传感器为正常模式 + 仅数字输出
 */
void IR_Init(void);

/**
 * @brief 读取 8 路红外传感器当前值
 * @param data 输出数组 data[0..7]
 *             0 = 检测到黑线，1 = 白色背景
 *             data[0] = 左1(x1), data[7] = 右8(x8)
 */
void IR_Read(uint8_t data[8]);

#endif
