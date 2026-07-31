#ifndef IR_TRACKING_H_
#define IR_TRACKING_H_

#include <stdint.h>

/**
 * @brief 初始化 IR8 红外循迹传感器（UART1，115200 波特率）
 * @note  发送 "DS" 配置传感器为数字信号模式
 */
void IR_Init(void);

/**
 * @brief 读取 8 路红外传感器当前值
 * @param data 输出数组 data[0..7]
 *             1 = 检测到黑线，0 = 白色背景
 *             data[0] = 左1(x1), data[7] = 右8(x8)
 */
void IR_Read(uint8_t data[8]);

#endif
