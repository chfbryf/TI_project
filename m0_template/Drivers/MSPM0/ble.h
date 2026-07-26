/**
 * @file    hc05.h
 * @brief   HC-05 蓝牙双机通信模块
 *
 * 通信方式：中断逐字节接收，阻塞轮询发送
 * 协议格式：[帧头 0xA5][长度 N][N字节数据][校验和]
 *
 * 硬件：
 *   UART_0（PA8=TX，PA9=RX）→ 主设备 HC-05，发送数据
 *   UART_1（PA21=TX，PA22=RX）→ 从设备 HC-05，接收数据
 *   PB22 → LED，收到有效帧时点亮
 */

#ifndef BLE_H
#define BLE_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ---------- 帧协议常量 ---------- */
#define BLE_FRAME_HEADER    0xA5      /* 帧头标识 */
#define BLE_RX_BUF_SIZE     128       /* 接收缓冲区大小 */
#define BLE_FRAME_TIMEOUT_MS 50       /* 帧超时（ms），超过此间隔认为一帧结束 */

/* ---------- 遥控命令 ---------- */
#define BLE_CMD_FORWARD     0x01      /* 前进，参数: 1字节速度 */
#define BLE_CMD_BACKWARD    0x02      /* 后退，参数: 1字节速度 */
#define BLE_CMD_LEFT        0x03      /* 左转，参数: 1字节速度 */
#define BLE_CMD_RIGHT       0x04      /* 右转，参数: 1字节速度 */
#define BLE_CMD_STOP        0x05      /* 停止 */

/* ---------- API ---------- */

/**
 * @brief 初始化 BLE 模块（使能 UART RX 中断）
 */
void BLE_Init(void);

/**
 * @brief 发送单字节（阻塞）
 */
void BLE_SendByte(uint8_t data);

/**
 * @brief 发送字符串（阻塞）
 */
void BLE_SendString(const uint8_t *str);

/**
 * @brief 发送带帧协议的数据包
 *        自动添加帧头、长度、校验和
 *
 * @param data  数据指针
 * @param len   数据长度（<= 250，不含帧头/长度/校验占用的3字节开销）
 */
void BLE_SendFrame(const uint8_t *data, uint16_t len);

/**
 * @brief 检查是否有新的完整帧到达
 * @return 1=有新帧，0=无
 */
uint8_t BLE_FrameAvailable(void);

/**
 * @brief 读取最新一帧的数据
 * @param buf      输出缓冲区
 * @param max_len  缓冲区最大容量
 * @return 实际读取的字节数
 */
uint16_t BLE_ReadFrame(uint8_t *buf, uint16_t max_len);

/**
 * @brief 处理 BLE 接收（在主循环中定期调用，用于帧超时检测）
 */
void BLE_Poll(void);

#endif /* BLE_H */
