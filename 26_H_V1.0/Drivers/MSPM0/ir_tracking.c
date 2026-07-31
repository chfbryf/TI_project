/**
 * @file    ir_tracking.c
 * @brief   IR8 八路红外循迹传感器 UART 驱动
 * @brief   协议: MCU→传感器 "DS"（数字模式）
 * @brief         传感器→MCU 0xAA 0xAA [16 bytes data] 0xFF 0xFF
 * @brief         16字节 = 8通道 × 2字节(高+低)，DS模式: 传感器输出 1=黑线 0=白
 * @brief         代码内部统一: 1=黑线, 0=白
 * @brief   UART: 115200 8N1，使用 UART1 (PA9=RX, PA8=TX)
 */

#include "sys.h"
#include <string.h>

static uint8_t ir_data[8];       /* 8路传感器值: 1=黑线, 0=白 */

/* UART RX 解析状态机: 帧格式 AA AA [16 bytes] FF FF */
static enum { IR_WAIT_HDR1, IR_WAIT_HDR2, IR_DATA, IR_WAIT_FTR1, IR_WAIT_FTR2 } ir_state = IR_WAIT_HDR1;
static uint8_t ir_buf[16];
static uint8_t ir_idx;

/**
 * @brief 逐字节处理 IR8 串口数据帧
 */
static void IR_ProcessRxByte(uint8_t ch)
{
    switch (ir_state) {

    case IR_WAIT_HDR1:
        if (ch == 0xAA) {
            ir_state = IR_WAIT_HDR2;
        }
        break;

    case IR_WAIT_HDR2:
        if (ch == 0xAA) {
            ir_state = IR_DATA;
            ir_idx = 0;
        } else {
            ir_state = IR_WAIT_HDR1;
        }
        break;

    case IR_DATA:
        ir_buf[ir_idx++] = ch;
        if (ir_idx >= 16) {
            ir_state = IR_WAIT_FTR1;
        }
        break;

    case IR_WAIT_FTR1:
        if (ch == 0xFF) {
            ir_state = IR_WAIT_FTR2;
        } else {
            ir_state = IR_WAIT_HDR1;
        }
        break;

    case IR_WAIT_FTR2:
        if (ch == 0xFF) {
            /* 帧结束，解析 8 路数据
             * 每通道占 2 字节(高+低)，DS模式值=0或1 */
            for (uint8_t i = 0; i < 8; i++) {
                uint16_t val = ((uint16_t)ir_buf[2 * i] << 8) | ir_buf[2 * i + 1];
                ir_data[i] = (val > 0) ? 1 : 0;  /* 传感器: 1=黑线, 0=白 */
            }
        }
        ir_state = IR_WAIT_HDR1;
        break;

    default:
        ir_state = IR_WAIT_HDR1;
        break;
    }
}

void IR_Init(void)
{
    /* 清除挂起中断，防止上电误触发 */
    NVIC_ClearPendingIRQ(UART1_SENSOR_INST_INT_IRQN);

    /* 设置 RX FIFO 阈值: 每收到 1 字节即触发中断 */
    DL_UART_Main_setRXFIFOThreshold(UART1_SENSOR_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);

    /* 使能 UART1 RX 中断 */
    DL_UART_Main_enableInterrupt(UART1_SENSOR_INST, DL_UART_INTERRUPT_RX);
    NVIC_EnableIRQ(UART1_SENSOR_INST_INT_IRQN);

    /* 发送控制命令: DS = 数字信号模式 */
    const char *cmd = "DS";
    for (uint8_t i = 0; cmd[i] != '\0'; i++) {
        DL_UART_transmitDataBlocking(UART1_SENSOR_INST, cmd[i]);
    }
}

void IR_Read(uint8_t data[8])
{
    /* 始终返回最新一帧数据，无需等待新帧 */
    memcpy(data, ir_data, 8);
}

/**
 * @brief UART1 RX 中断处理: 排空 FIFO，逐字节解析帧
 */
void UART1_SENSOR_INST_IRQHandler(void)
{
    DL_UART_Main_getPendingInterrupt(UART1_SENSOR_INST);
    while (!DL_UART_isRXFIFOEmpty(UART1_SENSOR_INST)) {
        uint8_t ch = DL_UART_receiveData(UART1_SENSOR_INST);
        IR_ProcessRxByte(ch);
    }
}
