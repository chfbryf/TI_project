/**
 * @file    ir_tracking.c
 * @brief   八路红外循迹传感器 UART 驱动
 * @brief   协议: MCU→传感器 $0,0,1# (正常模式+模拟关+数字开)
 * @brief         传感器→MCU $D,x1:V,x2:V,...,x8:V#
 * @brief         V='0'=黑线, V='1'=白色背景
 * @brief   UART: 115200 8N1，使用 UART1 (PA9=RX, PA8=TX)
 */

#include "sys.h"
#include <string.h>

static uint8_t ir_data[8];       /* 8路传感器值: 0=黑线, 1=白 */
static uint8_t ir_data_ready;    /* 新数据标志 */

/* UART RX 解析状态机 */
static enum { IR_WAIT_START, IR_WAIT_D, IR_PARSING } ir_state = IR_WAIT_START;
static char  ir_buf[64];
static uint8_t ir_idx;

void IR_Init(void)
{
    /* 设置 RX FIFO 阈值: 每收到 1 字节即触发中断 */
    DL_UART_Main_setRXFIFOThreshold(UART1_SENSOR_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);

    /* 使能 UART1 RX 中断 */
    DL_UART_Main_enableInterrupt(UART1_SENSOR_INST, DL_UART_INTERRUPT_RX);
    NVIC_EnableIRQ(UART1_SENSOR_INST_INT_IRQN);

    /* 发送控制命令: 正常模式 + 模拟关 + 数字开 */
    const char *cmd = "$0,0,1#";
    for (uint8_t i = 0; cmd[i] != '\0'; i++) {
        DL_UART_transmitDataBlocking(UART1_SENSOR_INST, cmd[i]);
    }
}

void IR_Read(uint8_t data[8])
{
    NVIC_DisableIRQ(UART1_SENSOR_INST_INT_IRQN);
    if (ir_data_ready) {
        memcpy(data, ir_data, 8);
        ir_data_ready = 0;
    }
    NVIC_EnableIRQ(UART1_SENSOR_INST_INT_IRQN);
}

/**
 * @brief UART1 RX 中断处理: 逐字符解析帧 $D,x1:V,...,x8:V#
 */
void UART1_SENSOR_INST_IRQHandler(void)
{
    uint8_t ch = DL_UART_receiveData(UART1_SENSOR_INST);

    switch (ir_state) {

    case IR_WAIT_START:
        if (ch == '$') {
            ir_state = IR_WAIT_D;
            ir_idx = 0;
        }
        break;

    case IR_WAIT_D:
        if (ch == 'D') {
            ir_buf[ir_idx++] = ch;
            ir_state = IR_PARSING;
        } else {
            ir_state = IR_WAIT_START;
        }
        break;

    case IR_PARSING:
        if (ch == '#') {
            /* 帧结束，解析 8 路数据
             * 格式: D,x1:V,x2:V,...,x8:V
             * 存储: buf[0]='D', buf[5+i*5]=Vi */
            for (uint8_t i = 0; i < 8; i++) {
                uint8_t pos = 5 + i * 5;
                if (pos < ir_idx) {
                    ir_data[i] = (ir_buf[pos] == '0') ? 0 : 1;
                } else {
                    ir_data[i] = 1;  /* 解析失败默认白 */
                }
            }
            ir_data_ready = 1;
            ir_state = IR_WAIT_START;
        } else {
            if (ir_idx < sizeof(ir_buf)) {
                ir_buf[ir_idx++] = ch;
            } else {
                ir_state = IR_WAIT_START;  /* 溢出: 丢包重同步 */
            }
        }
        break;
    }
}
