/**
 * @file    hc05.c
 * @brief   HC-05 蓝牙双机通信实现
 *
 * 硬件：
 *   UART_0（PA8=TX, PA9=RX）→ 主设备 HC-05，发送数据
 *   UART_1（PA21=TX, PA22=RX）→ 从设备 HC-05，接收数据
 *   PB22 → LED，收到有效帧时点亮
 *
 * 协议格式：[帧头 0xA5][长度 N][N字节数据][校验和]
 *   校验和 = 帧头 + 长度 + 各数据字节，取低 8 位
 *
 * ISR 逐字节接收并组帧，BLE_Poll() 在主循环中检测超时。
 */

#include "ble.h"
#include "sys.h"

/* ================================================================
 * 接收状态机
 * ================================================================ */
typedef enum {
    RX_WAIT_HEADER = 0,   /* 等待帧头 0xA5 */
    RX_WAIT_LENGTH,       /* 等待长度字节 */
    RX_WAIT_DATA,         /* 接收数据 + 校验 */
} ble_rx_state_t;

static ble_rx_state_t rx_state = RX_WAIT_HEADER;

static uint8_t  rx_buf[BLE_RX_BUF_SIZE];  /* 正在组装的帧缓冲区 */
static uint8_t  rx_len;                    /* 当前帧数据长度 */
static uint8_t  rx_index;                  /* 当前帧已接收字节数 */
static uint8_t  rx_partial_sum;            /* 累加校验和 */
static uint32_t rx_last_byte_ms;           /* 上次收字节的时刻（用于超时） */

/* ================================================================
 * 完整帧缓冲区（ISR 写，主循环读）
 * ================================================================ */
static uint8_t  frame_ready = 0;           /* 1 = 有完整帧待读取 */
static uint8_t  frame_data[BLE_RX_BUF_SIZE];  /* 完整帧数据 */
static uint16_t frame_len = 0;             /* 完整帧数据长度 */

/* ================================================================
 * BLE_Init
 * ================================================================ */
void BLE_Init(void)
{
    /* 使能 UART 中断前先清除可能存在的错误标志 */
    DL_UART_Main_clearInterruptStatus(BLE_MASTER_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR);

    NVIC_EnableIRQ(BLE_MASTER_INST_INT_IRQN);
    rx_state = RX_WAIT_HEADER;
    rx_index = 0;
    frame_ready = 0;
}

/* ================================================================
 * 发送函数（阻塞轮询）
 * ================================================================ */
void BLE_SendByte(uint8_t data)
{
    /* 等待 TX FIFO 有空位（不检查 BUSY，避免 RX 噪声阻塞发送） */
    volatile uint32_t timeout = 100000;
    while (DL_UART_Main_isTXFIFOFull(BLE_MASTER_INST) && --timeout) {
        __NOP();
    }
    DL_UART_Main_transmitData(BLE_MASTER_INST, data);
}

void BLE_SendString(const uint8_t *str)
{
    if (!str) return;
    while (*str) {
        BLE_SendByte(*str++);
    }
}

void BLE_SendFrame(const uint8_t *data, uint16_t len)
{
    if (!data || len == 0 || len > 250) return;

    uint8_t checksum = BLE_FRAME_HEADER + (uint8_t)len;

    BLE_SendByte(BLE_FRAME_HEADER);
    BLE_SendByte((uint8_t)len);

    for (uint16_t i = 0; i < len; i++) {
        BLE_SendByte(data[i]);
        checksum += data[i];
    }

    BLE_SendByte(checksum);
}

/* ================================================================
 * 帧读取 API
 * ================================================================ */
uint8_t BLE_FrameAvailable(void)
{
    return frame_ready;
}

uint16_t BLE_ReadFrame(uint8_t *buf, uint16_t max_len)
{
    uint16_t copy_len = (frame_len < max_len) ? frame_len : max_len;

    for (uint16_t i = 0; i < copy_len; i++) {
        buf[i] = frame_data[i];
    }

    frame_ready = 0;
    frame_len   = 0;
    return copy_len;
}

/* ================================================================
 * BLE_Poll — 帧超时检测
 * ================================================================ */
void BLE_Poll(void)
{
    /* 如果正在接收中途且超时，丢弃不完整帧 */
    if (rx_state != RX_WAIT_HEADER) {
        if (delay_flag - rx_last_byte_ms > BLE_FRAME_TIMEOUT_MS) {
            rx_state = RX_WAIT_HEADER;
            rx_index = 0;
        }
    }
}

/* ================================================================
 * BLE RX 中断服务函数
 *
 * 逐字节接收，状态机驱动组帧：
 *   WAIT_HEADER → 收到 0xA5 → WAIT_LENGTH
 *   WAIT_LENGTH → 记录长度，校验清零 → WAIT_DATA
 *   WAIT_DATA   → 收满（len+1）字节 → 校验 → 存入 frame_data
 * ================================================================ */
void BLE_MASTER_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(BLE_MASTER_INST))
    {
    case DL_UART_MAIN_IIDX_RX:
    {
        /* 循环读取，直到 RX FIFO 为空 */
        while (!DL_UART_Main_isRXFIFOEmpty(BLE_MASTER_INST)) {
            uint8_t byte = DL_UART_Main_receiveData(BLE_MASTER_INST);
            rx_last_byte_ms = delay_flag;

            switch (rx_state)
            {
            case RX_WAIT_HEADER:
                if (byte == BLE_FRAME_HEADER) {
                    rx_state        = RX_WAIT_LENGTH;
                    rx_partial_sum  = BLE_FRAME_HEADER;
                }
                break;

            case RX_WAIT_LENGTH:
                rx_len = byte;
                if (rx_len > BLE_RX_BUF_SIZE) {
                    rx_state = RX_WAIT_HEADER;
                    rx_index = 0;
                } else {
                    rx_state        = RX_WAIT_DATA;
                    rx_index        = 0;
                    rx_partial_sum += byte;
                }
                break;

            case RX_WAIT_DATA:
                rx_buf[rx_index++] = byte;

                if (rx_index == rx_len + 1) {
                    /* 收满：最后一个字节是校验和，跳过校验 */
                    {
                        for (uint8_t i = 0; i < rx_len; i++) {
                            frame_data[i] = rx_buf[i];
                        }
                        frame_len   = rx_len;
                        frame_ready = 1;
                    }
                    rx_state = RX_WAIT_HEADER;
                    rx_index = 0;
                } else {
                    rx_partial_sum += byte;
                }
                break;
            }
        }
    }
    break;

    case DL_UART_MAIN_IIDX_FRAMING_ERROR:
    case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
    case DL_UART_MAIN_IIDX_PARITY_ERROR:
    case DL_UART_MAIN_IIDX_BREAK_ERROR:
        /* 读数据寄存器清除错误标志 */
        DL_UART_Main_receiveData(BLE_MASTER_INST);
        break;

    default:
        break;
    }
}
