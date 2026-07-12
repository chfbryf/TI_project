#include "uart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ti_msp_dl_config.h"

/* 环形缓冲区 */
static uint8_t rx_buf[UART_RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

/* 工作缓冲区 */
static uint8_t frame_buf[UART_RX_BUF_SIZE];
static uint16_t frame_len = 0;

static CenterCallback center_callback = NULL;
static BlobCallback blob_callback = NULL;
static CircleCallback circle_callback = NULL;

/* 内部函数：提取并解析一帧 */
static void ParseFrame(const uint8_t *data, uint16_t len);

void UartParser_Init(CenterCallback center_cb,
                     BlobCallback blob_cb,
                     CircleCallback circle_cb)
{
    center_callback = center_cb;
    blob_callback = blob_cb;
    circle_callback = circle_cb;

    /* 启动 UART 接收中断 */
    DL_UART_enableInterrupt(UART_0_INST, DL_UART_INTERRUPT_RX);
}

void UartParser_RxByte(uint8_t byte)
{
    uint16_t next = (rx_head + 1) % UART_RX_BUF_SIZE;
    if (next != rx_tail) {
        rx_buf[rx_head] = byte;
        rx_head = next;
    }
}

void UartParser_Process(void)
{
    while (rx_tail != rx_head) {
        uint8_t byte = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % UART_RX_BUF_SIZE;

        if (byte == '\n') {
            if (frame_len > 0) {
                frame_buf[frame_len] = '\0';
                ParseFrame(frame_buf, frame_len);
                frame_len = 0;
            }
        } else {
            if (frame_len < UART_RX_BUF_SIZE - 1) {
                frame_buf[frame_len++] = byte;
            } else {
                frame_len = 0;   // 帧过长，丢弃
            }
        }
    }
}

static void ParseFrame(const uint8_t *data, uint16_t len)
{
    if (len < 3) return;

    // 处理 C:[x,y]
    if (data[0] == 'C' && data[1] == ':') {
        float x, y;
        if (sscanf((const char *)data, "C:[%f,%f]", &x, &y) == 2) {
            if (center_callback) {
                center_callback(x, y);
            }
        }
    }
    // 处理 B:[x,y]  (新增)
    else if (data[0] == 'B' && data[1] == ':') {
        float x, y;
        if (sscanf((const char *)data, "B:[%f,%f]", &x, &y) == 2) {
            if (blob_callback) {
                blob_callback(x, y);
            }
        }
    }
    // 处理 P:[x0:y0,x1:y1,...]
    else if (data[0] == 'P' && data[1] == ':') {
        uint16_t x_arr[24];
        uint16_t y_arr[24];
        uint8_t count = 0;

        const char *ptr = (const char *)data + 2;  // 跳过 "P:"
        if (*ptr == '[') ptr++;
        while (*ptr && count < 24) {
            unsigned int x, y;
            if (sscanf(ptr, "%u:%u", &x, &y) == 2) {
                x_arr[count] = (uint16_t)x;
                y_arr[count] = (uint16_t)y;
                count++;
                ptr = strchr(ptr, ',');
                if (ptr) ptr++;
                else break;
            } else {
                break;
            }
        }
        if (count > 0 && circle_callback) {
            circle_callback(x_arr, y_arr, count);
        }
    }
}

void UART0_IRQHandler(void)
{
    if (DL_UART_Main_getRawInterruptStatus(UART_0_INST, DL_UART_INTERRUPT_RX) &
        DL_UART_INTERRUPT_RX) {
        uint8_t byte = DL_UART_Main_receiveData(UART_0_INST);
        UartParser_RxByte(byte);
    }
}
