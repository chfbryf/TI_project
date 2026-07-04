#ifndef __UART_H
#define __UART_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* 一帧最大长度（含 P: 协议的 24 个点，约 180 字节，取 256 足够） */
#define UART_RX_BUF_SIZE   256

typedef void (*CenterCallback)(float x, float y);
typedef void (*BlobCallback)(float x, float y);
typedef void (*CircleCallback)(uint16_t *x_arr, uint16_t *y_arr, uint8_t count);

void UartParser_Init(CenterCallback center_cb,
                     BlobCallback blob_cb,
                     CircleCallback circle_cb);
void UartParser_RxByte(uint8_t byte);
void UartParser_Process(void);

#endif
