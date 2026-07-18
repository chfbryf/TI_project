/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "vl53l1x.h"
#include "icm42688.h"
#include <stdio.h>

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
    case DL_INTERRUPT_GROUP1_IIDX_GPIOB:
        /* VL53L1X INT (PB14) */
        if (DL_GPIO_getEnabledInterruptStatus(GPIO_VL53L1x_PORT,
                GPIO_VL53L1x_INT_PIN) & GPIO_VL53L1x_INT_PIN) {
            DL_GPIO_clearInterruptStatus(GPIO_VL53L1x_PORT,
                GPIO_VL53L1x_INT_PIN);
            VL53L1X_NotifyDataReady();
        }
        /* ICM42688 INT1 (PB15) */
        if (DL_GPIO_getEnabledInterruptStatus(INT1_PORT,
                INT1_PIN_1_PIN) & INT1_PIN_1_PIN) {
            DL_GPIO_clearInterruptStatus(INT1_PORT,
                INT1_PIN_1_PIN);
            ICM42688_NotifyDataReady();
        }
        break;
    default:
        break;
    }
}

int main(void)
{
    uint16_t distance;
    int8_t ret;
    uint32_t loop_cnt = 0;
    bool icm42688_ok = false;

    SYSCFG_DL_init();
    NVIC_EnableIRQ(GPIOB_INT_IRQn);

    /* PB14 (VL53L1X INT): 下降沿  |  PB15 (ICM42688 INT1): 上升沿（50us 高脉冲） */
    DL_GPIO_setLowerPinsPolarity(GPIOB,
        DL_GPIO_PIN_14_EDGE_FALL | DL_GPIO_PIN_15_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(GPIO_VL53L1x_PORT, GPIO_VL53L1x_INT_PIN);
    DL_GPIO_enableInterrupt(GPIO_VL53L1x_PORT, GPIO_VL53L1x_INT_PIN);
    DL_GPIO_clearInterruptStatus(INT1_PORT, INT1_PIN_1_PIN);
    DL_GPIO_enableInterrupt(INT1_PORT, INT1_PIN_1_PIN);

    VL53L1X_Init(I2C_VL53L1x_INST);
    icm42688_ok = (Init_ICM42688() == 0);

    while (1) {
        loop_cnt++;

        ret = VL53L1X_GetDistance(&distance);

        if (icm42688_ok && ICM42688_DataReady()) {
            static uint32_t last_loop = 0;
            float dt;

            g_icm42688_dataReady = false;

            /* dt = 两次 ICM42688 读取之间的实际时间（单位：秒）
             * 每次主循环约 10ms，以此估算 */
            if (last_loop == 0) {
                dt = 0.01f;
            } else {
                dt = (float)(loop_cnt - last_loop) * 0.01f;
                if (dt > 1.0f) dt = 1.0f;   /* 上限制，防止溢出后数值异常 */
            }
            last_loop = loop_cnt;

            ICM42688_ReadAndCompute(dt);
            printf("[%lu] Y:%5.1f P:%5.1f R:%5.1f | A:%.2f,%.2f,%.2f D:%dmm\r\n",
                   loop_cnt, icm42688_yaw, icm42688_pitch, icm42688_roll,
                   icm42688_acc_x, icm42688_acc_y, icm42688_acc_z,
                   (int)distance);
        } else if (ret == 0) {
            printf("[%lu] D:%dmm\r\n", loop_cnt, (int)distance);
        } else {
            printf("[%lu] --\r\n", loop_cnt);
        }

        delay_cycles(CPUCLK_FREQ / 100);
    }
}

/* ===================== printf 重定向 ===================== */
int __io_putchar(int ch)
{
    while (DL_UART_isBusy(UART_0_INST) == true);
    DL_UART_Main_transmitData(UART_0_INST, ch);
    return ch;
}

int _write(int fd, const char *ptr, int len)
{
    (void)fd;
    for (int i = 0; i < len; i++) {
        __io_putchar(ptr[i]);
    }
    return len;
}

int fputc(int ch, FILE *stream)
{
    (void)stream;
    return __io_putchar(ch);
}

int fputs(const char *restrict s, FILE *restrict stream)
{
    (void)stream;
    int char_len = 0;
    while (*s != 0) {
        __io_putchar(*s++);
        char_len++;
    }
    return char_len;
}

int puts(const char *ptr)
{
    int len = fputs(ptr, stdout);
    __io_putchar('\n');
    return len + 1;
}
