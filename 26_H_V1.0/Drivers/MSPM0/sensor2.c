/**
 * @file    sensor2.c
 * @brief   灰度传感器误差计算（加权质心法）
 * @brief   8路传感器从左到右检测到黑线时对应的 Digtal 值
 * @brief   bit=1 表示黑线，每路对应位置：
 * @brief   [左1]0x80 [左2]0x40 [左3]0x20 [左4]0x10
 * @brief   [右4]0x08 [右3]0x04 [右2]0x02 [右1]0x01
 */

#include "sys.h"

int16_t err2;
volatile uint8_t black_detected;
static int16_t last_valid_err2;   /* 丢线时保持方向用 */

#define BLACK_DEBOUNCE_MS  30

void Get_err2(void)
{
    static uint32_t black_start = 0;

    /* 黑线检测消抖: bit=1为黑线，任意bit为1表示检测到黑线 */
    if (Digtal != 0x00)
    {
        if (black_start == 0) black_start = delay_flag;
        if ((delay_flag - black_start) >= BLACK_DEBOUNCE_MS)
        {
            black_detected = 1;
        }
        else
        {
            black_detected = 0;
        }
    }
    else
    {
        black_start = 0;
        black_detected = 0;
    }


    /* 全白(0x00)或全黑(0xFF)丢线：保持上一帧误差方向 */
    if (Digtal == 0x00 || Digtal == 0xFF) {
        if (last_valid_err2 > 0)
            err2 = 5;   /* 上次偏右，继续右转找线 */
        else if (last_valid_err2 < 0)
            err2 = -5;  /* 上次偏左，继续左转找线 */
        else
            err2 = 0;
        return;
    }

    /* 加权质心法：8路传感器位置 × 见黑标志
     * bit7(左) → -7, bit6 → -5, bit5 → -3, bit4 → -1,
     * bit3 → +1, bit2 → +3, bit1 → +5, bit0(右) → +7
     * bit=1 表示见到黑线，对权重求和取平均 */
    {
        static const int8_t weight[8] = {7, 5, 3, 1, -1, -3, -5, -7};
        int16_t sum = 0;
        int8_t  cnt = 0;

        for (uint8_t i = 0; i < 8; i++) {
            if (Digtal & (1 << i)) {   /* bit=1 → 黑线 */
                sum += weight[i];
                cnt++;
            }
        }

        err2 = (cnt > 0) ? (sum / cnt) : 0;
    }

    if (err2 != 0) last_valid_err2 = err2;  /* 保存方向供丢线时使用 */
}

int16_t Err2(void)
{
    return err2;
}

