#include "sensor2.h"
#include "sys.h"

int16_t err2;
volatile uint8_t black_detected;

#define BLACK_DEBOUNCE_MS  30

/**
 * @file    sensor2.c
 * @brief   灰度传感器误差计算（加权质心法）
 * @brief   8路传感器从左到右检测到黑线时对应的 Digtal 值
 * @brief   bit=0 表示黑线，每路对应位置：
 * @brief   [左1]0xFE [左2]0xFD [左3]0xFB [左4]0xF7
 * @brief   [右4]0xEF [右3]0xDF [右2]0xBF [右1]0x7F
 */

void Get_err2(void)
{
    static volatile uint32_t black_start = 0;

    /* 黑线检测消抖 */
    if ((Digtal & 0xFF) != 0xFF)
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


    /* 全黑(Digtal==0x00)或全白(Digtal==0xFF) */
    if (Digtal == 0x00 || Digtal == 0xFF) {
        err2 = 0;
        return;
    }

    /* 加权质心法：8路传感器位置 × 见黑标志
     * bit0(左1) → +7, bit1(左2) → +5, bit2(左3) → +3, bit3(左4) → +1,
     * bit4(右4) → -1, bit5(右3) → -3, bit6(右2) → -5, bit7(右1) → -7
     * bit=0 表示见到黑线，对权重求和取平均 */
    {
        static const int8_t weight[8] = {7, 5, 3, 1, -1, -3, -5, -7};
        int16_t sum = 0;
        int8_t  cnt = 0;

        for (uint8_t i = 0; i < 8; i++) {
            if (!(Digtal & (1 << i))) {   /* bit=0 → 黑线 */
                sum += weight[i];
                cnt++;
            }
        }

        err2 = (cnt > 0) ? (sum / cnt) : 0;
    }
}

int16_t Err2(void)
{
    return err2;
}

