/**
 * @file    sensor2.c
 * @brief   灰度传感器误差计算（加权质心法）
 * @brief   8路传感器从左到右检测到黑线时对应的 Digtal 值
 * @brief   bit=0 表示黑线，每路对应位置：
 * @brief   [左1]0xFE [左2]0xFD [左3]0xFB [左4]0xF7
 * @brief   [右4]0xEF [右3]0xDF [右2]0xBF [右1]0x7F
 */

#include "sys.h"

int16_t err2;
static int16_t last_valid_err2;   /* 丢线时保持方向用 */

void Get_err2(void)
{
    /* 全白(0x00)或全黑(0xFF)丢线：逐帧衰减 err2 到 0，避免越偏越严重 */
    if (Digtal == 0x00 || Digtal == 0xFF) {
        if (last_valid_err2 > 1) {
            last_valid_err2 -= 1;
        } else if (last_valid_err2 < -1) {
            last_valid_err2 += 1;
        } else {
            last_valid_err2 = 0;
        }
        err2 = last_valid_err2;
        return;
    }

    /* 加权质心法：8路传感器位置 × 见黑标志
     * bit7(左) → -7, bit6 → -5, bit5 → -3, bit4 → -1,
     * bit3 → +1, bit2 → +3, bit1 → +5, bit0(右) → +7
     * bit=0 表示见到黑线，对权重求和取平均 */
    {
        static const int8_t weight[8] = {7, 5, 3, 1, -1, -3, -5, -0};
        int16_t sum = 0;
        int8_t  cnt = 0;

        for (uint8_t i = 0; i < 8; i++) {
            if (!(Digtal & (1 << i))) {   /* bit=0 → 黑线 */
                sum += weight[i];
                cnt++;
            }
        }

        err2 = (cnt > 0) ? (int16_t)((float)sum / (float)cnt) : 0;
    }

    if (err2 != 0) last_valid_err2 = err2;  /* 保存方向供丢线时使用 */
}

int16_t Err2(void)
{
    return err2;
}

