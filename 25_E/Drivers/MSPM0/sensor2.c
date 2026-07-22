#include "sensor2.h"
#include "sys.h"

int16_t err2;
volatile uint8_t black_detected;

#define BLACK_DEBOUNCE_MS  30

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
     * bit7(左) → -7, bit6 → -5, bit5 → -3, bit4 → -1,
     * bit3 → +1, bit2 → +3, bit1 → +5, bit0(右) → +7
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

