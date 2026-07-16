#include "sensor2.h"
#include "sys.h"

int16_t err2;
volatile uint8_t black_detected;

#define BLACK_DEBOUNCE_MS  50

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

    /* 8路灰度 0=黑线 1=白色 */
    if      ((Digtal & 0x18) == 0)       err2 =   0;  /* bit4+3 同时黑 → 居中 */
    else if ((Digtal & 0x17) == 0)       err2 =  100;  /* 右边多点同时黑 → 强左修 */
    else if ((Digtal & 0xE8) == 0)       err2 = -100;  /* 左边多点同时黑 → 强右修 */
    else if (!(Digtal & ~0xef))          err2 =   5;  /* bit4 单独黑 → 偏右 */
    else if (!(Digtal & ~0xf7))          err2 =  -5;  /* bit3 单独黑 → 偏左 */
    else if (!(Digtal & ~0xfb))          err2 =  10;
    else if (!(Digtal & ~0xfd))  err2 =  20;
    else if (!(Digtal & ~0xfe))  err2 =  30;
    else if (!(Digtal & ~0xdf))  err2 = -10;
    else if (!(Digtal & ~0xbf))  err2 = -20;
    else if (!(Digtal & ~0x7f))  err2 = -30;
    else                          err2 =   0;
}

int16_t Err2(void)
{
    return err2;
}

