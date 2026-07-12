#include "sensor2.h"
#include "sys.h"

int16_t err2;

void Get_err2(void)
{
    /* 全黑(Digtal==0x00)或全白(Digtal==0xFF) */
    if (Digtal == 0x00 || Digtal == 0xFF) {
        err2 = 0;
        return;
    }

    /* 5位灰度: bit0~bit4 对应左→右 5个传感器 */
    unsigned char key;
    if      (Digtal & 0x10) key = 0x10;  /* bit4: 最右 */
    else if (Digtal & 0x01) key = 0x01;  /* bit0: 最左 */
    else if (Digtal & 0x08) key = 0x08;  /* bit3: 偏右 */
    else if (Digtal & 0x02) key = 0x02;  /* bit1: 偏左 */
    else if (Digtal & 0x04) key = 0x04;  /* bit2: 中间 */
    else if (Digtal & 0x18) key = 0x18;  /* bit4+3 */
    else if (Digtal & 0x03) key = 0x03;  /* bit1+0 */
    else if (Digtal & 0x0C) key = 0x0C;  /* bit3+2 */
    else if (Digtal & 0x06) key = 0x06;  /* bit2+1 */
    else                    key = 0;

    switch (key) {
        case 0x10: err2 =  -200; break;  /* ●○○○○ */
        case 0x01: err2 = 200; break;  /* ○○○○● */
        case 0x08: err2 =  -100; break;  /* ○●○○○ */
        case 0x02: err2 = 100; break;  /* ○○○●○ */
        case 0x04: err2 =    0; break;  /* ○○●○○ */
        case 0x18: err2 =  -150; break;  /* ●●○○○ */
        case 0x03: err2 = 150; break;  /* ○○○●● */
        case 0x0C: err2 =   -50; break;  /* ○●●○○ */
        case 0x06: err2 =  50; break;  /* ○○●●○ */
        default:   err2 =    0; break;  /* 多传感器或无效 → 0 */
    }
}

int16_t Err2(void)
{
    return err2;
}

