/**
 * @file    vision.c
 * @brief   MaixCam 视觉模块 UART 接收与协议解析
 *
 * UART2 (vision_INST) RX 中断驱动, 115200 8N1
 * 帧格式: $B<status>,<DDD.DD>*\n (固定 12 字节)
 *
 * 状态机逐字符解析, 完整帧到达后更新 vision 全局结构体。
 */

#include "vision.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

/* ----- 全局变量 ----- */
volatile vision_data_t vision = { false, 0.0f, false, 0 };

/* ----- UART RX 解析状态机 ----- */
typedef enum {
    VW_START,       /* 等待 '$' */
    VW_FRAME,       /* 收集中, 等待 '\n' 结束 */
} vw_state_t;

static vw_state_t vstate = VW_START;
static char  vbuf[16];       /* 帧缓冲区 */
static uint8_t vidx;         /* 缓冲区写入位置 */

/* ----- 公开 API ----- */

/**
 * @brief 初始化视觉模块 UART 接收
 * @note  UART2 硬件已由 SYSCFG_DL_vision_init() 初始化,
 *        此处只需使能 RX 中断。
 */
void Vision_Init(void)
{
    DL_UART_Main_enableInterrupt(vision_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(vision_INST_INT_IRQN);
}

/* ----- UART2 RX 中断处理 ----- */

void UART2_IRQHandler(void)
{
    uint8_t ch = DL_UART_receiveData(vision_INST);
    extern volatile uint32_t test_ms;

    switch (vstate) {

    case VW_START:
        if (ch == '$') {
            vidx = 0;
            vbuf[vidx++] = (char)ch;
            vstate = VW_FRAME;
        }
        break;

    case VW_FRAME:
        /* 收字符, 防止溢出 */
        if (vidx < (uint8_t)(sizeof(vbuf) - 1)) {
            vbuf[vidx++] = (char)ch;
        }

        if (ch == '\n') {
            /* ----- 帧完整性校验 -----
             * 期望: $B<status>,<DDD.DD>*\n = 12 字节
             * 索引: 0=$  1=B  2=status  3=,
             *       4/5/6=DDD  7=.  8/9=FF  10=*  11=\n
             */
            if (vidx == 12 &&
                vbuf[0]  == '$'  &&
                vbuf[1]  == 'B'  &&
                (vbuf[2] == '0' || vbuf[2] == '1') &&
                vbuf[3]  == ','  &&
                vbuf[7]  == '.'  &&
                vbuf[10] == '*')
            {
                /* 解析状态位 */
                vision.ball_detected = (vbuf[2] == '1');

                /* 解析距离 DDD.FF → float (cm) */
                uint16_t int_part = (uint16_t)(vbuf[4] - '0') * 100U
                                  + (uint16_t)(vbuf[5] - '0') * 10U
                                  + (uint16_t)(vbuf[6] - '0');

                uint16_t frac_part = (uint16_t)(vbuf[8] - '0') * 10U
                                   + (uint16_t)(vbuf[9] - '0');

                vision.distance_cm = (float)int_part + (float)frac_part * 0.01f;
                vision.last_update_ms = test_ms;
                vision.data_ready = true;
            }
            /* 无论校验是否通过, 都重置状态机等下一帧 */
            vstate = VW_START;
        }
        else if (vidx >= (uint8_t)(sizeof(vbuf) - 1)) {
            /* 缓冲区溢出保护: 丢弃脏帧, 重等 '$' */
            vstate = VW_START;
        }
        break;

    default:
        vstate = VW_START;
        break;
    }
}
