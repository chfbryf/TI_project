/**
 * @file    vision.c
 * @brief   视觉颜色识别模块实现
 */

#include "vision.h"
#include "ti_msp_dl_config.h"
#include "speed_ctrl.h"       /* g_target_speed_L/R */

/* ================================================================
 * 颜色 → 差速动作 查找表
 *
 * C99 指定初始化器，其余位置自动 {0,0} → 停车。
 * ================================================================ */
static const color_action_t color_actions[256] = {
    ['R'] = { 0.0f,  0.0f},   /* 红 → 停止     */
    ['G'] = { 0.5f,  0.5f},   /* 绿 → 直行     */
    ['B'] = {-0.5f, -0.5f},   /* 蓝 → 后退     */
};

/* ---- 全局状态 ---- */
volatile char g_last_color = 0;

/* ================================================================
 * Vision_Poll
 *
 * 轮询 vision(UART1, PA9=RX) 接收 FIFO。
 * 识别 'R'/'G'/'B' 三个有效指令，其余字符忽略。
 * 非阻塞，无数据时立即返回。
 * ================================================================ */
void Vision_Poll(void)
{
    while (!DL_UART_isRXFIFOEmpty(vision_INST))
    {
        char c = (char)DL_UART_receiveData(vision_INST);
        if (c == 'R' || c == 'G' || c == 'B')
            g_last_color = c;
    }
}

/* ================================================================
 * Vision_Apply
 *
 * 根据 g_last_color 设置左右轮目标速度：
 *   非零 → 查表写入 g_target_speed_L/R（由 50ms 速度环 ISR 消费）
 *   零   → 左右均设 0（停车）
 * ================================================================ */
void Vision_Apply(void)
{
    if (g_last_color != 0)
    {
        g_target_speed_L = color_actions[(unsigned char)g_last_color].left;
        g_target_speed_R = color_actions[(unsigned char)g_last_color].right;
    }
    else
    {
        g_target_speed_L = 0.0f;
        g_target_speed_R = 0.0f;
    }
}
