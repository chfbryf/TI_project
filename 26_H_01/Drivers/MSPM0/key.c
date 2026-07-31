/**
 * @file    key.c
 * @brief   按键扫描与工作逻辑
 */

#include "sys.h"


/* ——— 按键消抖状态机（非阻塞，基于 delay_flag 1ms 时基） ——— */
#define KEY_DEBOUNCE_MS 20

typedef enum {
    KEY_IDLE = 0,
    KEY_PRESS_DEBOUNCE,
    KEY_PRESSED,
    KEY_RELEASE_DEBOUNCE,
} key_debounce_state_t;

/* 每个按键独立的状态和时间戳 */
static key_debounce_state_t k1_state = KEY_IDLE;
static key_debounce_state_t k2_state = KEY_IDLE;
static key_debounce_state_t k3_state = KEY_IDLE;
static uint32_t k1_time, k2_time, k3_time;

/**
 * @brief 单键消抖扫描（每次主循环调用）
 * @param port     GPIO 端口
 * @param pin      GPIO 引脚
 * @param state    当前状态指针
 * @param time     时间戳指针
 * @return 1 = 完成一次完整的按下-释放周期，0 = 无事件或等待中
 */
static uint8_t key_scan_pin(GPIO_Regs *port, uint32_t pin,
                             key_debounce_state_t *state, uint32_t *time)
{
    uint8_t pressed = (DL_GPIO_readPins(port, pin) == 0);

    switch (*state) {
    case KEY_IDLE:
        if (pressed) {
            *state = KEY_PRESS_DEBOUNCE;
            *time  = delay_flag;
        }
        break;

    case KEY_PRESS_DEBOUNCE:
        if (!pressed) {
            *state = KEY_IDLE;                          /* 毛刺，回退 */
        } else if (delay_flag - *time >= KEY_DEBOUNCE_MS) {
            *state = KEY_PRESSED;                       /* 确认按下 */
        }
        break;

    case KEY_PRESSED:
        if (!pressed) {
            *state = KEY_RELEASE_DEBOUNCE;
            *time  = delay_flag;
        }
        break;

    case KEY_RELEASE_DEBOUNCE:
        if (pressed) {
            *state = KEY_PRESSED;                       /* 毛刺，回退 */
        } else if (delay_flag - *time >= KEY_DEBOUNCE_MS) {
            *state = KEY_IDLE;
            return 1;                                   /* 完整周期完成 */
        }
        break;
    }
    return 0;
}

/**
 * @brief 按键扫描函数（非阻塞）
 * @return uint8_t 刚完成按下-释放周期的按键编号（1-3），无事件返回 0
 */
uint8_t Key_GetNum(void)
{
    if (key_scan_pin(KEY3_PORT, KEY3_KEY_3_PIN, &k3_state, &k3_time)) return 3;
    if (key_scan_pin(KEY2_PORT, KEY2_KEY_2_PIN, &k2_state, &k2_time)) return 2;
    if (key_scan_pin(KEY1_PORT, KEY1_KEY_1_PIN, &k1_state, &k1_time)) return 1;
    return 0;
}

/**
 * @brief 按键工作函数
 */
void key_work(void) 
{
    key.keynum = Key_GetNum();
    if (key.keynum == 1) {
        key.task_id++;
        if (key.task_id > 4)
            key.task_id = 1;
    }
    if (key.keynum == 2) {
        key.start = 1;
    }
    if (key.keynum == 3) {
        key.quan++;
        if (key.quan > 5)
            key.quan = 0;
    }
}
