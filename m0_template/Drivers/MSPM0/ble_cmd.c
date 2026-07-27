/**
 * @file    ble_cmd.c
 * @brief   蓝牙命令层 — 命令表派发与各命令 handler 实现
 *
 * 每增加一个新命令：
 *   1. 在 ble_cmd.h 中定义命令ID宏
 *   2. 在本文件中写一个 static handler 函数
 *   3. 在 cmd_table[] 中加一行
 *   4. 主循环代码（main.c）无需修改
 */

#include "ble_cmd.h"
#include "sys.h"

/* ================================================================
 * handler 前向声明
 * ================================================================ */
static void cmd_forward(const uint8_t *data, uint8_t len);
static void cmd_backward(const uint8_t *data, uint8_t len);
static void cmd_left(const uint8_t *data, uint8_t len);
static void cmd_right(const uint8_t *data, uint8_t len);
static void cmd_stop(const uint8_t *data, uint8_t len);
static void cmd_set_speed(const uint8_t *data, uint8_t len);
static void cmd_turn_left(const uint8_t *data, uint8_t len);
static void cmd_turn_right(const uint8_t *data, uint8_t len);

/* ================================================================
 * 命令表（const → 编译后放 Flash，不占 RAM）
 * ================================================================ */
static const ble_cmd_entry_t cmd_table[] = {
    /* 运动控制 */
    { BLE_CMD_FORWARD,    "forward",    cmd_forward    },
    { BLE_CMD_BACKWARD,   "backward",   cmd_backward   },
    { BLE_CMD_LEFT,       "left",       cmd_left       },
    { BLE_CMD_RIGHT,      "right",      cmd_right      },
    { BLE_CMD_STOP,       "stop",       cmd_stop       },

    /* 参数配置 */
    { BLE_CMD_SET_SPEED,  "set_speed",  cmd_set_speed  },

    /* 高级动作 */
    { BLE_CMD_TURN_LEFT,  "turn_left",  cmd_turn_left  },
    { BLE_CMD_TURN_RIGHT, "turn_right", cmd_turn_right },
};

#define CMD_TABLE_SIZE (sizeof(cmd_table) / sizeof(cmd_table[0]))

/* ================================================================
 * ble_cmd_dispatch — 查表派发（不进 switch，不改主循环）
 * ================================================================ */
void ble_cmd_dispatch(const uint8_t *data, uint8_t len)
{
    if (!data || len == 0) return;

    uint8_t cmd_id = data[0];

    for (uint16_t i = 0; i < CMD_TABLE_SIZE; i++) {
        if (cmd_table[i].cmd_id == cmd_id) {
            /* 跳过 cmd_id 字节，将参数区传给 handler */
            cmd_table[i].handler(&data[1], len - 1);
            return;
        }
    }
}

/* =================================================================
 * handler 函数实现
 *
 * 注意：handler 在主循环上下文中执行，必须快速返回！
 *       耗时动作（如转弯）使用状态机异步执行，禁止 delay。
 * ================================================================= */

/* —————————————— 运动控制 —————————————— */

/** 前进: [speed_level]  speed_level=0~5，对应 0~1000mm/s */
static void cmd_forward(const uint8_t *data, uint8_t len)
{
    if (len < 1) return;
    speed(data[0]);
    g_target_speed_L =  (float)base_speed / 1000.0f;
    g_target_speed_R =  (float)base_speed / 1000.0f;
}

/** 后退: [speed_level] */
static void cmd_backward(const uint8_t *data, uint8_t len)
{
    if (len < 1) return;
    speed(data[0]);
    g_target_speed_L = -(float)base_speed / 1000.0f;
    g_target_speed_R = -(float)base_speed / 1000.0f;
}

/** 左转: [speed_level] */
static void cmd_left(const uint8_t *data, uint8_t len)
{
    if (len < 1) return;
    speed(data[0]);
    g_target_speed_L = -(float)base_speed / 1000.0f;
    g_target_speed_R =  (float)base_speed / 1000.0f;
}

/** 右转: [speed_level] */
static void cmd_right(const uint8_t *data, uint8_t len)
{
    if (len < 1) return;
    speed(data[0]);
    g_target_speed_L =  (float)base_speed / 1000.0f;
    g_target_speed_R = -(float)base_speed / 1000.0f;
}

/** 停止: 无参数 */
static void cmd_stop(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;
    motor_stop();
}

/* —————————————— 参数配置 —————————————— */

/** 设置基础速度: [speed_level]  仅改速度，不改变运动方向 */
static void cmd_set_speed(const uint8_t *data, uint8_t len)
{
    if (len < 1) return;
    speed(data[0]);
}

/* —————————————— 高级动作 —————————————— */

/** 左转90度: 无参数，触发已有 TurnState 状态机 */
static void cmd_turn_left(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;
    if (turn_state != TURN_IDLE) return;  /* 正在转弯中，忽略 */
    turn_direction = TURN_DIR_LEFT;
    turn_state = TURN_SPIN;
}

/** 右转90度: 无参数 */
static void cmd_turn_right(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;
    if (turn_state != TURN_IDLE) return;
    turn_direction = TURN_DIR_RIGHT;
    turn_state = TURN_SPIN;
}
