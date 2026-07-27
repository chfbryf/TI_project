/**
 * @file    ble_cmd.c
 * @brief   蓝牙命令层 — 命令表派发 + 控制源管理
 *
 * 架构：
 *   所有写 g_target_speed_L/R 的地方都先检查 g_ctrl_source，
 *   只有当前控制源才允许写入，避免 BLE / 循迹 / 视觉互相覆盖。
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
 * 全局控制源
 * ================================================================ */
volatile ctrl_source_t g_ctrl_source = CTRL_SRC_NONE;

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
static void cmd_local_track(const uint8_t *data, uint8_t len);
static void cmd_local_vision(const uint8_t *data, uint8_t len);
static void cmd_ble_ctrl(const uint8_t *data, uint8_t len);

/* ================================================================
 * 运动控制辅助宏（避免重复代码）
 * ================================================================ */
#define BLE_SET_SPEED_VIA_SPEEDFN(speed_level)   do { \
    g_ctrl_source = CTRL_SRC_BLE;                      \
    speed(speed_level);                                \
} while(0)

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

    /* 模式切换 */
    { BLE_CMD_LOCAL_TRACK, "local_track",  cmd_local_track  },
    { BLE_CMD_LOCAL_VISION,"local_vision", cmd_local_vision },
    { BLE_CMD_BLE_CTRL,    "ble_ctrl",     cmd_ble_ctrl     },
};

#define CMD_TABLE_SIZE (sizeof(cmd_table) / sizeof(cmd_table[0]))

/* ================================================================
 * 帧去重（避免主机重复发送导致从机反复执行相同命令）
 * ================================================================ */
static uint8_t  last_frame[32];
static uint8_t  last_frame_len;

static int frame_is_same(const uint8_t *data, uint8_t len)
{
    if (len != last_frame_len) return 0;
    for (uint8_t i = 0; i < len; i++) {
        if (data[i] != last_frame[i]) return 0;
    }
    return 1;
}

static void frame_remember(const uint8_t *data, uint8_t len)
{
    last_frame_len = (len < sizeof(last_frame)) ? len : sizeof(last_frame);
    for (uint8_t i = 0; i < last_frame_len; i++) {
        last_frame[i] = data[i];
    }
}

/* ================================================================
 * ble_cmd_dispatch — 查表派发
 * ================================================================ */
void ble_cmd_dispatch(const uint8_t *data, uint8_t len)
{
    if (!data || len == 0) return;

    /* 与上一帧完全相同 → 跳过 */
    if (frame_is_same(data, len)) return;
    frame_remember(data, len);

    uint8_t cmd_id = data[0];

    for (uint16_t i = 0; i < CMD_TABLE_SIZE; i++) {
        if (cmd_table[i].cmd_id == cmd_id) {
            cmd_table[i].handler(&data[1], len - 1);
            return;
        }
    }
}

/* =================================================================
 * handler 函数实现
 *
 * 规则：
 *   - 运动命令（forward/backward/left/right/stop）→ 接管为 BLE 控制
 *   - 参数命令（set_speed）→ 仅在 BLE 控制时生效
 *   - 动作命令（turn）→ 任何模式下都生效（触发状态机）
 *   - 模式命令（local_track/vision/ble_ctrl）→ 仅切换控制源
 *   - 所有 handler 必须快速返回，禁止 delay
 * ================================================================= */

/* —————————————— 运动控制（抢占控制权为 BLE） —————————————— */

static void cmd_forward(const uint8_t *data, uint8_t len)
{
    if (len < 1) return;
    BLE_SET_SPEED_VIA_SPEEDFN(data[0]);
    g_target_speed_L =  (float)base_speed / 1000.0f;
    g_target_speed_R =  (float)base_speed / 1000.0f;
}

static void cmd_backward(const uint8_t *data, uint8_t len)
{
    if (len < 1) return;
    BLE_SET_SPEED_VIA_SPEEDFN(data[0]);
    g_target_speed_L = -(float)base_speed / 1000.0f;
    g_target_speed_R = -(float)base_speed / 1000.0f;
}

static void cmd_left(const uint8_t *data, uint8_t len)
{
    if (len < 1) return;
    BLE_SET_SPEED_VIA_SPEEDFN(data[0]);
    g_target_speed_L = -(float)base_speed / 1000.0f;
    g_target_speed_R =  (float)base_speed / 1000.0f;
}

static void cmd_right(const uint8_t *data, uint8_t len)
{
    if (len < 1) return;
    BLE_SET_SPEED_VIA_SPEEDFN(data[0]);
    g_target_speed_L =  (float)base_speed / 1000.0f;
    g_target_speed_R = -(float)base_speed / 1000.0f;
}

static void cmd_stop(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;
    g_ctrl_source = CTRL_SRC_BLE;
    motor_stop();
}

/* —————————————— 参数配置（仅 BLE 模式生效） —————————————— */

static void cmd_set_speed(const uint8_t *data, uint8_t len)
{
    if (len < 1) return;
    if (g_ctrl_source != CTRL_SRC_BLE) return;
    speed(data[0]);
}

/* —————————————— 高级动作（任何模式都生效，触发状态机） —————————————— */

static void cmd_turn_left(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;
    if (turn_state != TURN_IDLE) return;
    turn_direction = TURN_DIR_LEFT;
    turn_state = TURN_SPIN;
}

static void cmd_turn_right(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;
    if (turn_state != TURN_IDLE) return;
    turn_direction = TURN_DIR_RIGHT;
    turn_state = TURN_SPIN;
}

/* —————————————— 模式切换（不改变速度，只切换控制源） —————————————— */

static void cmd_local_track(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;
    g_ctrl_source = CTRL_SRC_LOCAL_TRACK;
    Tracking_SpeedLoop_Reset();
}

static void cmd_local_vision(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;
    g_ctrl_source = CTRL_SRC_LOCAL_VISION;
}

static void cmd_ble_ctrl(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;
    g_ctrl_source = CTRL_SRC_BLE;
}
