/**
 * @file    ble_cmd.h
 * @brief   蓝牙命令层 — 命令定义、命令表派发
 *
 * 负责"收到什么命令 → 执行什么动作"，与传输层（ble.h/ble.c）解耦。
 *
 * 命令ID编号规范：
 *   0x01~0x0F  运动控制（forward/backward/left/right/stop）
 *   0x10~0x2F  参数配置（set_speed, set_pid ...）
 *   0x30~0x4F  高级动作（turn_90, turn_180 ...）
 *   0x50~0x6F  外设控制（beep, led, servo ...）
 *   0x70~0x7F  系统命令（reset, status_query ...）
 *   0xF0~0xFE  调试预留
 */

#ifndef BLE_CMD_H
#define BLE_CMD_H

#include <stdint.h>

/* ================================================================
 * 命令ID宏
 * ================================================================ */

/* ——— 运动控制 0x01~0x0F ——— */
#define BLE_CMD_FORWARD     0x01   /* 前进，参数: 1字节速度档位(0~5) */
#define BLE_CMD_BACKWARD    0x02   /* 后退，参数: 1字节速度档位(0~5) */
#define BLE_CMD_LEFT        0x03   /* 左转，参数: 1字节速度档位(0~5) */
#define BLE_CMD_RIGHT       0x04   /* 右转，参数: 1字节速度档位(0~5) */
#define BLE_CMD_STOP        0x05   /* 停止，无参数 */

/* ——— 参数配置 0x10~0x2F ——— */
#define BLE_CMD_SET_SPEED   0x10   /* 设置基础速度，参数: 1字节速度档位(0~5) */

/* ——— 高级动作 0x30~0x4F ——— */
#define BLE_CMD_TURN_LEFT   0x30   /* 左转90度，无参数 */
#define BLE_CMD_TURN_RIGHT  0x31   /* 右转90度，无参数 */

/* ================================================================
 * 类型定义
 * ================================================================ */

/** 命令处理回调函数类型
 *  @param data  数据指针（不含命令ID，指向参数区）
 *  @param len   数据长度（不含命令ID）
 */
typedef void (*ble_cmd_handler_t)(const uint8_t *data, uint8_t len);

/** 命令表项 */
typedef struct {
    uint8_t           cmd_id;   /* 命令ID */
    const char       *name;     /* 命令名（调试用） */
    ble_cmd_handler_t handler;  /* 处理函数 */
} ble_cmd_entry_t;

/* ================================================================
 * 直角转弯状态机（handler 设置 → 主循环执行）
 * ================================================================ */

typedef enum {
    TURN_IDLE = 0,      /* 无转弯，正常循迹 */
    TURN_FORWARD,       /* 前进0.3s（停止循迹直行） */
    TURN_SPIN,          /* 原地旋转，等待中间灰度检测到黑线 */
    TURN_RECOVER        /* 1s内加速恢复到目标速度 */
} TurnState;

extern volatile TurnState turn_state;

/* 转弯方向 */
#define TURN_DIR_LEFT   0
#define TURN_DIR_RIGHT  1
extern volatile uint8_t turn_direction;

/* ================================================================
 * 派发入口
 * ================================================================ */

/**
 * @brief 分发蓝牙命令帧到对应的处理函数
 * @param data  完整帧数据（含命令ID在 data[0]）
 * @param len   帧数据长度
 */
void ble_cmd_dispatch(const uint8_t *data, uint8_t len);

#endif /* BLE_CMD_H */
