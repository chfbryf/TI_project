#ifndef KEY_H
#define KEY_H

#include "ti_msp_dl_config.h"

/* 按键数据结构 */
typedef struct {
    volatile uint8_t  keyspeed;
    volatile uint8_t  quan;
    volatile uint8_t  keynum;
    volatile uint8_t  start;
} key_t;

extern key_t key;

void key_work(void);

#endif