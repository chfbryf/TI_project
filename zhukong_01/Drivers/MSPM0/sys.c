#include "sys.h"
/* 跨模块全局变量定义 */

key_t key;
volatile int16_t  base_speed;
volatile uint8_t mode;
volatile uint8_t omega_flag;
volatile uint32_t delay_flag;
volatile unsigned char Digtal;

/**
 * @brief 根据按键档位设置目标速度
 * @param keyspeed 速度档位（0~5），0为停车
 * @note  base_speed 单位为 mm/s
 */
void speed(uint8_t keyspeed)
{
    if (keyspeed == 5)      base_speed = 600;    /* 0.60 m/s（全速） */
    else if (keyspeed == 4) base_speed = 450;    /* 0.45 m/s */
    else if (keyspeed == 3) base_speed = 300;    /* 0.30 m/s */
    else if (keyspeed == 2) base_speed = 200;    /* 0.20 m/s */
    else if (keyspeed == 1) base_speed = 100;    /* 0.10 m/s */
    else                    base_speed = 0;      /* 停车 */
}

/**
 * @brief printf重定向函数
 */
int __io_putchar(int ch)
{
    while (DL_UART_isBusy(UART_0_INST) == true);
    DL_UART_Main_transmitData(UART_0_INST, ch);
    return ch;
}

int _write(int fd, const char *ptr, int len)
{
    (void)fd;
    for (int i = 0; i < len; i++) {
        __io_putchar(ptr[i]);
    }
    return len;
}

int fputc(int ch, FILE *stream)
{
    (void)stream;
    return __io_putchar(ch);
}

int fputs(const char *restrict s, FILE *restrict stream)
{
    (void)stream;
    int char_len = 0;
    while (*s != 0) {
        __io_putchar(*s++);
        char_len++;
    }
    return char_len;
}

int puts(const char *ptr)
{
    int len = fputs(ptr, stdout);
    __io_putchar('\n');
    return len + 1;
}
