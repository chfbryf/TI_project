#include "sys.h"
/* 跨模块全局变量定义 */

key_t key;
volatile int16_t  base_speed = 160;
volatile uint8_t mode;
volatile uint8_t omega_flag;
volatile uint32_t delay_flag;
volatile unsigned char Digtal;

/**
 * @brief 固定默认速度，上电后按按键2直接开始循迹
 * @note  base_speed 单位为 mm/s，固定为 0.16 m/s
 */
void speed(void)
{
    base_speed = 160;    /* 0.16 m/s */
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
