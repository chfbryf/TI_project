#include "uart.h"
#include "ti_msp_dl_config.h"

/* 环形缓冲区 */
static uint8_t rx_buf[UART_RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

/* 工作缓冲区 */
static uint8_t frame_buf[UART_RX_BUF_SIZE];
static uint16_t frame_len = 0;

static AIMCallback aim_callback = NULL;
static CircleDoneCallback circle_done_callback = NULL;

/* 内部函数 */
static void ParseFrame(const uint8_t *data, uint16_t len);

void UartParser_Init(AIMCallback aim_cb, CircleDoneCallback circle_done_cb)
{
    aim_callback = aim_cb;
    circle_done_callback = circle_done_cb;

    /* 使能 NVIC，SysConfig 生成的 init 缺少这一步 */
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    /* 启动 UART 接收中断 */
    DL_UART_enableInterrupt(UART_0_INST, DL_UART_INTERRUPT_RX);
}

void UartParser_RxByte(uint8_t byte)
{
    uint16_t next = (rx_head + 1) % UART_RX_BUF_SIZE;
    if (next != rx_tail) {
        rx_buf[rx_head] = byte;
        rx_head = next;
    }
    /* else: 缓冲区满，丢弃字节 */
}

void UartParser_Process(void)
{
    while (rx_tail != rx_head) {
        uint8_t byte = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % UART_RX_BUF_SIZE;

        if (byte == '\n') {
            if (frame_len > 0) {
                frame_buf[frame_len] = '\0';
                ParseFrame(frame_buf, frame_len);
                frame_len = 0;
            }
        } else if (byte != '\r') {
            if (frame_len < UART_RX_BUF_SIZE - 1) {
                frame_buf[frame_len++] = byte;
            } else {
                frame_len = 0;   // 帧过长，丢弃
            }
        }
    }
}

static void ParseFrame(const uint8_t *data, uint16_t len)
{
    if (len < 4) return;

    /**
     * AIM,valid,mode,point_index,point_count,point_locked,error_x,error_y,
     *     target_x,target_y,laser_x,laser_y,rect_center_x,rect_center_y,rect_confidence
     * 共 15 个字段
     */
    if (data[0] == 'A' && data[1] == 'I' && data[2] == 'M' && data[3] == ',') {
        int valid_i, point_index_i, point_count_i, point_locked_i;
        float error_x, error_y, target_x, target_y;
        float laser_x, laser_y, rect_center_x, rect_center_y, rect_confidence;
        char mode[16];

        int n = sscanf((const char *)data,
            "AIM,%d,%15[^,],%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f",
            &valid_i, mode, &point_index_i, &point_count_i, &point_locked_i,
            &error_x, &error_y,
            &target_x, &target_y,
            &laser_x, &laser_y,
            &rect_center_x, &rect_center_y,
            &rect_confidence);

        if (n == 14 && aim_callback) {
            aim_callback((uint8_t)valid_i, mode,
                         point_index_i, point_count_i, point_locked_i,
                         error_x, error_y,
                         target_x, target_y,
                         laser_x, laser_y,
                         rect_center_x, rect_center_y,
                         rect_confidence);
        }
    }
    /* CIRCLE,DONE,<point_count> */
    else if (data[0] == 'C' && data[1] == 'I' && data[2] == 'R' && data[3] == 'C'
             && data[4] == 'L' && data[5] == 'E' && data[6] == ',') {
        int point_count;
        if (sscanf((const char *)data, "CIRCLE,DONE,%d", &point_count) == 1) {
            if (circle_done_callback) {
                circle_done_callback(point_count);
            }
        }
    }
}

/**
 * @brief 通过 UART 发送字符串
 */
void UartParser_SendString(const char *str)
{
    while (*str) {
        /* 等 TX FIFO 有空位（超时保护） */
        uint32_t t = 100000;
        while (DL_UART_isTXFIFOFull(UART_0_INST) && --t);
        if (t == 0) return;

        DL_UART_transmitData(UART_0_INST, (uint8_t)*str);
        str++;

        /* 等移位寄存器完成（115200bps，每字节约 87us） */
        for (volatile uint32_t d = 0; d < 8000; d++);
    }
}

void UART_0_INST_IRQHandler(void)
{
    if (DL_UART_Main_getRawInterruptStatus(UART_0_INST, DL_UART_INTERRUPT_RX) &
        DL_UART_INTERRUPT_RX) {
        uint8_t byte = DL_UART_Main_receiveData(UART_0_INST);
        UartParser_RxByte(byte);
    }
}
