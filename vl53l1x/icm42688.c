#include "icm42688.h"
#include "ti_msp_dl_config.h"
#include <math.h>

#ifndef M_PI
#define M_PI  3.14159265358979323846f
#endif

/* ===================== 坐标轴映射 =====================
 *
 * ICM42688 芯片物理轴：X=右, Y=前, Z=上（芯片面朝上时）
 * 但很多 breakout 板的芯片朝向不同，需根据你的实际安装方式调整。
 *
 * 映射规则：chip_X/Y/Z 分别映射到 body 的哪个轴，正负表示方向。
 * 0=chip_X, 1=chip_Y, 2=chip_Z
 *
 * 常见情况——芯片 Y 轴朝上时的映射：
 *   body_X = +chip_X,  body_Y = -chip_Z,  body_Z = +chip_Y
 */
#define AXIS_SRC_X   0   /* body_X 取自 chip_X */
#define AXIS_SGN_X  +1   /* 方向：+1 同向, -1 反向 */
#define AXIS_SRC_Y   2   /* body_Y 取自 chip_Z */
#define AXIS_SGN_Y  -1   /* 方向 */
#define AXIS_SRC_Z   1   /* body_Z 取自 chip_Y */
#define AXIS_SGN_Z  +1   /* 方向 */

/* ===================== 全局变量 ===================== */
volatile bool g_icm42688_dataReady = false;

float icm42688_yaw   = 0.0f;
float icm42688_pitch = 0.0f;
float icm42688_roll  = 0.0f;

float icm42688_acc_x  = 0.0f;
float icm42688_acc_y  = 0.0f;
float icm42688_acc_z  = 0.0f;
float icm42688_gyro_x = 0.0f;
float icm42688_gyro_y = 0.0f;
float icm42688_gyro_z = 0.0f;

/* 陀螺仪零偏 */
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;

/* ===================== CS 控制 ===================== */
static inline void spi_cs_low(void)
{
    DL_GPIO_clearPins(CS_PORT, CS_PIN_0_PIN);
}

static inline void spi_cs_high(void)
{
    DL_GPIO_setPins(CS_PORT, CS_PIN_0_PIN);
}

/* ===================== SPI 底层读写 ===================== */
#define SPI_TIMEOUT  100000u

static int spi_write_reg(uint8_t reg, uint8_t val)
{
    volatile uint32_t to;

    spi_cs_low();
    DL_SPI_transmitDataBlocking8(SPI_0_INST, reg & 0x7F);
    to = SPI_TIMEOUT;
    while (DL_SPI_isRXFIFOEmpty(SPI_0_INST)) { if (--to == 0) goto err; }
    (void)DL_SPI_receiveData8(SPI_0_INST);

    DL_SPI_transmitDataBlocking8(SPI_0_INST, val);
    to = SPI_TIMEOUT;
    while (DL_SPI_isRXFIFOEmpty(SPI_0_INST)) { if (--to == 0) goto err; }
    (void)DL_SPI_receiveData8(SPI_0_INST);

    spi_cs_high();
    return 0;

err:
    spi_cs_high();
    return -1;
}

static uint8_t spi_read_reg(uint8_t reg)
{
    uint8_t val = 0;
    volatile uint32_t to;

    spi_cs_low();
    DL_SPI_transmitDataBlocking8(SPI_0_INST, reg | 0x80);
    to = SPI_TIMEOUT;
    while (DL_SPI_isRXFIFOEmpty(SPI_0_INST)) { if (--to == 0) goto err; }
    (void)DL_SPI_receiveData8(SPI_0_INST);

    DL_SPI_transmitDataBlocking8(SPI_0_INST, 0x00);
    to = SPI_TIMEOUT;
    while (DL_SPI_isRXFIFOEmpty(SPI_0_INST)) { if (--to == 0) goto err; }
    val = DL_SPI_receiveData8(SPI_0_INST);

err:
    spi_cs_high();
    return val;
}

static int spi_read_regs(uint8_t reg, uint8_t *buf, uint32_t len)
{
    uint32_t i;
    volatile uint32_t to;

    if (len == 0) return 0;

    spi_cs_low();
    DL_SPI_transmitDataBlocking8(SPI_0_INST, reg | 0x80);
    to = SPI_TIMEOUT;
    while (DL_SPI_isRXFIFOEmpty(SPI_0_INST)) { if (--to == 0) goto err; }
    (void)DL_SPI_receiveData8(SPI_0_INST);

    for (i = 0; i < len; i++) {
        DL_SPI_transmitDataBlocking8(SPI_0_INST, 0x00);
        to = SPI_TIMEOUT;
        while (DL_SPI_isRXFIFOEmpty(SPI_0_INST)) { if (--to == 0) goto err; }
        buf[i] = DL_SPI_receiveData8(SPI_0_INST);
    }

    spi_cs_high();
    return 0;

err:
    spi_cs_high();
    return -1;
}

/* ===================== SPI 模式切换 ===================== */
static void spi_reinit(DL_SPI_FRAME_FORMAT frameFormat, uint32_t divider)
{
    DL_SPI_disable(SPI_0_INST);
    SPI_0_INST->CLKSEL = DL_SPI_CLOCK_BUSCLK;
    DL_SPI_Config cfg = {
        .mode        = DL_SPI_MODE_CONTROLLER,
        .frameFormat = frameFormat,
        .parity      = DL_SPI_PARITY_NONE,
        .dataSize    = DL_SPI_DATA_SIZE_8,
        .bitOrder    = DL_SPI_BIT_ORDER_MSB_FIRST,
    };
    DL_SPI_init(SPI_0_INST, &cfg);
    DL_SPI_setBitRateSerialClockDivider(SPI_0_INST, divider);
    DL_SPI_setFIFOThreshold(SPI_0_INST,
        DL_SPI_RX_FIFO_LEVEL_1_2_FULL, DL_SPI_TX_FIFO_LEVEL_1_2_EMPTY);
    DL_SPI_setChipSelect(SPI_0_INST, DL_SPI_CHIP_SELECT_NONE);
    DL_SPI_enable(SPI_0_INST);
}

/* ===================== 坐标轴映射辅助 ===================== */
static const int8_t axis_src[3] = { AXIS_SRC_X, AXIS_SRC_Y, AXIS_SRC_Z };
static const int8_t axis_sgn[3] = { AXIS_SGN_X, AXIS_SGN_Y, AXIS_SGN_Z };

static void apply_axis_map(const float chip[3], float body[3])
{
    const float *src = chip;
    for (int i = 0; i < 3; i++) {
        body[i] = axis_sgn[i] * src[axis_src[i]];
    }
}

/* ===================== 传感器初始化 ===================== */
int Init_ICM42688(void)
{
    uint8_t whoami;
    volatile uint32_t timeout;
    int i;

    spi_cs_high();
    delay_cycles(CPUCLK_FREQ / 1000);

    /* CS 脉冲：确保传感器 SPI 状态机复位 */
    for (i = 0; i < 3; i++) {
        spi_cs_low();
        delay_cycles(CPUCLK_FREQ / 10000);
        spi_cs_high();
        delay_cycles(CPUCLK_FREQ / 10000);
    }

    /* 自动扫描 SPI 模式/速度 */
    static const DL_SPI_FRAME_FORMAT fmt_tbl[] = {
        DL_SPI_FRAME_FORMAT_MOTO3_POL0_PHA0,
        DL_SPI_FRAME_FORMAT_MOTO3_POL1_PHA1,
        DL_SPI_FRAME_FORMAT_MOTO3_POL0_PHA1,
        DL_SPI_FRAME_FORMAT_MOTO3_POL1_PHA0,
    };
    static const uint32_t dividers[] = { 255, 127, 63, 31, 15, 7, 3, 1, 0 };

    int found = 0;
    for (int m = 0; m < 4 && !found; m++) {
        for (int d = 0; d < 9 && !found; d++) {
            spi_reinit(fmt_tbl[m], dividers[d]);
            spi_cs_high();
            delay_cycles(CPUCLK_FREQ / 10000);
            whoami = spi_read_reg(ICM42688_WHO_AM_I);
            if (whoami == ICM42688_WHO_AM_I_VALUE) found = 1;
        }
    }
    if (!found) return -1;

    /* 软复位 */
    spi_write_reg(ICM42688_DEVICE_CONFIG, 0x01);
    delay_cycles(CPUCLK_FREQ / 1000);
    timeout = 50000;
    while (spi_read_reg(ICM42688_DEVICE_CONFIG) & 0x01) {
        if (--timeout == 0) return -2;
    }

    /* 确保 Bank 0 */
    spi_write_reg(ICM42688_REG_BANK_SEL, 0x00);

    /* 断电：先关闭传感器再配置参数（对齐官方例程顺序） */
    spi_write_reg(ICM42688_PWR_MGMT0, 0x00);
    delay_cycles(CPUCLK_FREQ / 10);  /* 10ms, 数据手册要求 ≥200µs */

    /* 中断：UI_DRDY → INT1 */
    spi_write_reg(ICM42688_INT_SOURCE0, 0x10);

    /* 先配参数… 对齐官方例程：配置在前、上电在后
     * ODR 公式: reg_val = odr_enum + 1 (200Hz → enum=6 → reg=7 → 0x07)
     * 加速度计：±16g(FS=0), 200Hz  →  ACCEL_CONFIG0 = (0<<5) | 7 = 0x07
     * 陀螺仪：  ±2000dps(FS=0), 200Hz → GYRO_CONFIG0  = (0<<5) | 7 = 0x07 */
    spi_write_reg(ICM42688_ACCEL_CONFIG0, 0x07);
    spi_write_reg(ICM42688_GYRO_CONFIG0,  0x07);

    /* …最后上电启动：陀螺仪+加速度计 Low Noise 模式 */
    spi_write_reg(ICM42688_PWR_MGMT0, 0x0F);
    delay_cycles(CPUCLK_FREQ / 5);  /* 200ms, 等待传感器稳定 */

    /* 陀螺仪零偏校准（静止采样 100 次取平均） */
    ICM42688_CalibrateGyro(100);

    return 0;
}

/* ===================== 陀螺仪零偏校准 ===================== */
#define GYRO_RAW_SCALE  (1.0f / 16.4f)

void ICM42688_CalibrateGyro(int samples)
{
    float sum[3] = { 0.0f, 0.0f, 0.0f };
    uint8_t buf[6];
    int16_t raw[3];

    if (samples < 1) samples = 100;

    for (int n = 0; n < samples; n++) {
        /* 等数据就绪 */
        volatile uint32_t to = 50000;
        while (!(spi_read_reg(ICM42688_INT_STATUS) & 0x08)) {
            if (--to == 0) break;
        }

        spi_read_regs(ICM42688_GYRO_DATA_X1, buf, 6);
        raw[0] = ((int16_t)buf[0] << 8) | buf[1];
        raw[1] = ((int16_t)buf[2] << 8) | buf[3];
        raw[2] = ((int16_t)buf[4] << 8) | buf[5];

        sum[0] += raw[0] * GYRO_RAW_SCALE;
        sum[1] += raw[1] * GYRO_RAW_SCALE;
        sum[2] += raw[2] * GYRO_RAW_SCALE;
    }

    gyro_bias_x = sum[0] / samples;
    gyro_bias_y = sum[1] / samples;
    gyro_bias_z = sum[2] / samples;
}

/* ===================== 数据就绪检测 ===================== */
bool ICM42688_DataReady(void)
{
    if (g_icm42688_dataReady) return true;

    /* 快速通道：直接读 INT1 引脚电平（GPIO 读，无需 SPI） */
    if (DL_GPIO_readPins(INT1_PORT, INT1_PIN_1_PIN) & INT1_PIN_1_PIN)
        return true;

    return false;
}

void ICM42688_NotifyDataReady(void)
{
    g_icm42688_dataReady = true;
}

/* ===================== 读取传感器数据（已做轴映射） ===================== */
#define ACCEL_SCALE   (1.0f / 2048.0f)   /* ±16g → 2048 LSB/g */

void Get_Acc_ICM42688(void)
{
    uint8_t buf[6];
    float chip_acc[3], body_acc[3];

    spi_read_regs(ICM42688_ACCEL_DATA_X1, buf, 6);
    chip_acc[0] = ((int16_t)((buf[0] << 8) | buf[1])) * ACCEL_SCALE;
    chip_acc[1] = ((int16_t)((buf[2] << 8) | buf[3])) * ACCEL_SCALE;
    chip_acc[2] = ((int16_t)((buf[4] << 8) | buf[5])) * ACCEL_SCALE;

    apply_axis_map(chip_acc, body_acc);
    icm42688_acc_x = body_acc[0];
    icm42688_acc_y = body_acc[1];
    icm42688_acc_z = body_acc[2];
}

void Get_Gyro_ICM42688(void)
{
    uint8_t buf[6];
    float chip_gyro[3], body_gyro[3];

    spi_read_regs(ICM42688_GYRO_DATA_X1, buf, 6);
    chip_gyro[0] = ((int16_t)((buf[0] << 8) | buf[1])) * GYRO_RAW_SCALE;
    chip_gyro[1] = ((int16_t)((buf[2] << 8) | buf[3])) * GYRO_RAW_SCALE;
    chip_gyro[2] = ((int16_t)((buf[4] << 8) | buf[5])) * GYRO_RAW_SCALE;

    /* 零偏在芯片帧减去，再做轴映射 */
    chip_gyro[0] -= gyro_bias_x;
    chip_gyro[1] -= gyro_bias_y;
    chip_gyro[2] -= gyro_bias_z;

    apply_axis_map(chip_gyro, body_gyro);
    icm42688_gyro_x = body_gyro[0];
    icm42688_gyro_y = body_gyro[1];
    icm42688_gyro_z = body_gyro[2];
}

/* ===================== 姿态解算（互补滤波） ===================== */
void ICM42688_ComputeAttitude(float dt)
{
    float acc_pitch, acc_roll;
    float norm;

    norm = sqrtf(icm42688_acc_x * icm42688_acc_x +
                 icm42688_acc_y * icm42688_acc_y +
                 icm42688_acc_z * icm42688_acc_z);
    if (norm < 0.001f) return;

    acc_pitch = atan2f(-icm42688_acc_x,
                       sqrtf(icm42688_acc_y * icm42688_acc_y +
                             icm42688_acc_z * icm42688_acc_z))
                * (180.0f / (float)M_PI);
    acc_roll  = atan2f(icm42688_acc_y, icm42688_acc_z)
                * (180.0f / (float)M_PI);

    static bool first_run = true;
    if (first_run) {
        icm42688_pitch = acc_pitch;
        icm42688_roll  = acc_roll;
        icm42688_yaw   = 0.0f;
        first_run = false;
        return;
    }

    icm42688_yaw += icm42688_gyro_z * dt;

    const float alpha = 0.98f;
    icm42688_pitch = alpha * (icm42688_pitch + icm42688_gyro_x * dt)
                     + (1.0f - alpha) * acc_pitch;
    icm42688_roll  = alpha * (icm42688_roll  + icm42688_gyro_y * dt)
                     + (1.0f - alpha) * acc_roll;

    while (icm42688_yaw   >  180.0f) icm42688_yaw   -= 360.0f;
    while (icm42688_yaw   < -180.0f) icm42688_yaw   += 360.0f;
    while (icm42688_roll  >  180.0f) icm42688_roll  -= 360.0f;
    while (icm42688_roll  < -180.0f) icm42688_roll  += 360.0f;
    if      (icm42688_pitch >  90.0f) icm42688_pitch =  90.0f;
    else if (icm42688_pitch < -90.0f) icm42688_pitch = -90.0f;
}

/* ===================== 一站式读取+解算 ===================== */
void ICM42688_ReadAndCompute(float dt)
{
    Get_Acc_ICM42688();
    Get_Gyro_ICM42688();
    ICM42688_ComputeAttitude(dt);
}
