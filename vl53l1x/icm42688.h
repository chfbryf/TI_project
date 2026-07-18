#ifndef ICM42688_H
#define ICM42688_H

#include <stdint.h>
#include <stdbool.h>

/* ICM42688 SPI Register Map (Bank 0) */
#define ICM42688_DEVICE_CONFIG     0x11
#define ICM42688_INT_CONFIG        0x14
#define ICM42688_INT_SOURCE0       0x18
#define ICM42688_ACCEL_DATA_X1     0x1F
#define ICM42688_GYRO_DATA_X1      0x25
#define ICM42688_INT_STATUS        0x2D
#define ICM42688_PWR_MGMT0         0x4E
#define ICM42688_GYRO_CONFIG0      0x4F
#define ICM42688_ACCEL_CONFIG0     0x50
#define ICM42688_WHO_AM_I          0x75
#define ICM42688_REG_BANK_SEL      0x76

#define ICM42688_WHO_AM_I_VALUE    0x47

/* Data ready flag (set by ISR, read by main loop) */
extern volatile bool g_icm42688_dataReady;

/* Attitude angles (degrees) */
extern float icm42688_yaw;
extern float icm42688_pitch;
extern float icm42688_roll;

/* Sensor data in body frame (g and dps), already axis-remapped & bias-corrected */
extern float icm42688_acc_x;
extern float icm42688_acc_y;
extern float icm42688_acc_z;
extern float icm42688_gyro_x;
extern float icm42688_gyro_y;
extern float icm42688_gyro_z;

/* ---------- Public API ---------- */

/* 初始化传感器（含陀螺仪零偏校准），返回 0=成功 */
int Init_ICM42688(void);

/* 陀螺仪零偏校准，samples=采样次数（需保持传感器静止） */
void ICM42688_CalibrateGyro(int samples);

/* 检查新数据就绪（中断+轮询双模式） */
bool ICM42688_DataReady(void);

/* ISR 中调用 */
void ICM42688_NotifyDataReady(void);

/* 读取传感器数据（已做轴映射） */
void Get_Acc_ICM42688(void);
void Get_Gyro_ICM42688(void);

/* 姿态解算（互补滤波），dt=间隔秒数 */
void ICM42688_ComputeAttitude(float dt);

/* 一站式：读取+解算 */
void ICM42688_ReadAndCompute(float dt);

#endif /* ICM42688_H */
