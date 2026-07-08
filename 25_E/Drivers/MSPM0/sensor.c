#include "sensor.h"

static volatile int32_t   sensor_err;

/**
 * @brief 根据灰度传感器数据计算位置误差
 */
void Get_error(void)
{
        /*if(L1 && R1)    sensor_err = 0;
        else if(L1 && L2)      sensor_err = 50;
        else if(R1 && R2)      sensor_err = -50;
        else if(L2)     sensor_err = 100; 
        else if(R2)     sensor_err = -100;*/
        if(L4)  sensor_err = 4;
        if(L3)  sensor_err = 3;
        if(L2)  sensor_err = 2;
        if(L1)  sensor_err = 1;
        if(R1)  sensor_err = -1;
        if(R2)  sensor_err = -2;
        if(R3)  sensor_err = -3;
        if(R4)  sensor_err = -4;
}

int32_t Error(void)
{
        return sensor_err;
}
