#include "control.h"
#include "motor.h"
#include "sys.h"
#include "sensor.h"
#include "gyro.h"
#include "clock.h"

static volatile uint32_t B0,H0;
static volatile uint8_t B_H_flag,H_B_flag;
volatile uint32_t renwu_flag;
static volatile uint32_t black_start;

#define BLACK_DEBOUNCE_MS  50

void renwu_reset(void)
{
    B0 = 0;
    H0 = 0;
    B_H_flag = 0;
    H_B_flag = 0;
}

void renwu1(void)
{
    mode = 2;
    if((Digtal & 0xFF) != 0xFF)
    {
        if(black_start == 0) black_start = delay_flag;
        if((delay_flag - black_start) >= BLACK_DEBOUNCE_MS)
        {
            mode = 1;
        }
    }
    else
    {
        black_start = 0;
    }
}

void renwu2(void)
{
    if((Digtal & 0xFF) != 0xFF)
    {
        if(black_start == 0) black_start = delay_flag;
        if((delay_flag - black_start) >= BLACK_DEBOUNCE_MS)
        {
            mode = 3;
            H_B_flag = 1;
            if(B_H_flag == 1){
                H0++;
                B_H_flag = 0;
            }
        }
    }
    else {
        black_start = 0;
        mode = 2;

        B_H_flag = 1;
        if(H_B_flag == 1){
            B0++;
            H_B_flag = 0;
        }
    }
    if(B0 >= 2 && H0 >= 2)
    {
        mode = 1;
    }
}

void renwu3(void)
{
    if((Digtal & 0xFF) != 0xFF)
    {
        if(black_start == 0) black_start = delay_flag;
        if((delay_flag - black_start) >= BLACK_DEBOUNCE_MS)
        {
            mode = 3;
            H_B_flag = 1;
            if(B_H_flag == 1){
                H0++;
                B_H_flag = 0;
            }
        }
    }
    else {
        black_start = 0;
        B_H_flag = 1;
        mode = 4;
        if(H_B_flag == 1){
            B0++;
            H_B_flag = 0;
        }

        if(B0%2 == 0)
        {
            omega_flag = 0;
        }
        else
        {
            omega_flag = 1;
        }

    }
    if(B0 >= 2 && H0 >= 2)
    {
        mode = 1;
    }

}

void renwu4(void)
{
    if((Digtal & 0xFF) != 0xFF)
    {
        if(black_start == 0) black_start = delay_flag;
        if((delay_flag - black_start) >= BLACK_DEBOUNCE_MS)
        {
            mode = 3;
            H_B_flag = 1;
            if(B_H_flag == 1){
                H0++;
                B_H_flag = 0;
            }
        }
    }
    else {
        black_start = 0;
        B_H_flag = 1;
        mode = 4;
        if(H_B_flag == 1){
            B0++;
            H_B_flag = 0;
        }

        if(B0%2 == 0)
        {
            omega_flag = 0;
        }
        else
        {
            omega_flag = 1;
        }

    }
    if(B0 >= 8 && H0 >= 8)
    {
        mode = 1;
    }

}

