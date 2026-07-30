#ifndef SENSOR2_H_
#define SENSOR2_H_

#include "stdint.h"

void Get_err2(void);
int16_t Err2(void);

extern volatile uint8_t black_detected;

#endif
