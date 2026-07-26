#ifndef ADC_H_
#define ADC_H_

#include "ti_msp_dl_config.h"
extern uint16_t ADC_VALUE[40];
unsigned int adc_getValue(unsigned int number); /* 读取ADC的数据 */

#endif /* ADC_H_ */