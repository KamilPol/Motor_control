#ifndef BLDC_BRD_H
#define BLDC_BRD_H

#include "stm32g4xx.h"
#include "adc.h"
#include "pwm.h"

extern adcChannel_t adcPhaseCurrents;
extern adcChannel_t adcVin;

extern pwm_t inverterPWM;


#endif // BLDC_BRD_H