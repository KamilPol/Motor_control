#ifndef BOARD_H
#define BOARD_H

#include "stm32g4xx.h"
#include "UART.h"
#include "buffer.h"
#include "i2c.h"
#include "hd44780.h"
#include "gpio.h"
#include "pid.h"
class Serial;
class HD44780;
class I2C;

extern volatile uint32_t ticks;
extern int UARTclockFreq;
extern int AHB2clockFreq;
extern int clockFreq;


extern HD44780 lcd;
extern Serial uart;
extern I2C i2c3;
extern GPIO uartTX;
extern GPIO uartRX;
extern GPIO Led4PD_1;
extern GPIO led1 ;
extern GPIO led2 ;
//extern GPIO led3 ;
extern GPIO led4 ;
extern GPIO led5 ;
extern GPIO PWMtim1ch1;
enum class StateNames
{
    ADCread,
	plot,
	DMAarray,
    adcDMAtests,
    idle,
	count
};




void delay(uint32_t _delay); // zrobić klase na te funkcje
float map (float _variable, float _inLowerRange, float _inUpperRange, float _outLowerRange, float _outUpperRange);

#endif // BOARD_H