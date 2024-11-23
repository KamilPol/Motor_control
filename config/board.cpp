#include "board.h"
#include "clock_manager.h"

//const uint32_t ClockManager::lseClock = 32768;
// const uint32_t ClockManager::hseClock = 12000000; //4PD
const uint32_t ClockManager::hseClock = 24000000; //nucleo
volatile uint32_t ticks;

GPIO uartTX (GPIOD, 2, GPIOmode::AF, GPIOtype::PushPull, GPIOspeed::Low, GPIOpull::PullUp, 5);
GPIO uartRX (GPIOC, 12, GPIOmode::AF, GPIOtype::PushPull, GPIOspeed::Low, GPIOpull::PullDown, 5);
GPIO i2cSCL (GPIOC, 8, GPIOmode::AF, GPIOtype::OpenDrain, GPIOspeed::Low, GPIOpull::None, 8);
GPIO i2cSDA (GPIOC, 9, GPIOmode::AF, GPIOtype::OpenDrain, GPIOspeed::Low, GPIOpull::None, 8);
GPIO PWMtim2ch1 (GPIOA, 0, GPIOmode::AF, GPIOtype::PushPull, GPIOspeed::Low, GPIOpull::None, 1);

GPIO PWMtim1ch1 (GPIOC, 0, GPIOmode::AF, GPIOtype::PushPull, GPIOspeed::Low, GPIOpull::None, 2);
GPIO PWMtim1ch1N (GPIOC, 13, GPIOmode::AF, GPIOtype::PushPull, GPIOspeed::Low, GPIOpull::None, 4);

GPIO PWMtim1ch2 (GPIOC, 1, GPIOmode::AF, GPIOtype::PushPull, GPIOspeed::Low, GPIOpull::None, 2);
GPIO PWMtim1ch2N (GPIOB, 0, GPIOmode::AF, GPIOtype::PushPull, GPIOspeed::Low, GPIOpull::None, 6);


GPIO PWMtim1ch3 (GPIOA, 10, GPIOmode::AF, GPIOtype::PushPull, GPIOspeed::Low, GPIOpull::None, 6);
GPIO PWMtim1ch3N (GPIOB, 9, GPIOmode::AF, GPIOtype::PushPull, GPIOspeed::Low, GPIOpull::None, 12);

GPIO adc1Ch1 (GPIOA, 0, GPIOmode::Analog);
GPIO adc1Ch8 (GPIOC, 2, GPIOmode::Analog);
GPIO adc1Ch9 (GPIOC, 3, GPIOmode::Analog);
GPIO adc2Ch12 (GPIOB, 2, GPIOmode::Analog); //Vin


GPIO led (GPIOC, 4, GPIOmode::Output, GPIOtype::PushPull, GPIOspeed::Low, GPIOpull::None, 0);
Serial uart (UART5, 500000);
I2C i2c3 (I2C3);


int AHB2clockFreq= 160000000; // hard coded uart clock freq, to be changed later
int clockFreq = 160000000; // hard coded core clock freq, to be changed later

extern "C"
{
void UART5_IRQHandler()
	{
		uart.interrupt();
	}
}

void delay(uint32_t _delay)
{
	uint32_t start = milis;
	while (milis - start < _delay)
		;
}
float map (float _variable, float _inLowerRange, float _inUpperRange, float _outLowerRange, float _outUpperRange)
{
    float rangeCoefficient = (_outUpperRange - _outLowerRange)/(_inUpperRange - _inLowerRange);
    return _variable * rangeCoefficient + _outLowerRange;
}


