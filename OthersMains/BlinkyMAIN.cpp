#include "stm32g4xx.h"

volatile uint32_t ticks;
extern "C"
{
	void SysTick_Handler()
	{
	;
	}
}
void msDelay(uint32_t _delay);

int main(void)
{
	SysTick_Config(1600000);	
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	GPIOA->MODER &=	~(11<<10);
	GPIOA->MODER |=	 (01<<10);

   	while(1)
	{
		
		GPIOA->BSRR |= GPIO_BSRR_BS5;
		__WFI();
		GPIOA->BSRR |= GPIO_BSRR_BR5;
		__WFI();
		

	}
}

void msDelay(uint32_t _delay)
	{
	uint32_t start = ticks;
	ticks++;
	//while (ticks - start < _delay);
	
	__WFI();
	}