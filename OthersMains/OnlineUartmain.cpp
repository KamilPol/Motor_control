
#include <cstdlib>
#include "stm32g4xx.h"
#include "board.h"
#include "UART.h"
#include "buffer.h"
#include "typeConverter.h"

#define LD2_PIN 5
#define B1_PIN 13

void delay(uint32_t _delay);


void Init()
{
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOCEN);
	GPIOA->MODER &= ~(0b11<<(LD2_PIN*2));
	GPIOA->MODER |= (1<<(LD2_PIN*2));

	GPIOC->MODER  &= ~ (0b11<< (B1_PIN*2));
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	
	NVIC_EnableIRQ(IRQn_Type::TIM1_UP_TIM16_IRQn);
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	EXTI->IMR1  |= (EXTI_IMR1_IM13);
	EXTI->RTSR1 |= (EXTI_RTSR1_RT13);
	SYSCFG->EXTICR[3] |= 0b0010 << 4;

	NVIC_EnableIRQ(IRQn_Type::EXTI15_10_IRQn);
	TIM1->ARR = 16000;
	TIM1->DIER |= TIM_DIER_UIE;
	TIM1->CR1  |= TIM_CR1_CEN;
}

char string [50];
volatile bool flag =0;
volatile long ovf=0;
uint16_t blinkTime=1000;
uint16_t printTime=20;
long prevBlinkTime=0;
long prevPrintTime=0;
int i=0;
int main(void)
{
		Init();  
		initSysTick();		
	while (1)
	{	
		if ((ovf-prevBlinkTime) >= blinkTime)
		{			
			prevBlinkTime = ovf;
		}		
		if(uart.readTillEOL(string))
		{
			if (string[0] == 'A')
			{
				blinkTime = typeConverter::stringToInt(string+1);		
				//uart.print(blinkTime);		
			}
			if (string[0] == 'B')
			{
				flag=1;
				delay(100);
				
				//uart.print(blinkTime);		
			}
		}
		if (flag==1)
		{
			if ((ovf-prevPrintTime) >= printTime)
			{
				uart.print(i++); 
				for (int i=0 ; i<16 ;i++)
				{
					uart.sendChar(',');
					uart.print(rand() % 30000); 
				}

				uart.sendChar(',');
				
				uart.println(rand() % 30000); 
				
				GPIOA->ODR ^= GPIO_ODR_OD5;		
				prevPrintTime = ovf;
			}		
		}				
	}
}
		//	if (uart.available())
			//{

		//	uart.readLine (lineBuffer);
		//	uart.print (lineBuffer);
			
			
		//	}
		
			// if (buffer [bufferIndex]==1)
			// {
			// 	for (int i=0; i<bufferIndex+1; i++)
			// 	{
			// 		buffer[i]=0;
			// 	}
				
			// 	uart.print("\E[2J\E[2J\E[H");
			// 	bufferIndex=0;
				
			// }


			// if (prevBuffIndex<bufferIndex)
			// {
			// 	uart.print(buffer);
			// }
	

		
		
		




extern "C"
{
	// void USART2_IRQHandler()
	// {
	// 	if (USART2->ISR & USART_ISR_RXNE)
	// 	{
	// 		GPIOA->ODR ^= GPIO_ODR_OD5;
	// 		char data;
	// 		char c = USART2->RDR;
	// 		if (buffer.read(&data) != Buffer::status::bufferEmpty)
	// 		buffer.write(c);
			
	// 		buffer.write(c);
	// 		buffer.read(&data);
	// 		uart.sendChar (data);
				
	// 	}
	// 		//USART2->RQR |= USART_RQR_RXFRQ;
	// }

void EXTI15_10_IRQHandler(void)
{
	if(EXTI->PR1 & EXTI_PR1_PIF13)
	{			
		flag = !flag;
		EXTI->PR1 |= EXTI_PR1_PIF13;
	}
}

void TIM1_UP_TIM16_IRQHandler ()
{
	TIM1->SR &= ~(TIM_SR_UIF);
	ovf++;
}
	
}

void delay(uint32_t _delay)
{
	uint32_t start = ticks;
	ticks++;
	while (ticks - start < _delay);
}