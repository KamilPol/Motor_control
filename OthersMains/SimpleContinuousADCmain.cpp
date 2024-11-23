
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
	
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIOBEN);  // Clocks for GPIO Ports

	GPIOA->MODER &= ~(0b11<<(LD2_PIN*2)); // Reset MODER register for GPIOA
	GPIOA->MODER |= (1<<(LD2_PIN*2)); // Set LD2 pin as out
	GPIOB->MODER = (1<<22 | 1<<23); // Set PB11 as analog in
	GPIOC->MODER  &= ~ (0b11<< (B1_PIN*2)); // Reset bits in MODER register for button 

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;  // Clock for timer 1
	NVIC_EnableIRQ(IRQn_Type::EXTI15_10_IRQn);
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	EXTI->IMR1  |= (EXTI_IMR1_IM13);
	EXTI->RTSR1 |= (EXTI_RTSR1_RT13);
	SYSCFG->EXTICR[3] |= 0b0010 << 4;

	NVIC_EnableIRQ(IRQn_Type::TIM1_UP_TIM16_IRQn);
	TIM1->ARR = 16000;
	TIM1->DIER |= TIM_DIER_UIE;
	TIM1->CR1  |= TIM_CR1_CEN;


	RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
	
	ADC12_COMMON->CCR |= (11 << ADC_CCR_CKMODE_Pos | ADC_CCR_VREFEN);
	
	ADC1->CR |= ADC_CR_ADSTP;
	
	ADC1->CR =0;
	
	ADC1->CFGR |= (ADC_CFGR_CONT | ADC_CFGR_OVRMOD);
	
	ADC1->CR |= ADC_CR_ADVREGEN;
	
	ADC1->CR |= ADC_CR_ADCAL;
	while(ADC1->CR & ADC_CR_ADCAL);
	
	ADC1->SQR1 = 14<<6;
	
	
	ADC1->CR |= ADC_CR_ADEN;
	while(!(ADC1->ISR & ADC_ISR_ADRDY));
	

	
	ADC1->CR |= ADC_CR_ADSTART;

	
}

char string [50];
volatile bool flag =0;
volatile long ovf=0;
uint16_t blinkTime=1000;
uint16_t printTime=100;
long prevBlinkTime=0;
long prevPrintTime=0;
int i=0;
 uint16_t adc_value =0;
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
			

				uart.sendChar(',');
				adc_value = ADC1->DR;
				uart.println(adc_value);
				
				GPIOA->ODR ^= GPIO_ODR_OD5;		
				prevPrintTime = ovf;
			}		
		}		
		else 
		{
		
			adc_value = ADC1->DR;
			uart.println(adc_value);
			
		}	
	}
}
extern "C"
{
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