#include <cstdlib>
#include "stm32g4xx.h"
#include "board.h"
#include "UART.h"
#include "buffer.h"
#include "typeConverter.h"
#include "states.h"
#include "i2c.h"
#include "hd44780.h"
#include "pid.h"
#include "clock_manager.h"
#define M_PI 3.14159265358979323846f
#define M_2PI 6.28318530717958647692f
#define M_SQRT3_2 0.86602540378f
#define M_2_SQRT3 1.15470053838f
#define M_1_SQRT3 0.57735026919f

#define LD2_PIN 5
#define B1_PIN 13
#define I2C2_SCL_PIN 9
#define I2C2_SDA_PIN 8
#define POLE_PAIRS 11.0f
#define PWM_FREQ 40000.0f
#define PWM_PERIOD 1.0f/PWM_FREQ

#define ADC_GAIN 0.000833190416f
#define SHUNT_RESISTOR 0.001f
#define INA_GAIN 20.0f
#define VREF 1.65f
#define ADC_TO_PHASE_CURRENT (ADC_GAIN/(INA_GAIN*SHUNT_RESISTOR))
#define ADC_OFFSET (VREF/(INA_GAIN*SHUNT_RESISTOR))

#define VIN_ADC_GAIN 0.009137642f



uint32_t sineLookUp [360]={0}; 
uint32_t sineLookUp2 [360]={0};
uint32_t sineLookUp3 [360]={0};

char UARTrxData [50];
uint16_t AdcDmaReadings[3];
uint16_t ADCVin;
uint32_t motorProcessLastTime=0;
uint32_t printProcessLastTime=0;
long UARTprevTime=0;
volatile long prevTick = 0;
volatile float normalizedCoeff=0.001f;

volatile bool dataReadyToPrint = false;
char LCDstring[20]="0";
uint8_t checkFlag;
uint8_t i2cData;
float sineFreq = 1;
volatile uint32_t acceltime = 0;
volatile uint32_t lastacceltime=0;
volatile float iA = 0;
volatile float iB = 0;
volatile float iC = 0;
float iAlpha = 0;
float iBeta	= 0;
float iD = 0;
float iQ = 0;
float setiQ = 30;
float setiD = 0;
float theta = 0;
float lastAngle=0;
uint32_t lastTimeAngle=0;
uint32_t lastPrintTime=0;
uint32_t lastAngleOffsetChange = 0;
volatile float velChange = 0;	
volatile float SetOLangle =0;
static volatile float speedMul=0;
float setUq=0;
float setUd=0;
float Ualpha = 0;
float Ubeta	= 0;
volatile float pidOUT = 0;

float filterediQ = 0;
float filterediD = 0;

uint32_t motorState=0;
volatile uint32_t motorSpeed=0;
uint32_t setMotorSpeed=100;
uint32_t accell = 100;
uint32_t slopeInterval = 1000/accell;

PID pidUq (&filterediQ,  &setUq, &setiQ, 1, 2.5, 0.0001, PIDPON_TypeDef::_PID_P_ON_E, PIDCD_TypeDef::_PID_CD_DIRECT);
PID pidUd (&filterediD,  &setUd, &setiD, 1, 2.5, 0.0001, PIDPON_TypeDef::_PID_P_ON_E, PIDCD_TypeDef::_PID_CD_DIRECT);

 

void Init()
{ 
	ClockManager::hseInit();
	ClockManager::pllCfg(1, 320000000, 2, ClockManager::pllDiv::div2, ClockManager::pllDiv::div2); //160 MHz clock
	ClockManager::setSysClk(clkSrc::pll);
	//ClockManager::clockSummary();
	ClockManager::initTick();
	
	//delay(500);
	

	

	// timer 2 - sine wave generation
	//NVIC_EnableIRQ(TIM2_IRQn);
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; 
	//RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; 
	TIM2->PSC = 15; // 160000000/160 = 1000000 Hz
	TIM2->ARR = 100; // 100/1000000 = 1kHz
	TIM2->CCR1 = 10;
	//TIM2 -> DIER |= TIM_DIER_CC1DE | TIM_DIER_UIE; // Update DMA request enable
	TIM2->CR1  |= TIM_CR1_ARPE;
	TIM2->CR1  |= TIM_CR1_CEN;	
	

	// // Timer 1 - PWM generation
	NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;  
	TIM1-> CCMR1 |= 0b110<< TIM_CCMR1_OC1M_Pos | TIM_CCMR1_OC1PE ; // PWM mode 1 channel 1
	TIM1-> CCMR1 |= 0b110<< TIM_CCMR1_OC2M_Pos | TIM_CCMR1_OC2PE ; // PWM mode 1 channel 2
	TIM1-> CCMR2 |= 0b110<< TIM_CCMR2_OC3M_Pos | TIM_CCMR2_OC3PE ; // PWM mode 1 channel 3
	TIM1-> CCMR2 |= 0b0110<< TIM_CCMR2_OC4M_Pos;
	TIM1-> CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE;
	TIM1-> CCER |= TIM_CCER_CC2E | TIM_CCER_CC2NE;
	TIM1-> CCER |= TIM_CCER_CC3E | TIM_CCER_CC3NE;
	//TIM1->CR2 |= 0b010<<TIM_CR2_MMS_Pos;   // Set TRGO on Update Event
	TIM1->PSC = 3;  // 160000000/4 = 40000000 Hz
	TIM1->ARR = 40000000/PWM_FREQ; 
	TIM1-> CCR1 = 0;
	TIM1-> CCR2 = 0;
	TIM1-> CCR3 = 0;
	TIM1-> CCR4 = 500;
	TIM1 -> DIER |=  TIM_DIER_UIE; // interrupt enable
	TIM1->CR2 = 0b0111<<TIM_CR2_MMS_Pos;   // Set TRGO on Update Event
	TIM1->CR1  |= TIM_CR1_ARPE | 0b01<<TIM_CR1_CMS_Pos;
	TIM1->EGR |= TIM_EGR_UG;
	TIM1->BDTR |= TIM_BDTR_MOE | 0b00100000<<TIM_BDTR_DTG_Pos;
	TIM1->CR1  |= TIM_CR1_CEN;
	

  
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
	// DMAMUX1_Channel1->CCR = 56; // dma request from TIM2
	// DMA1_Channel2-> CCR = 0b10<<DMA_CCR_MSIZE_Pos | 0b10<<DMA_CCR_PSIZE_Pos | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_DIR ; // 16 bit memory size, 32 bit peripheral size, memory increment mode, circular mode, transfer complete interrupt enable
	// DMA1_Channel2->CMAR = (uint32_t) sineLookUp;
	// DMA1_Channel2->CPAR = (uint32_t) &(TIM1-> CCR1);
	// DMA1_Channel2->CNDTR = (sizeof(sineLookUp)/sizeof(sineLookUp [0]));
	// DMA1_Channel2->CCR |= DMA_CCR_EN;

	// DMAMUX1_Channel2->CCR = 56; // dma request from TIM2
	// DMA1_Channel3-> CCR = 0b10<<DMA_CCR_MSIZE_Pos | 0b10<<DMA_CCR_PSIZE_Pos | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_DIR ; // 16 bit memory size, 32 bit peripheral size, memory increment mode, circular mode, transfer complete interrupt enable
	// DMA1_Channel3->CMAR = (uint32_t) sineLookUp2;
	// DMA1_Channel3->CPAR = (uint32_t) &(TIM1-> CCR2);
	// DMA1_Channel3->CNDTR = (sizeof(sineLookUp2)/sizeof(sineLookUp2 [0]));
	// DMA1_Channel3->CCR |= DMA_CCR_EN;

	// DMAMUX1_Channel3->CCR = 56; // dma request from TIM2
	// DMA1_Channel4-> CCR = 0b10<<DMA_CCR_MSIZE_Pos | 0b10<<DMA_CCR_PSIZE_Pos | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_DIR ; // 16 bit memory size, 32 bit peripheral size, memory increment mode, circular mode, transfer complete interrupt enable
	// DMA1_Channel4->CMAR = (uint32_t) sineLookUp3;
	// DMA1_Channel4->CPAR = (uint32_t) &(TIM1-> CCR3);
	// DMA1_Channel4->CNDTR = (sizeof(sineLookUp3)/sizeof(sineLookUp3 [0]));
	// DMA1_Channel4->CCR |= DMA_CCR_EN;

	// ADC1 configuration
	RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;	
	ADC12_COMMON->CCR |= (0b11 << ADC_CCR_CKMODE_Pos | ADC_CCR_VREFEN);	// Set ADC clock to HCLK/2 and enable VREFINT
	ADC1->CR |= ADC_CR_ADSTP;
	while((ADC1->ISR & ADC_ISR_ADRDY));	
	ADC1->CR =0;
	ADC1->CFGR = ADC_CFGR_OVRMOD | 1<<ADC_CFGR_EXTEN_Pos | 0b01001<<ADC_CFGR_EXTSEL_Pos | ADC_CFGR_DMAEN | ADC_CFGR_DMACFG; // Set overrun mode, external trigger rising edge, TIM1_TRGO as trigger, DMA enable, DMA circular mode
	ADC1->CR |= ADC_CR_ADVREGEN;	
	ADC1->CR |= ADC_CR_ADCAL;
	while(ADC1->CR & ADC_CR_ADCAL);
	ADC1->SQR1 |= 0b10<<ADC_SQR1_L_Pos; // 3 ADC1 conversions
	ADC1->SQR1 |= 1<<ADC_SQR1_SQ1_Pos | 8<<ADC_SQR1_SQ2_Pos | 9<<ADC_SQR1_SQ3_Pos; // First conversion - channel 14. Second conversion - channel 2.
	ADC1->CR |= ADC_CR_ADEN;
	while(!(ADC1->ISR & ADC_ISR_ADRDY));
	ADC1->CR |= ADC_CR_ADSTART;

	DMAMUX1_Channel4->CCR = 5;
	DMA1_Channel5-> CCR = 0b1<<DMA_CCR_MSIZE_Pos | 0b10<<DMA_CCR_PSIZE_Pos | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE; // 16 bit memory size, 32 bit peripheral size, memory increment mode, circular mode, transfer complete interrupt enable
	DMA1_Channel5->CPAR = (uint32_t) &(ADC1->DR);
	DMA1_Channel5->CMAR = (uint32_t) AdcDmaReadings;
	DMA1_Channel5->CNDTR = 3;
	DMA1_Channel5->CCR |= DMA_CCR_EN;
	// // DAC1 configuration
	// RCC->AHB2ENR |=RCC_AHB2ENR_DAC1EN;
	// DAC1->CR |= DAC_CR_EN1;
	// DAC1->DHR12R1 = 2050;
	
	ADC2->CR |= ADC_CR_ADSTP;
	while((ADC2->ISR & ADC_ISR_ADRDY));	
	ADC2->CR =0;
	ADC2->CFGR = ADC_CFGR_OVRMOD | ADC_CFGR_CONT; // Set overrun mode, external trigger rising edge, TIM1_TRGO as trigger, DMA enable, DMA circular mode
	ADC2->CR |= ADC_CR_ADVREGEN;	
	ADC2->CR |= ADC_CR_ADCAL;
	while(ADC2->CR & ADC_CR_ADCAL);
	ADC2->SQR1 |= 0b0<<ADC_SQR1_L_Pos; // 1 ADC2 conversions
	ADC2->SQR1 |= 12<<ADC_SQR1_SQ1_Pos; // First conversion - ch12
	ADC2->CR |= ADC_CR_ADEN;
	while(!(ADC2->ISR & ADC_ISR_ADRDY));
	ADC2->CR |= ADC_CR_ADSTART;
	
}

void generateSine(uint32_t* _sineLookUp, uint32_t _phaseShift, uint32_t _amplitude, uint32_t _samples)
{
      float phaseShiftDeg = _phaseShift* M_PI/180.0f;
      float inc= 2* M_PI/ _samples;
      for (uint32_t i = 0; i<_samples; i++)
      {
            _sineLookUp[i] = 180+(_amplitude*sin(i*inc+phaseShiftDeg))*0.5f;
      }
}
void setPwm(float Ua, float Ub, float Uc)
{
	TIM1->CCR1 = Ub;
	TIM1->CCR2 = Ua;
	TIM1->CCR3 = Uc;
}
void setPhaseVoltage(float Uq, float Ud, float angle_el) 
{  
    // Inverse park transform
	Ualpha = cos(angle_el) * Ud -sin(angle_el) * Uq;  // -sin(angle) * Uq;
	Ubeta = sin(angle_el) * Ud + cos(angle_el) * Uq;    //  cos(angle) * Uq;

	// Inverse Clarke transform
	uint32_t Ua = Ualpha + 500;
	uint32_t Ub = -0.5f * Ualpha  + M_SQRT3_2 * Ubeta + 500;
	uint32_t Uc = -0.5f * Ualpha - M_SQRT3_2 * Ubeta + 500;
	setPwm(Ua, Ub, Uc);
	
	// uart.print("Ua:");
	// uart.print((int)Ua);
	// uart.print(",");
	// uart.print("Ub:");
	// uart.print((int)Ub);
	// uart.print(",");
	// uart.print("Uc:");
	// uart.println((int)Uc);
}

int main(void)
{
	// generateSine(sineLookUp, 0, 0, 360);
	// generateSine(sineLookUp2, 120, 0, 360);
	// generateSine(sineLookUp3, 240, 0, 360);
	
	// int i, j=0;
	static uint32_t prevMotorState=0;
	float offsetAngle = 0;
	//initSysTick();	
	Init();  
	pidUq.Init();
	pidUq.SetOutputLimits(-490,490);
	pidUq.SetMode(PIDMode_TypeDef::_PID_MODE_AUTOMATIC);

	pidUd.Init();
	pidUd.SetOutputLimits(-490,490);
	pidUd.SetMode(PIDMode_TypeDef::_PID_MODE_AUTOMATIC);

	//uart.print((int)ClockManager::coreClock);
	
		
	while (1)
	{
	
		
		if (uart.readTillEOL(UARTrxData)) 
		{
			
			if (UARTrxData[0] == '0')
			{
				motorState = 0;
			}
			else if (UARTrxData[0] == '1')
			{
					motorState = 1;
			}
			else if (UARTrxData[0] == 'p')
			{
					setiQ +=1;
			}
			else if (UARTrxData[0] == 'm')
			{
					setiQ -=1;
			}
			else if (UARTrxData[0] == 's')
			{
					setMotorSpeed += 50;
			}
			else if (UARTrxData[0] == 'w')
			{
					setMotorSpeed -= 100;
			}

		}

		if (milis-motorProcessLastTime>=slopeInterval)
		{
			if (!motorState)
			{
				if (prevMotorState)
				{
					
				}

			
				
				if (motorSpeed > 5)
				{
					motorSpeed-=5;
				}
				else
				{
					
				// 	generateSine(sineLookUp, 0, 0, 360);
				// 	generateSine(sineLookUp2, 120, 0, 360);
				// 	generateSine(sineLookUp3, 240, 0, 360);
					setUq = 0;
					setUd = 0;
					motorSpeed = 0;
				}	
				prevMotorState = 0;
											
			}
			if (motorState)
			{			
			
				if (prevMotorState == 0)
				{
					setUq=35;
			// 		generateSine(sineLookUp, 0, 200, 360);  //200hz 42 amplitude 21sek bez rad, 300hz 50 amplitude 6,5sek bez rad
			// 		generateSine(sineLookUp2, 120, 200, 360); // 200hz 42 amplitude 2min+++sek z rad, 300hz 50 amplitude 36 sek z rad
			// 		generateSine(sineLookUp3, 240, 200, 360); 
					// lastacceltime = milis;
				}			
				if ((motorSpeed <setMotorSpeed) && setUd > -480)
				{				
					motorSpeed++;			
				}
				else
				{
					motorSpeed =setMotorSpeed;
					//motorState = 0;
					// acceltime = milis - lastacceltime;				
				}
				prevMotorState = 1;
			}
			// sineFreq = motorSpeed * 0.1166666f; /// 7 pole pairs * 60 * speed = freq
			// if (sineFreq) 
			// {
			// 	TIM2->ARR = (uint32_t)(44444.44f)/sineFreq; /// clk/tim1->ARR = 16000000/360 = 44,444.44 (HZ)
			// }else 
			{
				//TIM2->ARR = 1;
			}  
			
			motorProcessLastTime = milis;
		}

			
			// uint8_t i2cData = 0x0C;
			// //delay(1);
			// i2c3.sendByte(&i2cData, 0x36);
			// uint8_t recieved = i2c3.recieveByte(0x36);
			// uint8_t recieved1 = i2c3.recieveByte(0x36);
			// float angle = ((recieved << 8) | recieved1)*0.0879120;

			//theta = angle * POLE_PAIRS * M_PI/180.0f - 4.4;
			
			
			
			
			// uart.print("Motor state: ");
			// uart.print((int)motorState);
			// uart.print(" Sine freq: ");
			// uart.print(sineFreq);
			// uart.print(" RPM: ");
			// uart.println((int)motorSpeed);
			//uart.println((ADC1->DR*0.0008270676f-1.65f)/0.01f/50);     // shunt/ina gain
			//uart.println((ADC1->DR*0.0008270676f-1.65f)/0.5);     // shunt*ina gain = 0.01f*50=0.5
			//uart.println(ADC1->DR*0.0016541352-3.3f);     // 0008270676f / 0.5 = 0.0016541352, 1.65f/0.5 = 3.3f

			//uart.println((ADC1->DR*4.135338f-8250f));     // shunt/ina gain = 0.01f/50=0.0002

			// uart.print("A:");
			// uart.print(iA);
 			// uart.print(",");
			// uart.print("B:");
			// uart.print(iB);
			// uart.print(",");
			// uart.print("C:");
			// uart.print(iC);
			// uart.print(",");
			// uart.print("alpha:");
			// uart.print(iAlpha);
			// uart.print(",");
			// uart.print("beta:");
			// uart.print(iBeta);
			// uart.print("iD:");
			// uart.print(iD);
			// uart.print(",");
			uart.print("FilterediQ:");
			uart.print(filterediQ);
			uart.print(",");			
			uart.print("uD:");
			uart.print(setUd);
			uart.print(",");
			uart.print("uQ:");
			uart.print(setUq);
			uart.print(",");
			uart.print("speed:");
			uart.print((int)motorSpeed);
			// uart.print(",");
			// uart.print("iQ:");
			// uart.print(iQ);
			uart.print(",");
			uart.print("filterediD:");
			uart.println(filterediD);

		
		// if(milis-lastAngleOffsetChange>=100)
		// 	{
		// 	//	led.toggle();
		// 		lastAngleOffsetChange = milis;			
		// 	}

	
		

		// if(ClockManager::micros()-lastPrintTime>=10)
		// {
		// 	if (motorState)
			
		// 	 lastPrintTime = ClockManager::micros();		
		// }
		
	}
}


extern "C"
{
	void TIM1_UP_TIM16_IRQHandler()
	{
		TIM1->SR &= ~TIM_SR_UIF;
		
			//iAlpha  = (0.66f)*(AdcDmaReadings[0]*0.0016541352f-3.3f) - (0.33f)*(AdcDmaReadings[1]*0.0016541352f-3.3f) + (0.33f)*(AdcDmaReadings[2]*0.0016541352f-3.3f);

			//iBeta   = (1.1547005f)*(AdcDmaReadings[1]*0.0016541352f-3.3f) - (1.1547005f)*(AdcDmaReadings[2]*0.0016541352f-3.3f);
		iA=AdcDmaReadings[0]*ADC_TO_PHASE_CURRENT-ADC_OFFSET;
		iB=AdcDmaReadings[2]*ADC_TO_PHASE_CURRENT-ADC_OFFSET;
		iC=AdcDmaReadings[1]*ADC_TO_PHASE_CURRENT-ADC_OFFSET;
		iAlpha  = iA;
		iBeta = (M_1_SQRT3*iAlpha) + (M_2_SQRT3 * iB);
		theta = SetOLangle;
		iD = iAlpha*cos(theta)+iBeta*sin(theta);
		iQ = -iAlpha*sin(theta)+iBeta*cos(theta);
		filterediQ = filterediQ + normalizedCoeff * (iQ - filterediQ);
		filterediD = filterediD + normalizedCoeff * (iD - filterediD);
		
			velChange = (motorSpeed * 0.10472f) * PWM_PERIOD * POLE_PAIRS; 
			if (SetOLangle < M_2PI * POLE_PAIRS) 
			{
				SetOLangle += velChange;
			}
			else
			{
				SetOLangle = 0;
			}
		if (motorState )
		{
		
			pidUq.Compute();
			pidUd.Compute();
			if (UART5->ISR & USART_ISR_ORE)
			{
				led.set();
				UART5->ICR |= USART_ICR_ORECF;
				uart.bufferFlush();
			}
			// if (iQ < setiQ +1 && iQ > setiQ -1)
			// {
			// 	;
			// }
			// else if (iQ < setiQ)
			// {
			// 	setUq += 0.01f;
			// }
			// else
			// {
			// 	setUq -= 0.01f;
			// }

			// if (iD < setiD +1 && iD > setiD -1)
			// {
			// 	;
			// }
			// else if (iD < setiD)
			// {
			// 	setUd += 0.001f; 
			// }
			// else
			// {
			// 	setUd -= 0.001f;
			// }
			
			
		
		}


		setPhaseVoltage(setUq, setUd, SetOLangle);
	}
}
// 	void TIM1_UP_TIM16_IRQHandler()
// 	{
// 		TIM1->SR &= ~TIM_SR_UIF;
// 		led.toggle();
// 		//PWMtim1ch1.toggle();
// 	}
// 	void DMA1_Channel2_IRQHandler()
// 	{
// 		DMA1->IFCR |= DMA_IFCR_CTCIF2;
// 		//led.toggle();
// 	}

// }





