 
#include "stm32f10x.h"
#include <stdint.h>
 
 int main(void);
 void Init_Clock(void);
 uint16_t Read_Encoder_Pulses(void);
 void Init_Encoder_Counter(void);
 void Init_GPIO(void);
 void PWM_Output(uint8_t);
 
 
#define Frecuency_Hz (1000)
#define Period   (SystemCoreClock/Frecuency_Hz)-1     //APB1 CLK 36 MHz 8,000,000/1000= 8000 ciclos
 
 //Global variable
 uint16_t Encoder_Pulses;
 
 
 int main(void){
 //Init_Clock(); 
	 
	  //Advance Port Bus 2 enabled: AFIO, GPIO_A & GPIO_B
	 RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN + RCC_APB2ENR_IOPBEN;
	 
	 //Advance Port Bus 1 enabled: TIM2 
	 RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // enable Timer2 clock
	 
	 //Init_GPIO();
	 
	 //Init_Encoder_Counter();
   while(1){
		// Encoder_Pulses = Read_Encoder_Pulses();
		 PWM_Output(50);
 }
}
 
 
 void Init_GPIO(void){
	 
	
	 
	
	 
	 
	 GPIOA->CRL |= GPIO_CRL_MODE2_1 + GPIO_CRL_MODE2_0; //PA2 Output mode 2 MHz  (A1 H Bridge )
	 GPIOA->CRL |= GPIO_CRL_MODE3_1 + GPIO_CRL_MODE3_0; //PA3 Output mode 2 MHz  (A2 H Bridge
	 
	 
	 //Port configuration register high (GPIOx_CRH) 
	 GPIOB->CRH |= GPIO_CRL_CNF6_1 + GPIO_CRL_CNF6_0; //PB6 Input mode pullup/pulldown (Encoder A signal)
	 GPIOB->CRH |= GPIO_CRL_CNF7_1 + GPIO_CRL_CNF7_0; //PB7 Input mode pullup/pulldown (Encoder B signal)
	 
	
 
 }
 
 void Init_Encoder_Counter(void){   //Configure TIM4 for quadrature encoder read
	
	 //Advance Port Bus 1 enabled: TIM4
	 RCC->APB1ENR |= RCC_APB1ENR_TIM4EN ;
	
	 //TIMx capture/compare mode register 1 (TIMx_CCMR1) 
	 TIM4->CCMR1 |= TIM_CCMR1_CC1S_0 + TIM_CCMR1_CC1S_1; // 01: CC1 channel is configured as input, IC1 is mapped on TI1
	 TIM4->CCMR1 |= TIM_CCMR1_CC2S_0 + TIM_CCMR1_CC2S_1; // 01: CC2 channel is configured as input, IC2 is mapped on TI2 
	 
	 //TIMx slave mode control register (TIMx_SMCR)   *****4 MULTIPLICATOR *******
	 TIM4->SMCR |=  TIM_SMCR_SMS_1+TIM_SMCR_SMS_0;  //011: Encoder mode 3 - Counter counts up/down on both TI1FP1 and TI2FP2 edges 
	 ; 
	 //TIMx control register 1 (TIMx_CR1) 
	 TIM4->CR1 |= TIM_CR1_CEN;    //Enable TIM4
 
 }
 
 uint16_t Read_Encoder_Pulses(void){
	 
	 
	 //TIMx counter (TIMx_CNT) 
	 Encoder_Pulses = TIM4->CNT;
	 return Encoder_Pulses;
 
 }

 void Init_Clock(void){
	 
    // Conf clock : 72MHz using HSE 8MHz crystal w/ PLL X 9 (8MHz x 9 = 72MHz)
    FLASH->ACR      |= FLASH_ACR_LATENCY_2; // 1. Two wait states, per datasheet
    RCC->CFGR       |= RCC_CFGR_PPRE1_2;    // 2. prescale AHB1 = HCLK/2
    RCC->CR         |= RCC_CR_HSEON;        // 3. enable HSE clock
    while( !(RCC->CR & RCC_CR_HSERDY) );    // 4. wait for the HSEREADY flag
    
    RCC->CFGR       |= RCC_CFGR_PLLSRC;     // 5. set PLL source to HSE
    RCC->CFGR       |= RCC_CFGR_PLLMULL9;   // 6. multiply by 9
    RCC->CR         |= RCC_CR_PLLON;        // 7. enable the PLL
    while( !(RCC->CR & RCC_CR_PLLRDY) );    // 8. wait for the PLLRDY flag
    
    RCC->CFGR       |= RCC_CFGR_SW_PLL;     // 9. set clock source to pll

    while( !(RCC->CFGR & RCC_CFGR_SWS_PLL) );    // 10. wait for PLL as source
    
    SystemCoreClockUpdate();                // 11. calculate the SYSCLOCK value
	
}
 
void PWM_Output(uint8_t duty_cycle){
	
	//Clock configuration register (RCC_CFGR)     ver  MCO: Microcontroller clock output 
	
	 //Port configuration register low (GPIOx_CRL)   
	 //set MODE: 0b10 out @ 2 MHz; CNF: 0b10 alternate out push-pull
	 GPIOA->CRL |= GPIO_CRL_MODE1_1 + GPIO_CRL_CNF1_1; //PA1 (PWM H Bridge)

	
	//TIMx control register 1 (TIMx_CR1) 
//	TIM2->CR1 |=   TIM_CR1_ARPE;
	
	//TIMx auto-reload register (TIMx_ARR)   Period
  TIM2->ARR = Period;     //8000 cycles
	
	//TIMx capture/compare register 1 (TIMx_CCR1)    Duty
	TIM2->CCR2 = Period-3000;//(duty_cycle/100)*Period;
	
	
	//TIMx capture/compare mode register 1 (TIMx_CCMR1) 
	TIM2->CCMR1 |= TIM_CCMR1_OC2M + TIM_CCMR1_OC2PE; //TIM_CCMR1_OC1M_2 + TIM_CCMR1_OC1M_1 + TIM_CCMR1_OC1M_0;  //111: PWM mode 2 - In upcounting, channel 1 is inactive as long as TIMx_CNT<TIMx_CCR1 else active 
	
  
	//TIMx capture/compare enable register (TIMx_CCER) 
	TIM2->CCER |= TIM_CCER_CC2E;     //Enable Output Compare in OC2 pin
	
	//TIMx control register 1 (TIMx_CR1) 
	TIM2->CR1 |= TIM_CR1_CEN;        //Enable TIM2

	TIM1
	
    
}
