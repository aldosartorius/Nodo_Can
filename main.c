 
#include "stm32f10x.h"
#include <stdint.h>
#include <stdbool.h>
 
 int main(void);
 void Init_Clock(void);
 void Init_GPIO(void);
 void PWM_Output(float);
 void Init_Encoder_Counter(void);
 void Init_External_Interrupt(void);
 
#define Frecuency_Hz (20000)
#define Period_in_clock_cycles   ((SystemCoreClock)/(Frecuency_Hz))-1     //APB1 Timer-CLK 72 MHz 72,000,000/20000= 3600 ciclos
 
 //Global variables
int32_t Encoder_Pulses;
float Encoder_grades;
bool Flag =0;          //Flag = 0 means positive turn while Flag = 1 negative turn
 
 
 int main(void){

	 //Advance Port Bus 2 enabled: AFIO, GPIO_A & GPIO_B
	 RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN + RCC_APB2ENR_IOPBEN;
	 
	 //Advance Port Bus 1 enabled: TIM2 (PWM) & TIM4 (Encoder)
	 RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // enable Timer2 clock
	 RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;  // enable Timer4 clock
	 
	 Init_Clock();
	 Init_External_Interrupt();
	 Init_Encoder_Counter();
	 PWM_Output(10);
	  
	 
   while(1){
	
		 
 }
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

void Init_GPIO(void){
	
	 GPIOA->CRL |= GPIO_CRL_MODE2_1 + GPIO_CRL_MODE2_0; //PA2 Output mode 2 MHz  (A1 H Bridge )
	 GPIOA->CRL |= GPIO_CRL_MODE3_1 + GPIO_CRL_MODE3_0; //PA3 Output mode 2 MHz  (A2 H Bridge
		 
}

void PWM_Output(float duty_cycle){
	
	 //Port configuration register low (GPIOx_CRL)   
	 //set MODE: 0b10 out @ 2 MHz; CNF: 0b10 alternate out push-pull
	 GPIOA->CRL &= ~GPIO_CRL_CNF1_0;
	 GPIOA->CRL |= GPIO_CRL_MODE1_1 + GPIO_CRL_CNF1_1; //PA1 (PWM H Bridge)
	
	 //TIMx auto-reload register (TIMx_ARR) 
  TIM2->ARR = Period_in_clock_cycles;     //3600 cycles
	
	//TIMx capture/compare register 1 (TIMx_CCR1)    Duty
	float duty_clock_cycles = Period_in_clock_cycles-(duty_cycle/100)*Period_in_clock_cycles;
	TIM2->CCR2 =duty_clock_cycles;
	
	//TIMx capture/compare mode register 1 (TIMx_CCMR1) 
	TIM2->CCMR1 |= TIM_CCMR1_OC2M + TIM_CCMR1_OC2PE; //111: PWM mode 2 - In upcounting, channel 1 is inactive as long as TIMx_CNT<TIMx_CCR1 else active 
	
	//TIMx capture/compare enable register (TIMx_CCER) 
	TIM2->CCER |= TIM_CCER_CC2E;     //Enable Output Compare in OC2 pin
	
	//TIMx control register 1 (TIMx_CR1) 
	TIM2->CR1 |= TIM_CR1_CEN;        //Enable TIM2
   
}

 void Init_Encoder_Counter(void){   //Configure TIM4 for quadrature encoder read
	
	
	 //Port configuration register high (GPIOx_CRH) 
	 GPIOB->CRL &= ~GPIO_CRL_CNF6_0;
	 GPIOB->CRL |= GPIO_CRL_CNF6_1; //PB6 Input mode pullup/pulldown (Encoder A signal)
	 GPIOB->CRL &= ~GPIO_CRL_CNF7_0;
	 GPIOB->CRL |= GPIO_CRL_CNF7_1; //PB7 Input mode pullup/pulldown (Encoder B signal)
	 
	 
	 //TIMx capture/compare mode register 1 (TIMx_CCMR1) 
	 TIM4->CCMR1 |= TIM_CCMR1_CC1S_0 + TIM_CCMR1_CC1S_1; // 01: CC1 channel is configured as input, IC1 is mapped on TI1
	 TIM4->CCMR1 |= TIM_CCMR1_CC2S_0 + TIM_CCMR1_CC2S_1; // 01: CC2 channel is configured as input, IC2 is mapped on TI2 
	 
	 //TIMx slave mode control register (TIMx_SMCR)   *****2 MULTIPLICATOR *******
	 TIM4->SMCR |=  TIM_SMCR_SMS_1;  //010: Encoder mode 2 - Counter counts up/down on TI1FP2 edge depending on TI2FP1 level. 
	  
 
   //TIMx capture/compare enable register (TIMx_CCER) 
	 TIM4->CCER |= TIM_CCER_CC4E;     //Enable Output Compare in OC2 pin
	 
	 //TIMx control register 1 (TIMx_CR1) 
	 TIM4->CR1 |= TIM_CR1_CEN;    //Enable TIM4
	 	
}
 

void Init_External_Interrupt(void){
	
	  __disable_irq();
	 
	 //External interrupt configuration register 2 (AFIO_EXTICR2) 
	 AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PB;     //PB[6] pin
	 
	 //Interrupt mask register (EXTI_IMR) 
	 EXTI->IMR |=   EXTI_IMR_MR6;     //0x00000040
	 
	 //Rising trigger selection register (EXTI_RTSR) 
	 EXTI->RTSR |= EXTI_RTSR_TR6;      //0x00000040
	 
	 //Falling trigger selection register (EXTI_FTSR) 
	 EXTI->FTSR |= EXTI_FTSR_TR6;     //0x00000040
	 
	 //Enable NVIC interface
	 NVIC_EnableIRQ(EXTI9_5_IRQn);    
	 
	 //Set interrupt as pending
	 NVIC_SetPendingIRQ(EXTI9_5_IRQn);
	
	 //Enable global interrup
	 __enable_irq();                 
}

float EXTI9_5_IRQHandler(void){
	
	 
	 //TIMx counter (TIMx_CNT)                            //Actual pulse read
	 Encoder_Pulses = TIM4->CNT;   
	 
	
	if(Flag == 0 ){                                                      //Flag = 0 UIF = 0
		if(!(TIM4->SR & 0x1)){ //IUF = 0
				Encoder_grades = (float)(Encoder_Pulses*360)/117484; //100 PPR * 2 * 127.7 * 4.6 =  117484  
		}	
		else{ //IUF = 1
			Flag = 1;                                                        //Flag = 0 UIF = 1
			TIM4->SR = 0x0;   //UIF=0x0 Reset IUF
			Encoder_grades =  (float)((Encoder_Pulses-65536)*360)/117484;   
		}
	}
	else{                                                                //Flag = 1 UIF = 0
			if(!(TIM4->SR & 0x1)){ //IUF = 0
				Encoder_grades =  (float)((Encoder_Pulses-65536)*360)/117484;    
		  }
			else{ //IUF = 1
				Flag = 0;                                                      //Flag = 1 UIF = 1
				TIM4->SR = 0x0;   //UIF=0x0 Reset IUF
				Encoder_grades =  (float)(Encoder_Pulses*360)/117484;          
				}
	}
	
	//Pending register (EXTI_PR) 
	EXTI->PR |= EXTI_PR_PR6;
 return Encoder_grades;
}




