 /*                          ***********                                    */
 /*                          *         *                                  */
 /*                          *         *                                           */
 /*                          *         *                                           */
/*                           *         *                                           */
 /*                          *         *<--PB11 (UART Rx3)                                           */
 /*                          *         *-->PB10 (UART Tx3)                                            */
 /*                          *         *                                           */
 /*                          *         *                                           */
 /*                          *         *                                           */
 /*                          *         *                                           */
 /*                          *         *                                           */
 /*                          *         *                                           */
 /*                          *         *-->PA3 (PWM A2)                                           */
 /*        (T4 Enc A) PB6 -->*         *-->PA2 (PWM A1)                                           */
 /*        (T4 Enc B) PB7 -->*         *-->PA1 (PWM Enable)                                          */
 /*          (CAN Rx) PB8 -->*         *                                           */
 /*          (CAN Tx) PB9 <--*         *                                           */
 /*                          *         *                                           */
 /*                          *         *                                           */
 /*                          *         *                                           */
 /*                          ***********                                           */

 

#include "stm32f10x.h"
#include <stdint.h>
#include <stdbool.h>

#include "controller.h"
 
 typedef struct
{
double dState; // Last position input
double iState; // Integrator state
double iMax, iMin; // Maximum and minimum allowable integrator state
double iGain, // integral gain
pGain, // proportional gain
dGain; // derivative gain
} SPid;
 
 
 int main(void);
 void Init_Clock(void);
 void Init_GPIO(void);
 void PWM_Output(float);
 void Init_Encoder_Counter(void);
 void Init_External_Interrupt(void);
 void PID_Init(void);
 float PID(SPid *, float , float );

 
 void USART_Init(void);
 void USART_Send(int8_t);
 
 
 
#define Frecuency_Hz (500000)
#define Period_in_clock_cycles   ((SystemCoreClock)/(Frecuency_Hz))-1     //APB1 Timer-CLK 72 MHz 72,000,000/20000= 3600 ciclos



//By default the clock is 72 MHz.
#define UART_BAUDRATE        (115200)
#define UART_BBR_VALUE  (SystemCoreClock/UART_BAUDRATE)      //APB1 Timer-UART 36,000,000/115200 = 312 = 0x138



 
 //Global variables
int32_t Encoder_Pulses;
float Encoder_grades;
bool Flag =0;          //Flag = 0 means positive turn while Flag = 1 negative turn

SPid PID_Controller;
float control_signal = 0.0;



 
 
 int main(void){

	 //Advance Port Bus 1 enabled: USART3
	 RCC->APB1ENR |= RCC_APB1ENR_USART3EN; 
	 
	 //Advance Port Bus 2 enabled: AFIO, GPIO_A & GPIO_B
	 RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN + RCC_APB2ENR_IOPBEN;
	 
	 //Advance Port Bus 1 enabled: TIM2 (PWM) & TIM4 (Encoder)
	 RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // enable Timer2 clock
	 RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;  // enable Timer4 clock
	 
	 Init_Clock();
	 Init_External_Interrupt();
	 Init_Encoder_Counter();
	 
	 
   while(1){
	
		  control_signal = PID(&PID_Controller, Encoder_grades, 90);
	    PWM_Output(control_signal);
		 
 }
}
 

void PID_Init(void){

   
	 PID_Controller.pGain = 0.1;
	 PID_Controller.iGain = 0;
	 PID_Controller.dGain = 0;
	
	 PID_Controller.iMax = 10;
	 PID_Controller.iMin = -10;
	
	 PID_Controller.iState = 0;
	 PID_Controller.dState = 0;

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
	
	
}

void PWM_Output(float control_signal){
	
	float duty_cycle_percent = 0;
	float duty_clock_cycles = 0; 
	uint8_t Max_sink_voltaje = 24;
	//Control signal saturation (Max an Minus DC sink value)
	if (control_signal < - 24) control_signal = -24;
	else if (control_signal > 24) control_signal = 24;
	
	//Configure PA2 & PA3 as outputs 
	GPIOA->CRL |= GPIO_CRL_MODE2_1 + GPIO_CRL_MODE2_0; //PA2 Output mode 2 MHz  (A1 H Bridge )
	GPIOA->CRL |= GPIO_CRL_MODE3_1 + GPIO_CRL_MODE3_0; //PA3 Output mode 2 MHz  (A2 H Bridge
		 
	
	 //Port configuration register low (GPIOx_CRL)   
	 //set MODE: 0b10 out @ 2 MHz; CNF: 0b10 alternate out push-pull
	 GPIOA->CRL &= ~GPIO_CRL_CNF1_0;
	 GPIOA->CRL |= GPIO_CRL_MODE1_1 + GPIO_CRL_CNF1_1; //PA1 (PWM H Bridge)
	
	 //TIMx auto-reload register (TIMx_ARR) 
  TIM2->ARR = Period_in_clock_cycles;     //3600 cycles
	
	//TIMx capture/compare register 1 (TIMx_CCR1)    Duty
	
	if(control_signal > 0 && control_signal < 24){
	  
		//Port output data register (GPIOx_ODR) (x=A..G) 
		GPIOA->ODR |= GPIO_ODR_ODR0;                      //Motor turn right
		duty_cycle_percent = control_signal/Max_sink_voltaje;
	}
	
	if(control_signal < 0 && control_signal > -24){
	  
		//Port output data register (GPIOx_ODR) (x=A..G) 
		GPIOA->ODR |= GPIO_ODR_ODR1; 		//Motor turn left
		duty_cycle_percent = -(control_signal/Max_sink_voltaje);
	}
	
	if(control_signal == 0 ){
	  
		//Port output data register (GPIOx_ODR) (x=A..G) 
		GPIOA->ODR |= 0x0;                      //Motor turn left
		duty_cycle_percent = 0;
	}
	
	duty_clock_cycles = Period_in_clock_cycles-(duty_cycle_percent/1)*Period_in_clock_cycles;
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

void USART_Init(){
	
	//PA 11 Rx
	//PA 12 Tx
	
	//Port configuration register high (GPIOx_CRH)
	GPIOA->CRH |= ~GPIO_CRH_CNF11_0 + GPIO_CRH_CNF11_1;                          //PA11  Input mode/Pull Up
	GPIOA->CRH |= GPIO_CRH_MODE12_0 + ~GPIO_CRH_CNF12_0 + GPIO_CRH_CNF12_1 ;     //PA12  Output mode/ Push pull
	
	//Baud rate register (USART_BRR)
	USART3->BRR = UART_BBR_VALUE;          //0x138   
	
	//Control register 1 (USART_CR1) 
	USART3->CR1 |=  USART_CR1_UE + USART_CR1_TE + USART_CR1_RE;    //USART ENABLE, TRANSMIT ENABLE, RECIVE ENABLE
		
}

void USART_Send(int8_t ch){
	
		while(!(USART3->SR & 0x80)){   //TXE: Transmit data register empty
			
			USART1->DR = (ch & 0xFF);
			
		}

}

float PID(SPid * pid, float real_position, float desired_position)
{
	
	float error = desired_position -real_position;
	float pTerm, dTerm, iTerm;
	pTerm = pid->pGain * error; // calculate the proportional term

	
	// calculate the integral state with appropriate limiting
	pid->iState += error;
	
	if (pid->iState > pid->iMax) pid->iState = pid->iMax;
	else if (pid->iState < pid->iMin) pid->iState = pid->iMin;

	iTerm = pid->iGain * pid->iState; // calculate the integral term

	
	dTerm = pid->dGain * (pid->dState - real_position);
	pid->dState = real_position;

	return pTerm + dTerm + iTerm;

}



