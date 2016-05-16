#include "stm32f10x.h"                  // Device header
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "tim_pwm.h"

//#include "stm32f10x_crc.h"              // Keil::Device:StdPeriph Drivers:CRC

/**------------------------------------------------
  * configure timer 1 as pwm output
	* 4_ch are used
	* each ccrx value can be change independently
	* -----------------------------------------------
*/

	GPIO_InitTypeDef GPIO_InitStructure;
/**
@brief RCC configure 
   
*/
//void timer1_conf(void);
//void rcctimer_conf(void);
//void pwm_start(void);
//void GPIO_timer1_conf(void);
//void pwm_ch_invalue(u16 ch1,u16 ch2,u16 ch3,u16 ch4);
//*************************************************
void rcctimer_conf()
{
	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);
	if(RCC_WaitForHSEStartUp() == SUCCESS)
	{
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div2);
		
		//FLASH_SetLatency(FLASH_Latency_2);
		//FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		RCC_PLLCmd(ENABLE);
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource() != 0x08);
	}
  // enable clock for timer 1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOA ,ENABLE);

}

void GPIO_timer1_conf()
{
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8| GPIO_Pin_9| GPIO_Pin_10| GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0| GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

}

/**
@brief timer1 pwm config
*
*/
// configuration
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
   TIM_OCInitTypeDef TIM_OCInitStructure;	
TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
TIM_ICInitTypeDef icin;
 void timer1_conf()
 { 
     
	 	/* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 5;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period =  47970;//250Hz enter the freq 47970
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	 
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 /* Configures the TIM1 Channel1 in PWM Mode */ 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;    //23990-11999=11
	TIM_OCInitStructure.TIM_Pulse =11999;//(uint16_t) (((uint32_t) 5 * ( TIM_TimeBaseStructure.TIM_Period - 1)) / 10);   // 50% duty cycle
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;     
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	 
	

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
  
  TIM_Cmd(TIM2, ENABLE);

 }
 
 /*
 * @brief ccr register value control
 // channel duty cyc input value here function
 */
 void pwm_ch_invalue(u16 ch1,u16 ch2,u16 ch3,u16 ch4)
 {
//4d58    min 2af8     range 8800
	 TIM1->CCR1 = ch1;
	 TIM1->CCR2 = ch2;
	 TIM1->CCR3 = ch3;
	 TIM1->CCR4 = ch4;
	 
 } 
 
 /* 
  *to start pwm out put
 */
 void pwm_start()
 {
	rcctimer_conf();
	GPIO_timer1_conf();
	timer1_conf();
	TIM_Cmd(TIM1, ENABLE);
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
 }
 
 /**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
/*.............end of pwm file ............*/
