#include "stm32f10x.h"                  // Device header
#include "arm_math.h"
#include "stm32f10x_crc.h"              // Keil::Device:StdPeriph Drivers:CRC
#include "stm32f10x_dma.h"              // Keil::Device:StdPeriph Drivers:DMA
#include "stm32f10x_exti.h"             // Keil::Device:StdPeriph Drivers:EXTI
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_i2c.h"              // Keil::Device:StdPeriph Drivers:I2C
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_pwr.h"              // Keil::Device:StdPeriph Drivers:PWR
#include "stm32f10x_usart.h"            // Keil::Device:StdPeriph Drivers:USART
#include "stm32f10x_wwdg.h"             // Keil::Device:StdPeriph Drivers:WWDG
#include "E:\stm32\final_zmr250\lib\MPU6050\MPU6050.h"
#include "E:\stm32\final_zmr250\lib\esp8266\USART3_Config.h"
#include "E:\stm32\final_zmr250\lib\esp8266\esp8266.h"
#include "E:\stm32\final_zmr250\lib\PWM\tim_pwm.h"
#include "E:\stm32\final_zmr250\lib\esc_control\esc_pwm.h"
#include <stdio.h>
#include <string.h>
#include "RTE_Components.h"             // Component selection
#include "RTE_Device.h"                 // Keil::Device:Startup
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "BMP180.h"

// ARM::CMSIS:DSP
/**	Notes: 
 *		- Under "Options for target" > "C/C++" > "Define" you must add 2 defines :
 *			- ARM_MATH_CM3
 *			- __FPU_PRESENT=1
 */
//----------------------------------------------------------------------------------
//***********************************************************************************
char USART3_RxBuffer[35],t,copy[25],temp[25];
int rx_buffer,throttle,cmd;bool settings= true,pid_loop=false;
float error,value;
Offset off;
float x,y,z,pitch,roll,a,b,yaw;
bool get_value(int dt);
int main()
{
	
	SystemInit();
	pwm_start();
  
	GPIO_SetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
  
	pid_init();
	Init_USART3(9600, ENABLE);
	mpu6050_setup();
  esp8266_Local_AP();
	memset(USART3_RxBuffer,0x00,35);    //clear buffer
  
  
	GPIO_ResetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
	//BMP180_Reset();
   
	while(1)
	{
		strcpy(copy,USART3_RxBuffer);
    
    if((strcmp(copy,temp))!=0)
		{
      GPIO_ResetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
//GPIO_SetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3); 
	strtok(copy,"$");
  cmd=atoi(strtok(NULL,"@"));
  throttle=atoi(strtok(NULL,"#"));
	x=atof(strtok(NULL,"&"));
  y=atof(strtok(NULL,"%"));
  strcpy(temp,copy);
	get_value(cmd);			    
 
		}

//
   // a = BMP180_Read_PT(0);
     //   b = altitude();
  //  b=temperature();
//***********start code from here*************


   if (TIM_GetFlagStatus(TIM2, TIM_FLAG_Update) != RESET)
    {
  if(pid_loop==true)
  {

      GPIO_SetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
	    off =  PID_Update(throttle,x,y,0);
     
      GPIO_ResetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
    }
   TIM_ClearFlag(TIM2, TIM_IT_Update);
  }

	}
}

//*----------------------------------------------------
//
//
//-----------------------------------------------------

bool get_value(int dt)
{

	switch(cmd )
	{
		case 0: 
      pwm_ch_invalue(11999,11999,11999,11999);
     // settings= true;
    pid_loop=false;
		        break;
			     
		case 1:
			      pid_loop=true;break;
			    
		case 2:pid_loop=false;
// pwm_ch_invalue(23990,23990,23990,23990);
//	         if(ipddata.thrust>=0)
//					 {
//             ipddata.thrust=ipddata.thrust-10;
//             PID_Update(ipddata.thrust,0,0,0);
//             Delay_Ms(200);
//					 }	
//          if(	ipddata.thrust<=0)
//              {
//                ipddata.cmd= 0;
//                 }  
		
		
		break;//landing algorithm use while()
			
		case 3:
       GPIO_SetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
    pid_loop=false;NVIC_SystemReset();break;
			
			     
		default:break;
					 }
    return 0;
}



//-----------------------------------------------------
//************** usart interrupt routine****************************
//-----------------------------------------------------
//void USART3_IRQHandler(void) //USART3 - ESP8266 Wifi Module
//{
//  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
//  {
//		
//	   USART3_RxBuffer[rx_buffer++] = USART_ReceiveData(USART3);flag=1;       
//      
//  }
// 
// USART_ClearITPendingBit(USART3,USART_IT_RXNE);
//}
void USART3_IRQHandler(void) //USART3 - ESP8266 Wifi Module
{
  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
  {
		
	   t = USART_ReceiveData(USART3);      
      
  }
  if( (t != '\n') && (rx_buffer < 35) ){ 
			 USART3_RxBuffer[rx_buffer] = t;
			rx_buffer++;
		}
		else { // otherwise reset the character counter and print the received string
			 rx_buffer= 0;	//memset(USART3_RxBuffer,0x00,35);
			
		}
  USART_ClearITPendingBit(USART3,USART_IT_RXNE);
}
