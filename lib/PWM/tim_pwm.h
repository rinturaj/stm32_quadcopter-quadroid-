#ifndef __TIM_PWM_H
#define __TIM_PWM_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "GPIO_STM32F10x.h"  
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include "stm32f10x_rcc.h"
/**
*
* header file for pwm output 
* from timer 1
*
*
*/
void timer1_conf(void);
void rcctimer_conf(void);
void pwm_start(void);
void GPIO_timer1_conf(void);
void pwm_ch_invalue(u16 ch1,u16 ch2,u16 ch3,u16 ch4);

#ifdef __cplusplus
}
#endif

#endif /* __MISC_H */

