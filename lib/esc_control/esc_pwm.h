#ifndef __ESC_PWM_H
#define __ESC_PWM_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "GPIO_STM32F10x.h"  
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include "stm32f10x_rcc.h"

typedef struct{
				float pitch_offset,pitch_of,roll_of;
	      float yaw_offset;
	      float roll_offset;
}Offset;

typedef struct{
				float kp;
	      float kd;
	      float Ki;
  float last_err_value,i_error,MAX,MIN;
}PIDdata;


float pid_value_loop(float setpoint,float current,float delta_time,PIDdata *s);
void pid_init(void);
void esc_init(void);
void pwm_esc_duty(uint16_t dutyA,uint16_t dutyB,uint16_t dutyC,uint16_t dutyD);
void pwm_esc_all_duty(uint16_t duty);
Offset PID_Update(int throttle,float pitch,float roll,float yaw);
void PID_init(void);
float change_range(float oldvalue,int select);
int check_range(int value);
float pid_range_change(float p,int percentage);

#ifdef __cplusplus
}
#endif

#endif /* __MISC_H */

