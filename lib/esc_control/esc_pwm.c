
#include "E:\stm32\final_zmr250\lib\PWM\tim_pwm.h"
#include "stm32f10x.h"                  // Device header
#include "esc_pwm.h"
#include "arm_math.h"                   // ARM::CMSIS:DSP
#include "E:\stm32\final_zmr250\lib\MPU6050\MPU6050.h"
#include "esp8266.h"

#define round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))


void delay_ms(__IO uint32_t nTime);

/**
  * @brief  Initializes the esc module ......
            first put 100% throttle then waiting for the beep sound
            after down it to 0%..............
  * @param  duty:   thrust value.............
  *
  * @retval None
  */
/* 
  @ note ======================================= 
motor1Power = throttle + pitchOffset + yawOffset;
 
motor2Power = throttle + rollOffset  - yawOffsett;
 
motor3Power = throttle - pitchOffset + yawOffsett;
 
motor4Power = throttle - rollOffset  - yawOffsett;
 
*/
	
void esc_init()
{
	//above code for setting throttle value

delay_ms(9000);
pwm_ch_invalue(19800,19800,19800,19800);
delay_ms(9000);	
pwm_ch_invalue(11000,11000,11000,11000);	
}

void pwm_esc_duty(uint16_t dutyA,uint16_t dutyB,uint16_t dutyC,uint16_t dutyD)
{
	int pwmA,pwmB,pwmC,pwmD,freq_hz;
	freq_hz = TIM1->ARR;
	pwmA=(dutyA*freq_hz)/100;
	pwmB=(dutyB*freq_hz)/100;
	pwmC=(dutyC*freq_hz)/100;
	pwmD=(dutyD*(freq_hz))/100;
	pwm_ch_invalue(pwmA,pwmB,pwmC,pwmD);
	
}

void pwm_esc_all_duty(uint16_t duty)
{
	int pwmA,pwmB,pwmC,pwmD,freq_hz;
	freq_hz = TIM1->ARR;
	pwmA=(duty*freq_hz)/100;
	pwmB=(duty*freq_hz)/100;
	pwmC=(duty*freq_hz)/100;
	pwmD=(duty*freq_hz)/100;
	pwm_ch_invalue(pwmA,pwmB,pwmC,pwmD);
	
}
void stable_on_air(int throttle)
{

}

PIDdata pitch_pid;
PIDdata roll_pid;
PIDdata yaw_pid;
PIDdata roll_rate_pid;
PIDdata pitch_rate_pid;


// obj for angle
MPU_angle ag_angle;
// obj for offset value
Offset off_set;
/*
  * @brief  PID update function ......
            
  * @param  set point:   pitch,roll and yaw angle.............
  *
  * @retval pid offset value
*/
	PIDdata r;
 

Offset PID_Update(int throttle,float pitch,float roll,float yaw)
{
  
  int gain=40; 
  int i;
  u16 motor1Power,motor2Power,motor3Power,motor4Power;
	float xRate,yRate;
	float p=0,r=0,y=0,avg_pitch,avg_roll,avg_yaw;
  //-------compute angle-------------
//    for(i=0;i<100;i++)
//      {
    ag_angle=comfilter();
		
  //  ag_angle.pitch= ag_angle.pitch-0.796836674;
		ag_angle.roll= ag_angle.roll-7.99067545;
	//	ag_angle.yaw= ag_angle.yaw+8.30005169;
    xRate =ag_angle.gyro_xRate+3;
    yRate =ag_angle.gyro_yRate-1;	
    delay_ms(20);  
//      }
      
     avg_pitch = ag_angle.pitch;
     avg_roll  = ag_angle.roll;
     avg_yaw   = ag_angle.yaw;   
//      xRate = xRate/100;
//      yRate = yRate/100;
      
      p=0;r=0;y=0;
     avg_yaw =avg_yaw - 1.49100041;
  //-----------------------

  
  
  
  //--------pid loop-----------------
      
      
  off_set.pitch_offset = pid_value_loop(pitch,avg_pitch,.004,&pitch_pid);
  off_set.roll_offset = pid_value_loop(roll,avg_roll,.004,&roll_pid);
  off_set.yaw_offset = pid_value_loop(yaw,avg_yaw,.004,&yaw_pid);
 
   
//   off_set.roll_offset  =  pid_value_loop(off_set.roll_of,yRate,.001,&roll_rate_pid);
//   off_set.pitch_offset  =  pid_value_loop(off_set.pitch_of,xRate,.001,&pitch_rate_pid);   
//off_set.pitch_offset = arm_pid_f32(&PID_x,pitch_error);
//	off_set.roll_of            = arm_pid_f32(&PID_y,roll_error);
//	off_set.yaw_offset = (int)arm_pid_f32(&PID_z,yaw_error);
	
	//error_roll   =  off_set.roll_of  -  yRate;
  //error_pitch  =  off_set.pitch_of -  xRate;
  
  //off_set.pitch_offset = (int)arm_pid_f32(&PID_x_rate,error_pitch);
  //off_set.roll_offset  = (int)arm_pid_f32(&PID_y_rate,error_roll);
 
	motor1Power =(int)change_range(throttle,1) + pid_range_change( off_set.pitch_offset,gain*.75) +  pid_range_change( off_set.roll_offset,gain*.75) -  pid_range_change( off_set.yaw_offset,gain*.75);
 
  motor2Power =(int)change_range(throttle,1) + pid_range_change( off_set.pitch_offset,gain*.75) -  pid_range_change( off_set.roll_offset,gain*.75) + pid_range_change( off_set.yaw_offset,gain*.75);
 
  motor3Power =(int)change_range(throttle,1) - pid_range_change( off_set.pitch_offset,gain*.75) -  pid_range_change( off_set.roll_offset,gain*.75)- pid_range_change( off_set.yaw_offset,gain*.75);
 
  motor4Power =(int)change_range(throttle,1) - pid_range_change( off_set.pitch_offset,gain*.75) +  pid_range_change( off_set.roll_offset,gain*.75) + pid_range_change( off_set.yaw_offset,gain*.75);
	
	// update
	pwm_ch_invalue(  (u16) check_range( motor1Power)
	              ,  (u16) check_range( motor2Power)
								,  (u16) check_range( motor3Power)
								,  (u16) check_range( motor4Power));
	
	
	return off_set;
}


void delay_ms(__IO uint32_t nTime)
{ 
	int i;
  //TimingDelay = nTime;

  //while(TimingDelay != 0);
	for ( i = 0; i < nTime * 1000; i++);
}

/**
  * @brief  convert one range of numbers to another, maintaining ratio
  *
  * @param  oldvalue   the value within the old range.............
  *
  * @retval float vlue of new value within new range
  */
float change_range(float oldvalue,int select)
{
//	if(select == 1)
//{
	return((((oldvalue - 0) * (23990- 12237)) / (100 - 0)) + 12237);
//
//return((((oldvalue - 0) * (99- 49)) / (100 - 0)) + 49);
}

float pid_range_change(float p,int percentage)
   {
     float a;
     
       a=((((p - 0) * ((percentage*119))-0) / (500 - 0)) + 0); 
     
     return a;
   }
/**
  * @brief  check whether the number within the range specified
  *
  * @param  value   the value to check.............
  *
  * @retval float value will return
  */
int check_range(int value)
{
	
	if((value >12237)&&(value <23990))
	{return value;}
	else if (value > 23990)
	{return 23990;}
  else
  {return 12237;	}	
	 
}

void pid_init()
{
  pitch_pid.i_error=0;
  pitch_pid.kd = 5.000000;
  pitch_pid.Ki = 0.03;
  pitch_pid.kp = 1.1;
  pitch_pid.MAX = 500;
  pitch_pid.MIN = -500;
  
  roll_pid.i_error=0;
  roll_pid.kd = 5.000000;
  roll_pid.Ki = 0.03;
  roll_pid.kp = 1.1;
  roll_pid.MAX = 500;
  roll_pid.MIN = -500;
  
  yaw_pid.i_error=0.00000;
  yaw_pid.kd = 20.00000;
  yaw_pid.Ki = 00.00;
  yaw_pid.kp = 5;
  yaw_pid.MAX = 500;
  yaw_pid.MIN = -500;
  
  pitch_rate_pid.i_error=0;
  pitch_rate_pid.kd = 0;
  pitch_rate_pid.Ki = 0.002;
  pitch_rate_pid.kp = .0;
  pitch_rate_pid.MAX = 500;
  pitch_rate_pid.MIN = -500;
  
  roll_rate_pid.i_error=0;
  roll_rate_pid.kd = 0;
  roll_rate_pid.Ki = 0.002;
  roll_rate_pid.kp = .0;
  roll_rate_pid.MAX = 500;
  roll_rate_pid.MIN = -500;
  
}

float pid_value_loop(float setpoint,float current,float delta_time,PIDdata *s)
{
  float error,p_term,i_term,d_term,out;
  
  //current error
  error = (int)setpoint - (int) current;
  
  //p
  p_term = s->kp * error;
  
  //I
  s->i_error  = error * delta_time + s->i_error;

  i_term = s->Ki *s->i_error;
  
  //D
  d_term =  s->kd *((error - s->last_err_value)/delta_time);

  //save data 
  s->last_err_value = error;
  
  out = (p_term + i_term + d_term);
  
    if( s->MAX < out){
          out=s->MAX;s->i_error=0;
        }
    else if(out<s->MIN){
          out = s->MIN;s->i_error=0;
        }
    
  return out;  
}
