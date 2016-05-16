

#include "mytime.h"

volatile uint32_t millis = 0;
uint32_t micros = 0;
uint8_t freqMhz = 72;

void Init_Time(Time_Resolution res, uint32_t clockMhz)
{
	freqMhz = clockMhz;
SysTick_Config((SystemCoreClock/res));

//Just making sure that the Systick Interrupt is top priority
//Or the timer wont be accurate
NVIC_SetPriority(SysTick_IRQn, 0);

}

/*
void Delay(uint16_t milliseconds)
{

}
*/

uint32_t Micros()
{
	micros = millis * 1000 + 1000 - SysTick->VAL / freqMhz;
	return micros;
}

uint32_t Millis()
{
return millis;
}

void SysTick_Handler()
{
	millis++;
}

//
