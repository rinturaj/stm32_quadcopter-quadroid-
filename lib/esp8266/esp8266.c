

#include "esp8266.h"
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

void Delay_Ms(__IO uint32_t nTime);

void esp8266_Local_AP()
{
	//char *ok="ok";
	USART3_SendString("AT \r");
	 // wait
	Delay_Ms(2000);
	USART3_SendString("AT+RST \r");
		 // wait
	Delay_Ms(2000);
	USART3_SendString("ATE0 \r");
		 // wait
	Delay_Ms(2000);
	USART3_SendString("AT+CWMODE=3 \r");
	Delay_Ms(2000);
	USART3_SendString("AT+CIPMUX=1 \r");
	Delay_Ms(2000);
	USART3_SendString("AT+CIPSERVER=1,80 \r");
	
}

void flush_array(char buffer[], uint16_t size)
	{
		
		memset(buffer, '\0', size);
	}

void SetArray(char buffer[], uint16_t size)
	{
		memset(buffer, '1', size);
	}

//Breaks the IPD message into a proper request object
	// +IPD,0,5:,thrust,x_axis,y_axis,;
	IPD_Data thisIPDMessage; 
	
IPD_Data ProcessIPD_Data(char *IPD_Buffer)
{


	
	strtok(IPD_Buffer,"$");
  thisIPDMessage.cmd=atoi(strtok(NULL,"@"));
  thisIPDMessage.thrust=atoi(strtok(NULL,"#"));
	thisIPDMessage.x_cmd=atof(strtok(NULL,"&"));
  thisIPDMessage.y_cmd=atof(strtok(NULL,"%"));
	

	Delay_Ms(50);
		
	return thisIPDMessage;
	
}

void send_via_wifi(char *txData)
{ 
	char str[30];

	sprintf(str,"AT+CIPSEND=0,%d",strlen(txData));
	Delay_Ms(10);
	USART3_SendString(str);
	Delay_Ms(10);
	USART_SendData(USART3,0x0D);
	Delay_Ms(100);
	USART3_SendString(txData);
	Delay_Ms(10);
	
}

void Delay_Ms(__IO uint32_t nTime)
{ 
	int i;
//	NVIC_DisableIRQ(USART3_IRQn);
	for ( i = 0; i < nTime * 1000; i++);
//	NVIC_EnableIRQ(USART3_IRQn);
}
