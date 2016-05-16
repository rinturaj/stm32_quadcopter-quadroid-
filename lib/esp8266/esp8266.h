
#ifndef _ESP8266_H
#define _ESP8266_H

//#include "stm32f10x_gpio.h"
//#include "stm32f10x_usart.h"
#include "misc.h"
//#include "stm32f10x_rcc.h"
#include "E:\stm32\final_zmr250\lib\esp8266\USART3_Config.h"

#ifdef __cplusplus
 extern "C" {
#endif
	 
	 
typedef struct{
					//uint8_t ConnectionNum;
					//char *DataSize;
					//uint16_t datasize;
					//char *data; //ie.. /api/foo?id=123
	        
	       // char *axis_x,*axis_y;
				     int thrust,cmd;float x_cmd,y_cmd;
}IPD_Data;

					 
	  void esp8266_Local_AP(void);
	  IPD_Data ProcessIPD_Data(char *IPD_Buffer);
		void flush_array(char buffer[], uint16_t size);
		void SetArray(char buffer[], uint16_t size);
	  void send_via_wifi(char *txData);
   void Delay_Ms(__IO uint32_t nTime);
	 
#ifdef __cplusplus
}
#endif

#endif /* ESP8266.h */
