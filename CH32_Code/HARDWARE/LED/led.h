#ifndef __LED_H
#define __LED_H	 
#include "sys.h"

#define LED  PCout(13)
void LED_Init(void);//≥ı ºªØ
void GPI_Init(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,BitAction BitVal);
void GPO_Init(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,BitAction BitVal); 				    
#endif
