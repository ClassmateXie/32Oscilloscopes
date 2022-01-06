#include "led.h"

//////////////////////////////////////////////////////////////////////////////////	 
//CH32F103C8T6单片机
//LED驱动代码	   
//作者：纯粹
//修改日期:2021/12/31
//版本：V1.0							  
////////////////////////////////////////////////////////////////////////////////// 	   

//初始化PC13为输出口.并使能时钟		    
//LED IO初始化
void LED_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //使能PC端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;				 //LED-->PC13端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOC, &GPIO_InitStructure);	  				 //推挽输出 ，IO口速度为50MHz
}

void GPO_Init(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,BitAction BitVal)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOx, &GPIO_InitStructure);	  				 //推挽输出 ，IO口速度为50MHz
	GPIO_WriteBit(GPIOx,GPIO_Pin,BitVal);
}
void GPI_Init(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,BitAction BitVal)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
	if(BitVal==0)GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
	else GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOx, &GPIO_InitStructure);	  				 //上/下拉输入 ，IO口速度为50MHz
	GPIO_WriteBit(GPIOx,GPIO_Pin,BitVal);
}

