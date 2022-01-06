#include "led.h"

//////////////////////////////////////////////////////////////////////////////////	 
//CH32F103C8T6��Ƭ��
//LED��������	   
//���ߣ�����
//�޸�����:2021/12/31
//�汾��V1.0							  
////////////////////////////////////////////////////////////////////////////////// 	   

//��ʼ��PC13Ϊ�����.��ʹ��ʱ��		    
//LED IO��ʼ��
void LED_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //ʹ��PC�˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;				 //LED-->PC13�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOC, &GPIO_InitStructure);	  				 //������� ��IO���ٶ�Ϊ50MHz
}

void GPO_Init(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,BitAction BitVal)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOx, &GPIO_InitStructure);	  				 //������� ��IO���ٶ�Ϊ50MHz
	GPIO_WriteBit(GPIOx,GPIO_Pin,BitVal);
}
void GPI_Init(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,BitAction BitVal)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
	if(BitVal==0)GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
	else GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOx, &GPIO_InitStructure);	  				 //��/�������� ��IO���ٶ�Ϊ50MHz
	GPIO_WriteBit(GPIOx,GPIO_Pin,BitVal);
}

