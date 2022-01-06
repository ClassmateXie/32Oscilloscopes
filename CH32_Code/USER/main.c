#include "led.h"
#include "delay.h"
#include "sys.h"
#include "adc.h"
//��ֲUSBʱ���ͷ�ļ�
#include "usb_lib.h"
#include "hw_config.h"
#include "usb_pwr.h"

u16 LED_Breathe_Buf[1000];
void Buf_Dataset(u16* buf)
{
	u16 temp;
	for(temp=0;temp<500;temp++)buf[temp]=temp*2+100;
	for(temp=0;temp<500;temp++)buf[temp+500]=999-temp*2+100;
}

void TIM2_Init(u16 Period,u16 Prescaler)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//ʹ��TIM2ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;	//�����������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = Period-1; //����TIM2�Ƚϵ�����
	TIM_TimeBaseStructure.TIM_Prescaler = Prescaler-1;//ϵͳ��Ƶ72M�������Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//������ϸ˵��
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_Pulse = Period>>1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//�����PWM1ҪΪLow��PWM2��ΪHigh
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;
	NVIC_Init(&NVIC_InitStructure);//��ʼ���жϣ������жϵ����ȼ�
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//������ʱ���ж�
	
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);//��ʱ��2 ͨ��1
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);//��ʱ��2 ͨ��2
	
	TIM_Cmd(TIM2, ENABLE);
}

int main(void)
{	
	Buf_Dataset(LED_Breathe_Buf);
	delay_init();	  	 	//��ʱ������ʼ��
 	LED_Init();			   	//LED�˿ڳ�ʼ��
	USB_Init();				 	//USB���⴮�ڳ�ʼ��
	ADC1_CH1_Init();		//ADC��ʼ��
	TIM2_Init(1200,60); //72000000/1200/60=1000Hz
	while(1)
	{
		LED=PAin(2);
	}	 
}

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET)
	{
		static u16 cnt;
		TIM_SetCompare3(TIM2,LED_Breathe_Buf[cnt]);
		if(cnt++>=1000)cnt=0;
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);//����жϱ�־λ
	}
}
