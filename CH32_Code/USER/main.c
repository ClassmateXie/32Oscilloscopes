#include "led.h"
#include "delay.h"
#include "sys.h"
#include "adc.h"
//移植USB时候的头文件
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
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//使能TIM2时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = Period-1; //设置TIM2比较的周期
	TIM_TimeBaseStructure.TIM_Prescaler = Prescaler-1;//系统主频72M，这里分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//下面详细说明
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_Pulse = Period>>1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//如果是PWM1要为Low，PWM2则为High
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;
	NVIC_Init(&NVIC_InitStructure);//初始化中断，设置中断的优先级
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//开启定时器中断
	
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);//定时器2 通道1
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);//定时器2 通道2
	
	TIM_Cmd(TIM2, ENABLE);
}

int main(void)
{	
	Buf_Dataset(LED_Breathe_Buf);
	delay_init();	  	 	//延时函数初始化
 	LED_Init();			   	//LED端口初始化
	USB_Init();				 	//USB虚拟串口初始化
	ADC1_CH1_Init();		//ADC初始化
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
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);//清除中断标志位
	}
}
