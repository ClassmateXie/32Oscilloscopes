#include "adc.h"
#include "delay.h"
#include "hw_config.h"
#include "led.h"
//////////////////////////////////////////////////////////////////////////////////	 
//CH32F103C8T6单片机
//ADC驱动代码	   
//作者：纯粹
//修改日期:2021/12/31
//版本：V1.0							  
////////////////////////////////////////////////////////////////////////////////// 	
u16 buf_len = 512; //USB发送缓存最大1200字节，buf_len必须小于600
u16 ADC_Buf[512]={0};
/*******************************************************************************
* 函 数 名         : DMA_ReSet
* 函数功能		   	 : DMA重启	
* 输    入         : len传输数据量
* 输    出         : 无
*******************************************************************************/
void DMA_ReSet(u16 len)
{
	static DMA_InitTypeDef DMA_InitStructure;
	buf_len = len;
	//========DMA配置=============/
	DMA_ClearFlag(DMA1_FLAG_TC1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;//ADC地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_Buf; //内存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //方向(从外设到内存)
	DMA_InitStructure.DMA_BufferSize = len; //传输内容的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址固定
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord ; //外设数据单位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord ;    //内存数据单位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; //DMA模式：循环传输
	DMA_InitStructure.DMA_Priority = DMA_Priority_High ; //优先级：高
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;   //禁止内存到内存的传输
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);  //配置DMA1的1通道
	DMA_Cmd(DMA1_Channel1,ENABLE);
}
/*******************************************************************************
* 函 数 名         : TIM_ReSet
* 函数功能		   	 : TIM重启	
* 输    入         : Period重装载值，Prescaler分频因子
* 输    出         : 无
*******************************************************************************/
void TIM_ReSet(u16 Period,u16 Prescaler)
{
	static TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	static TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_TimeBaseStructure.TIM_Period = Period-1; //设置TIM2比较的周期
	TIM_TimeBaseStructure.TIM_Prescaler = Prescaler-1;//系统主频72M，这里分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//下面详细说明
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_Pulse = Period>>1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//如果是PWM1要为Low，PWM2则为High
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_Cmd(TIM4, ENABLE);
}

void ADC1_CH1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; //定义结构体变量	
	ADC_InitTypeDef  ADC_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_ADC1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//使能TIM4时钟
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M
	//==========端口设置====================//
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;//ADC
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;	//模拟输入
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	//==========ADC配置====================//
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T4_CC4;//定时器4通道4触发检测
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
	ADC_InitStructure.ADC_NbrOfChannel = 1;//1个转换在规则序列中 也就是只转换规则序列1 
	ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
	ADC_DMACmd(ADC1, ENABLE);//使能ADC1模块DMA
	//=========定时器配置==============//
//	TIM_ReSet(100,72);//10K采样率
	//========DMA配置=============/
	DMA_DeInit(DMA1_Channel1);
	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);//开启DMA传输完成中断
//	DMA_DeInit(DMA1_Channel1);
//	DMA_ReSet(buf_len);
//	DMA_ITConfig(DMA1_Channel1,DMA_IT_HT,ENABLE);
//	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);
//	DMA_ITConfig(DMA1_Channel1,DMA_IT_TE,ENABLE);
	//=======NVIC配置============//
  /* 配置P[A|B|C|D|E]0为中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	ADC_Cmd(ADC1, ENABLE);//开启AD转换器
	ADC_ResetCalibration(ADC1);//重置指定的ADC的校准寄存器
	while(ADC_GetResetCalibrationStatus(ADC1)){};//获取ADC重置校准寄存器的状态
	ADC_StartCalibration(ADC1);//开始指定ADC的校准状态
	while(ADC_GetCalibrationStatus(ADC1));//获取指定ADC的校准程序
	ADC_ExternalTrigConvCmd(ADC1, ENABLE);
	ADC_RegularChannelConfig(ADC1, 1, 1, ADC_SampleTime_1Cycles5);	//ADC1,ADC通道1,序号1,1.5个周期,提高采样时间可以提高精确度	
	
	TIM_InternalClockConfig(TIM4);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_UpdateDisableConfig(TIM4, DISABLE);
}

/*******************************************************************************
* 函 数 名         : DMA1_Channel1_IRQHandler
* 函数功能		   	 : DMA通道1的中断
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void DMA1_Channel1_IRQHandler(void)
{
		u16 i;
		if(DMA_GetFlagStatus(DMA1_FLAG_TC1))
    {
				TIM_Cmd(TIM4, DISABLE);
				DMA_Cmd(DMA1_Channel1,DISABLE);
				for(i=0;i<buf_len;i++)USB_USART_SendData((ADC_Buf[i]>>8)),USB_USART_SendData(ADC_Buf[i]&0xff);
				DMA_ClearFlag(DMA1_FLAG_TC1); //清除全部中断标志
				LED=!LED;
    }
}


//处理从USB虚拟串口接收到的数据
//databuffer:数据缓存区
//Nb_bytes:接收到的字节数.
void USB_To_USART_Send_Data(u8* data_buffer, u16 Nb_bytes)
{
	u16 i,arr,div,num;
	static u8 temp,step,buf[6],cnt;
	for(i=0;i<Nb_bytes;i++)
	{
		temp = data_buffer[i];
		switch(step)
		{
			case 0:if(temp==0xa5)step=1;break;
			case 1:if(temp==0x5a)step=2;else if(temp==0xa5)step=1;else step=0;break;
			case 2:buf[cnt++]=temp;if(cnt>=6)step=3,cnt=0;break;
			case 3:if(temp==0xff)
						{
							arr=buf[0]*256+buf[1];
							div=buf[2]*256+buf[3];	
							num=buf[4]*256+buf[5];	
							DMA_ReSet(num);
							TIM_ReSet(arr,div);
							step=0;
						}
						else if(temp==0xa5)step=1;
						else step=0;
						break;
		}	
	}
} 

