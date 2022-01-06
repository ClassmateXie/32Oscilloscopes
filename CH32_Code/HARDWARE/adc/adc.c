#include "adc.h"
#include "delay.h"
#include "hw_config.h"
#include "led.h"
//////////////////////////////////////////////////////////////////////////////////	 
//CH32F103C8T6��Ƭ��
//ADC��������	   
//���ߣ�����
//�޸�����:2021/12/31
//�汾��V1.0							  
////////////////////////////////////////////////////////////////////////////////// 	
u16 buf_len = 512; //USB���ͻ������1200�ֽڣ�buf_len����С��600
u16 ADC_Buf[512]={0};
/*******************************************************************************
* �� �� ��         : DMA_ReSet
* ��������		   	 : DMA����	
* ��    ��         : len����������
* ��    ��         : ��
*******************************************************************************/
void DMA_ReSet(u16 len)
{
	static DMA_InitTypeDef DMA_InitStructure;
	buf_len = len;
	//========DMA����=============/
	DMA_ClearFlag(DMA1_FLAG_TC1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;//ADC��ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_Buf; //�ڴ��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //����(�����赽�ڴ�)
	DMA_InitStructure.DMA_BufferSize = len; //�������ݵĴ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //�����ַ�̶�
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //�ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord ; //�������ݵ�λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord ;    //�ڴ����ݵ�λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; //DMAģʽ��ѭ������
	DMA_InitStructure.DMA_Priority = DMA_Priority_High ; //���ȼ�����
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;   //��ֹ�ڴ浽�ڴ�Ĵ���
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);  //����DMA1��1ͨ��
	DMA_Cmd(DMA1_Channel1,ENABLE);
}
/*******************************************************************************
* �� �� ��         : TIM_ReSet
* ��������		   	 : TIM����	
* ��    ��         : Period��װ��ֵ��Prescaler��Ƶ����
* ��    ��         : ��
*******************************************************************************/
void TIM_ReSet(u16 Period,u16 Prescaler)
{
	static TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	static TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_TimeBaseStructure.TIM_Period = Period-1; //����TIM2�Ƚϵ�����
	TIM_TimeBaseStructure.TIM_Prescaler = Prescaler-1;//ϵͳ��Ƶ72M�������Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//������ϸ˵��
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_Pulse = Period>>1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//�����PWM1ҪΪLow��PWM2��ΪHigh
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_Cmd(TIM4, ENABLE);
}

void ADC1_CH1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; //����ṹ�����	
	ADC_InitTypeDef  ADC_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_ADC1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//ʹ��TIM4ʱ��
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M
	//==========�˿�����====================//
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;//ADC
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;	//ģ������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	//==========ADC����====================//
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//��ɨ��ģʽ	
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//�ر�����ת��
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T4_CC4;//��ʱ��4ͨ��4�������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���	
	ADC_InitStructure.ADC_NbrOfChannel = 1;//1��ת���ڹ��������� Ҳ����ֻת����������1 
	ADC_Init(ADC1, &ADC_InitStructure);//ADC��ʼ��
	ADC_DMACmd(ADC1, ENABLE);//ʹ��ADC1ģ��DMA
	//=========��ʱ������==============//
//	TIM_ReSet(100,72);//10K������
	//========DMA����=============/
	DMA_DeInit(DMA1_Channel1);
	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);//����DMA��������ж�
//	DMA_DeInit(DMA1_Channel1);
//	DMA_ReSet(buf_len);
//	DMA_ITConfig(DMA1_Channel1,DMA_IT_HT,ENABLE);
//	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);
//	DMA_ITConfig(DMA1_Channel1,DMA_IT_TE,ENABLE);
	//=======NVIC����============//
  /* ����P[A|B|C|D|E]0Ϊ�ж�Դ */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	ADC_Cmd(ADC1, ENABLE);//����ADת����
	ADC_ResetCalibration(ADC1);//����ָ����ADC��У׼�Ĵ���
	while(ADC_GetResetCalibrationStatus(ADC1)){};//��ȡADC����У׼�Ĵ�����״̬
	ADC_StartCalibration(ADC1);//��ʼָ��ADC��У׼״̬
	while(ADC_GetCalibrationStatus(ADC1));//��ȡָ��ADC��У׼����
	ADC_ExternalTrigConvCmd(ADC1, ENABLE);
	ADC_RegularChannelConfig(ADC1, 1, 1, ADC_SampleTime_1Cycles5);	//ADC1,ADCͨ��1,���1,1.5������,��߲���ʱ�������߾�ȷ��	
	
	TIM_InternalClockConfig(TIM4);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_UpdateDisableConfig(TIM4, DISABLE);
}

/*******************************************************************************
* �� �� ��         : DMA1_Channel1_IRQHandler
* ��������		   	 : DMAͨ��1���ж�
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void DMA1_Channel1_IRQHandler(void)
{
		u16 i;
		if(DMA_GetFlagStatus(DMA1_FLAG_TC1))
    {
				TIM_Cmd(TIM4, DISABLE);
				DMA_Cmd(DMA1_Channel1,DISABLE);
				for(i=0;i<buf_len;i++)USB_USART_SendData((ADC_Buf[i]>>8)),USB_USART_SendData(ADC_Buf[i]&0xff);
				DMA_ClearFlag(DMA1_FLAG_TC1); //���ȫ���жϱ�־
				LED=!LED;
    }
}


//�����USB���⴮�ڽ��յ�������
//databuffer:���ݻ�����
//Nb_bytes:���յ����ֽ���.
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

