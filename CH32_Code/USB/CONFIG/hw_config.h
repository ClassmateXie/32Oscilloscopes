#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H
#include "platform_config.h"
#include "usb_type.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������V3
//USB-hw_config ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/1/28
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
  
 
#define USB_USART_TXFIFO_SIZE   1200	//USB���⴮�ڷ���FIFO��С		
#define USB_USART_REC_LEN	 	200		//USB���ڽ��ջ���������ֽ���

//����һ��USB USART FIFO�ṹ��
typedef struct  
{										    
	u8  buffer[USB_USART_TXFIFO_SIZE];	//buffer
	vu16 writeptr;						//дָ��
	vu16 readptr;						//��ָ��
}_usb_usart_fifo; 
extern _usb_usart_fifo uu_txfifo;		//USB���ڷ���FIFO

extern u8  USB_USART_RX_BUF[USB_USART_REC_LEN]; //���ջ���,���USB_USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USB_USART_RX_STA;   					//����״̬���	
 
//USBͨ�ô��뺯������
void Set_USBClock(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Interrupts_Config(void);
void USB_Cable_Config (FunctionalState NewState);
void USB_Port_Set(u8 enable);
void IntToUnicode (u32 value,u8 *pbuf,u8 len);
void Get_SerialNum(void);

//��ͬUSB������ӵĺ������� 
bool USART_Config(void);
void USB_To_USART_Send_Data(uint8_t* data_buffer, uint16_t Nb_bytes);
void USART_To_USB_Send_Data(void);
void USB_USART_SendData(u8 data);
void usb_printf(char* fmt,...); 

#endif  
























