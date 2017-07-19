#include "usart.h"


//Zmienne====================================================================================
FIFO_type USART_PC_RX_Buffer={{},0,0};
FIFO_type USART_PC_TX_Buffer={{},0,0};


//Funckcja inicjujaca========================================================================
void USART_PC_Config()
{
	//Struktury=============================================================
	GPIO_InitTypeDef  	GPIO_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure;
	USART_InitTypeDef 	USART_InitStructure;

	//RCC===================================================================
	RCC_APB2PeriphClockCmd(USART_PC_CLOCK | USART_PC_PORT_CLOCK | RCC_APB2Periph_AFIO, ENABLE);

	//GPIO==================================================================
	GPIO_InitStructure.GPIO_Pin = USART_PC_PIN_TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART_PC_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = USART_PC_PIN_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART_PC_PORT, &GPIO_InitStructure);

	//NVIC==================================================================
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = USART_PC_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//UART==================================================================
	USART_InitStructure.USART_BaudRate=1000000;
	USART_InitStructure.USART_Mode=USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_Init(USART_PC,&USART_InitStructure);
	USART_ITConfig(USART_PC,USART_IT_RXNE,ENABLE);
	USART_ITConfig(USART_PC,USART_IT_TXE,ENABLE);
	USART_Cmd(USART_PC,ENABLE);
}

void usartSendString(char* string)
{
	uint8_t index=0;
	while(string[index]!='\0')
		FIFO_Put(&USART_PC_TX_Buffer,string[index++]);
	USART_ITConfig(USART_PC, USART_IT_TXE, ENABLE);
}
void usartSendInt(uint16_t number)
{
	uint8_t length=1;
	uint16_t helpnumber;
	uint8_t i=0;
	if (number>0x7FFF)
	{
		FIFO_Put(&USART_PC_TX_Buffer,(char)('-'));
		number*=-1;
	}
	helpnumber=number;
	while (helpnumber>9)
	{
		helpnumber/=10;
		length++;
	}
	helpnumber=1;
	for (i=0;i<length-1;i++) helpnumber*=10;
	for (i=0;i<length;i++)
	{
		FIFO_Put(&USART_PC_TX_Buffer,(char)(number/helpnumber+48));
		number%=helpnumber;
		helpnumber/=10;
	}
	USART_ITConfig(USART_PC,USART_IT_TXE,ENABLE);
}
void usartSendInt8(int8_t number)
{
	uint8_t length=1;
	uint8_t helpnumber;
	uint8_t i=0;
	if (number<0)
	{
		FIFO_Put(&USART_PC_TX_Buffer,(char)('-'));
		number*=-1;
	}
	helpnumber=number;
	while (helpnumber>9)
	{
		helpnumber/=10;
		length++;
	}
	helpnumber=1;
	for (i=0;i<length-1;i++) helpnumber*=10;
	for (i=0;i<length;i++)
	{
		FIFO_Put(&USART_PC_TX_Buffer,(char)(number/helpnumber+48));
		number%=helpnumber;
		helpnumber/=10;
	}
	USART_ITConfig(USART_PC,USART_IT_TXE,ENABLE);
}
void usartSendInt16(int16_t number)
{
	uint8_t length=1;
	uint16_t helpnumber;
	uint8_t i=0;
	if (number<0)
	{
		FIFO_Put(&USART_PC_TX_Buffer,(char)('-'));
		number*=-1;
	}
	helpnumber=number;
	while (helpnumber>9)
	{
		helpnumber/=10;
		length++;
	}
	helpnumber=1;
	for (i=0;i<length-1;i++) helpnumber*=10;
	for (i=0;i<length;i++)
	{
		FIFO_Put(&USART_PC_TX_Buffer,(char)(number/helpnumber+48));
		number%=helpnumber;
		helpnumber/=10;
	}
	USART_ITConfig(USART_PC,USART_IT_TXE,ENABLE);
}
void usartSendFloat(float number)
{
	uint8_t length=1;
	uint8_t i=0;
	int helpnumber=(int)number;
	int number2;
	while (helpnumber>9)
	{
		helpnumber/=10;
		length++;
	}
	length+=3;
	number2=(int)(number*=100);
	helpnumber=1;
	for (i=0;i<length-2;i++) helpnumber*=10;
	for (i=0;i<length-3;i++)
	{
		FIFO_Put(&USART_PC_TX_Buffer,(char)(number2/helpnumber+48));
		number2%=helpnumber;
		helpnumber/=10;
	}
	FIFO_Put(&USART_PC_TX_Buffer,(char)(','));
	FIFO_Put(&USART_PC_TX_Buffer,(char)((number2/10)+48));
	FIFO_Put(&USART_PC_TX_Buffer,(char)((number2%10)+48));
	USART_ITConfig(USART_PC,USART_IT_TXE,ENABLE);
}

void USART1_IRQHandler(void)
{
	uint16_t tmp;
	if(USART_GetITStatus(USART_PC,USART_IT_RXNE) != RESET)
    {
		USART_ClearITPendingBit(USART_PC,USART_IT_RXNE);
	    tmp=USART_ReceiveData(USART_PC);
	    FIFO_Put(&USART_PC_RX_Buffer,(uint8_t)(tmp%256));
	    USART_SendData(USART_PC,tmp);
    }
	if(USART_GetITStatus(USART_PC,USART_IT_TXE) != RESET)
    {
		USART_ClearITPendingBit(USART_PC,USART_IT_TXE);
	    if(FIFO_IsEmpty(&USART_PC_TX_Buffer)) USART_ITConfig(USART_PC,USART_IT_TXE,DISABLE);
	    else USART_SendData(USART_PC,(uint16_t)FIFO_Pop(&USART_PC_TX_Buffer));
    }
	USART_ClearITPendingBit(USART_PC,USART_IT_RXNE);
}
