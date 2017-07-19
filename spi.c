#include "spi.h"
#include "string.h"

volatile uint8_t spi_message_in[39];
volatile uint8_t spi_message_out[110];
volatile uint8_t main_process_spi = 0;

// ======================================================================================
// ================================ PUBLIC FUNCTIONS ====================================
// ======================================================================================

void SPI_Config(void)
{
	//Wlaczenie zegarow dla wszystkich peryferiow
	RCC_APB2PeriphClockCmd(SPI_CLK | SPI_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(SPI_TIM_CLOCK, ENABLE);

	//Definicja struktury GPIO
	GPIO_InitTypeDef  GPIO_InitStructure;

	//Skonfiguruj piny SCK oraz MOSI jako wyjscie Push-Pull
	GPIO_InitStructure.GPIO_Pin = SPI_PIN_SCK | SPI_PIN_MOSI;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(SPI_PORT, &GPIO_InitStructure);

	//Skonfiguruj pin MISO jako wejscie plywajace
	GPIO_InitStructure.GPIO_Pin = SPI_PIN_MISO ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPI_PORT, &GPIO_InitStructure);

	//Ustaw przerwanie od SPI
	NVIC_InitTypeDef SPI_NVIC;
	SPI_NVIC.NVIC_IRQChannel = SPI_IRQn;
	SPI_NVIC.NVIC_IRQChannelPreemptionPriority = 0;
	SPI_NVIC.NVIC_IRQChannelSubPriority = 0;
	SPI_NVIC.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&SPI_NVIC);

	//Ustaw przerwanie od TIM2
	SPI_NVIC.NVIC_IRQChannel = TIM3_IRQn;
	SPI_NVIC.NVIC_IRQChannelPreemptionPriority = 0;
	SPI_NVIC.NVIC_IRQChannelSubPriority = 1;
	SPI_NVIC.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&SPI_NVIC);

	SPI_NVIC.NVIC_IRQChannel = DMA1_Channel2_IRQn;
	SPI_NVIC.NVIC_IRQChannelPreemptionPriority = 0;
	SPI_NVIC.NVIC_IRQChannelSubPriority = 2;
	SPI_NVIC.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&SPI_NVIC);

	SPI_NVIC.NVIC_IRQChannel = DMA1_Channel3_IRQn;
	SPI_NVIC.NVIC_IRQChannelPreemptionPriority = 0;
	SPI_NVIC.NVIC_IRQChannelSubPriority = 3;
	SPI_NVIC.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&SPI_NVIC);

	//Definicja struktury SPI
	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI, &SPI_InitStructure);
	SPI_Cmd(SPI, ENABLE);

	// Definicja DMA
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Channel2);
	DMA_InitTypeDef DMA_InitStructure_RX;
	DMA_StructInit(&DMA_InitStructure_RX);
	DMA_InitStructure_RX.DMA_PeripheralBaseAddr = (uint32_t)(&SPI->DR); 			//Address of peripheral the DMA must map to
	DMA_InitStructure_RX.DMA_MemoryBaseAddr = (uint32_t)(spi_message_in); 		//Variable to which received data will be stored
	DMA_InitStructure_RX.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure_RX.DMA_BufferSize = 0;
	DMA_InitStructure_RX.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure_RX.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure_RX.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure_RX.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure_RX.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure_RX.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure_RX.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel2, &DMA_InitStructure_RX);
	DMA1_Channel2->CCR &= ~(DMA_CCR2_EN);												// Disable DMA1 CH2

	// Definicja DMA
	DMA_DeInit(DMA1_Channel3);
	DMA_InitTypeDef DMA_InitStructure_TX;
	DMA_StructInit(&DMA_InitStructure_TX);
	DMA_InitStructure_TX.DMA_PeripheralBaseAddr = (uint32_t)(&SPI->DR); 			//Address of peripheral the DMA must map to
	DMA_InitStructure_TX.DMA_MemoryBaseAddr = (uint32_t)(spi_message_out); 		//Variable to which received data will be stored
	DMA_InitStructure_TX.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure_TX.DMA_BufferSize = 0;
	DMA_InitStructure_TX.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure_TX.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure_TX.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure_TX.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure_TX.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure_TX.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure_TX.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure_TX);
	DMA1_Channel3->CCR &= ~(DMA_CCR3_EN);												// Disable DMA1 CH3

	DMA1_Channel2->CCR |= (DMA_CCR2_TCIE);												// Enable DMA1_CH2 TC interrupt
	DMA1_Channel3->CCR |= (DMA_CCR3_TCIE);												// Enable DMA1_CH3 TC interrupt
	SPI1->CR2 &= ~(SPI_CR2_TXEIE);														// Disable SPI RX interrupt
	SPI1->CR2 |= (SPI_CR2_RXNEIE);														// Enable SPI RX interrupt
	SPI1->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);									// Disbale DMAReq from SPI1 RX & TX


	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = 36000-1;							// 2kHz
	TIM_TimeBaseStructure.TIM_Period = 200;									// 100ms
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	SPI_TIM->SR &= ~TIM_SR_UIF;								//Reset update interrupt flag
	SPI_TIM->DIER |= TIM_DIER_UIE;							//Enable update interrupt
}

void DMA1_Channel2_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_TCIF2)						// If DMA CH2 TC interrutpt
	{
		DMA1->IFCR |= (DMA_IFCR_CGIF2);					// Clear All flags
		DMA1_Channel2->CCR &= ~(DMA_CCR2_EN);			// Disable DMA1 CH2
		SPI1->CR2 |= (SPI_CR2_RXNEIE);					// Enable SPI RX interrupt
		SPI1->CR2 &= ~(SPI_CR2_RXDMAEN);				// Disable DMAReq from SPI1 RX

		main_process_spi = 1;							// Process message in main loop
	}
}

void DMA1_Channel3_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_TCIF3)
	{
		DMA1->IFCR |= (DMA_IFCR_CGIF3);					// Clear All flags
		DMA1_Channel3->CCR &= ~(DMA_CCR3_EN);			// Disable DMA1 CH3
		SPI1->CR2 &= ~(SPI_CR2_TXDMAEN);				// Disbale DMAReq from SPI1 TX
	}
}

void TIM3_IRQHandler(void)
{
	//Przerwanie od przepelnienia TIM2
	if (TIM_GetITStatus(SPI_TIM, TIM_IT_Update) != RESET)
	{
		SPI_TIM->SR &= ~(TIM_SR_UIF);							// Reset update interrupt flag

	  	DMA1->IFCR |= DMA_IFCR_CGIF2;							// Clear All flags
	  	DMA1_Channel2->CCR &= ~(DMA_CCR2_EN);					// Disable DMA1 CH2
	  	SPI1->CR2 &= ~(SPI_CR2_RXDMAEN);						// Disable DMAReq from SPI1 RX
	  	SPI1->CR2 |= (SPI_CR2_RXNEIE);							// Enable SPI RX interrupt

	  	SPI_TIM->CR1 &= ~TIM_CR1_CEN;							//Disable counter
  }
}

void SPI1_IRQHandler(void)
{
	//Przerwanie od odbierania
	if (SPI_I2S_GetITStatus(SPI, SPI_I2S_IT_RXNE) != RESET)
	{
		uint16_t spi_no = SPI->DR;							// Get the number of bytes
		if (spi_no != 0)
		{
			DMA1_Channel2->CNDTR = spi_no;					// Set number of bytes
			DMA1_Channel2->CCR |= (DMA_CCR2_EN);			// Enable DMA1 CH2
			SPI1->CR2 |= (SPI_CR2_RXDMAEN);					// Enable DMAReq from SPI1 RX
			SPI1->CR2 &= ~(SPI_CR2_RXNEIE);					// Disable SPI RX interrupt

			SPI_TIM->CNT = 0;								//Reset TIM counter
			SPI_TIM->SR &= ~TIM_SR_UIF;						//Reset update interrupt flag
			SPI_TIM->CR1 |= TIM_CR1_CEN;					//Enable counter
		}
	}
}
