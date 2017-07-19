#include "AX12A_library.h"

volatile uint8_t uart_message_in[110];
volatile uint8_t uart_message_out[110];
volatile uint32_t read_bytes_no = 0;

/**
 * 	@brief		Wait flag, possiblities:
 * 				0 - wait
 * 				1 - success
 * 				2 - timeout
 */
volatile uint8_t uart_wait_flag = 0;

//========================================================================
//================== DEFINITIONS OF PRIVATE FUNCTIONS ====================
//========================================================================

BulletStatus waitForReturnPacket(uint8_t output_bytes_number, uint8_t input_bytes_number)
{
	DMA1_Channel7->CMAR = (uint32_t)uart_message_out;				// Set output buffer address
	DMA1_Channel7->CNDTR = output_bytes_number;						// No of bytes to send
	read_bytes_no = input_bytes_number;								// No of bytes to read
	USART2->SR &= ~USART_FLAG_TC;									// Clear USART TC flag
	DMA1_Channel7->CCR |= DMA_CCR7_EN;								// Enable CH7

	TIM_SV->CNT = 0;
	TIM_SV->SR &= ~TIM_SR_UIF;
	TIM_SV->CR1 |= TIM_CR1_CEN;

    uart_wait_flag = 0;												// Clear the flag
	while(!uart_wait_flag);											// Wait for reciveing the message

	TIM_SV->SR &= ~TIM_SR_UIF;
	TIM_SV->CR1 &= ~TIM_CR1_CEN;

	if (uart_wait_flag == 2) return BULLET_SUCCESS;					// Return success
	else if (uart_wait_flag == 3) return BULLET_ERROR_TIMEOUT;		// Timeout

	return BULLET_ERROR_UNKNOWN;
}
BulletStatus waitForSendPacket(uint8_t output_bytes_number)
{
	DMA1_Channel7->CMAR = (uint32_t)uart_message_out;				// Set output buffer address
	DMA1_Channel7->CNDTR = output_bytes_number;						// No of bytes to send
	read_bytes_no = 0;												// No of bytes to read
	USART2->SR &= ~USART_FLAG_TC;									// Clear USART TC flag
	DMA1_Channel7->CCR |= DMA_CCR7_EN;								// Enable CH7

    uart_wait_flag = 0;												// Clear the flag
	while(!uart_wait_flag);											// Wait for reciveing the message

    if (uart_wait_flag == 1)									// Got timeout
    	return BULLET_SUCCESS;									// SUCCESS

    return BULLET_ERROR_UNKNOWN;
}

//========================================================================
//========================== PUBLIC CONFIG ===============================
//========================================================================

BulletStatus AX12A_Config(void)
{
	//Wlaczenie zegarow USART, GPIOA, AFIO, TIM2
	RCC_APB2PeriphClockCmd(USART_SV_PORT_CLOCK | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(USART_SV_CLOCK | TIM_SV_CLOCK, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	//Definicja struktury GPIO
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = USART_SV_PIN_TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART_SV_PORT, &GPIO_InitStructure);

	//Definicja struktury NVIC
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	//Ustaw przerwanie od TIM2
	NVIC_InitStructure.NVIC_IRQChannel = TIM_SV_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Definicja DMA
	DMA_DeInit(DMA1_Channel6);
	DMA_InitTypeDef DMA_InitStructure_RX;
	DMA_StructInit(&DMA_InitStructure_RX);
	DMA_InitStructure_RX.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR); 		//Address of peripheral the DMA must map to
	DMA_InitStructure_RX.DMA_MemoryBaseAddr = (uint32_t)(uart_message_in); 		//Variable to which received data will be stored
	DMA_InitStructure_RX.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure_RX.DMA_BufferSize = 12;
	DMA_InitStructure_RX.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure_RX.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure_RX.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure_RX.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure_RX.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure_RX.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure_RX.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel6, &DMA_InitStructure_RX);
	DMA1_Channel6->CCR &= ~(DMA_CCR6_EN);												// Disable DMA1 CH6

	// Definicja DMA
	DMA_DeInit(DMA1_Channel7);
	DMA_InitTypeDef DMA_InitStructure_TX;
	DMA_StructInit(&DMA_InitStructure_TX);
	DMA_InitStructure_TX.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR); 			//Address of peripheral the DMA must map to
	DMA_InitStructure_TX.DMA_MemoryBaseAddr = (uint32_t)(uart_message_out); 		//Variable to which received data will be stored
	DMA_InitStructure_TX.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure_TX.DMA_BufferSize = 0;
	DMA_InitStructure_TX.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure_TX.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure_TX.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure_TX.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure_TX.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure_TX.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure_TX.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel7, &DMA_InitStructure_TX);
	DMA1_Channel7->CCR &= ~(DMA_CCR7_EN);

	//DEFINICJA STRUKTURY USART
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = BAUD_RATE;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART_SV,&USART_InitStructure);
	USART_HalfDuplexCmd(USART_SV, ENABLE);			// Half duplex



	DMA1_Channel6->CCR |= DMA_CCR6_TCIE;
	DMA1_Channel7->CCR |= DMA_CCR7_TCIE;

	USART2->CR1 |= USART_CR1_UE;

	DMA1_Channel6->CCR &= ~DMA_CCR6_EN;
	DMA1_Channel7->CCR &= ~DMA_CCR7_EN;
	USART2->CR3 |= (USART_CR3_DMAT | USART_CR3_DMAR);



	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = 7200-1;							// 10kHz
	TIM_TimeBaseStructure.TIM_Period = 10ul;								// 1ms
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM_SV, &TIM_TimeBaseStructure);

	TIM_SV->SR &= ~TIM_SR_UIF;
	TIM_SV->DIER |= TIM_DIER_UIE;

	// Return success
	return BULLET_SUCCESS;
}

//========================================================================
//============================= HANDLERS =================================
//========================================================================

void DMA1_Channel6_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_TCIF6)						// If DMA CH6 TC interrutpt
	{
		DMA1->IFCR |= DMA_IFCR_CGIF6;				// Clear All flags
		DMA1_Channel6->CCR &= ~(DMA_CCR6_EN);			// Disable DMA1 CH6
		uart_wait_flag = 2;								// Set the wait flag value
	}
}
void DMA1_Channel7_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_TCIF7)
	{
		while (!(USART2->SR & USART_SR_TC));						// Wait until transmition is completed
		DMA1->IFCR |= DMA_IFCR_CGIF7;							// Clear All flags for CH7
		DMA1_Channel7->CCR &= ~DMA_CCR7_EN;							// Disable CH7

		if (read_bytes_no)
		{
			DMA1_Channel6->CMAR = (uint32_t)uart_message_in;		// Set input buffer address
			DMA1_Channel6->CNDTR = read_bytes_no;					// No of bytes to read
			DMA1_Channel6->CCR |= DMA_CCR6_EN;						// Enable CH6
		} else uart_wait_flag = 1;
	}
}
void TIM2_IRQHandler(void)
{
	//Przerwanie od przepelnienia TIM2
	if (TIM_GetITStatus(TIM_SV, TIM_IT_Update) != RESET)
	{
		TIM_SV->SR &= ~(TIM_SR_UIF);

	  	DMA1->IFCR |= DMA_IFCR_CGIF6;							// Clear All flags
	  	DMA1_Channel6->CCR &= ~(DMA_CCR6_EN);						// Disable DMA1 CH6

	  	DMA1->IFCR |= DMA_IFCR_CGIF7;							// Clear All flags
	  	DMA1_Channel7->CCR &= ~(DMA_CCR7_EN);						// Disable DMA1 CH7

	    uart_wait_flag = 3;											// Set the wait flag value
  }
}

//========================================================================
//=========================== PING FUNCTIONS =============================
//========================================================================

BulletStatus  AX12A_Ping(uint8_t servo_id, uint8_t *value)
{
	// Fill the output buffer data
	uart_message_out[0] = 0xFF;
	uart_message_out[1] = 0xFF;
	uart_message_out[2] = servo_id;
	uart_message_out[3] = 0x02;
	uart_message_out[4] = INST_PING;
	uart_message_out[5] = ~(servo_id+0x2+INST_PING);

	BulletStatus stat = waitForReturnPacket(6, 7);					// Wait for return message
	if (stat == BULLET_SUCCESS)										// If it succeed
		*value=uart_message_in[read_bytes_no-2];					// Set value
	return stat;													// Return stat
}

//========================================================================
//============================ EEPROM READ ===============================
//========================================================================

BulletStatus AX12A_ReadModelNumber(uint8_t servo_id, uint16_t *value)
{
	return AX12A_ReadWord(servo_id, P_MODEL_NUMBER_L, (uint8_t*)value, 1);
}
BulletStatus AX12A_ReadFirmwareVersion(uint8_t servo_id, uint8_t *value)
{
	return AX12A_ReadByte(servo_id, P_VERSION, value);
}
BulletStatus AX12A_ReadReturnDelayTime(uint8_t servo_number, uint8_t *value)
{
	return AX12A_ReadByte(servo_number, P_RETURN_DELAY_TIME, value);
}
BulletStatus AX12A_ReadLowerAngleLimit(uint8_t servo_number, uint16_t *value)
{
	return AX12A_ReadWord(servo_number, P_CW_ANGLE_LIMIT_L, (uint8_t*)value, 1);
}
BulletStatus AX12A_ReadUpperAngleLimit(uint8_t servo_number, uint16_t *value)
{
	return AX12A_ReadWord(servo_number, P_CCW_ANGLE_LIMIT_L, (uint8_t*)value, 1);
}
BulletStatus AX12A_ReadTemperatureLimit(uint8_t servo_number, uint8_t *value)
{
	return AX12A_ReadByte(servo_number, P_LIMIT_TEMPERATURE, value);
}
BulletStatus AX12A_ReadLowerVolgageLimit(uint8_t servo_number, uint8_t *value)
{
	return AX12A_ReadByte(servo_number, P_DOWN_LIMIT_VOLTAGE, value);
}
BulletStatus AX12A_ReadUpperVolgageLimit(uint8_t servo_number, uint8_t *value)
{
	return AX12A_ReadByte(servo_number, P_UP_LIMIT_VOLTAGE, value);
}
BulletStatus AX12A_ReadMaxTorqueEEPROM(uint8_t servo_number, uint16_t *value)
{
	return AX12A_ReadWord(servo_number, P_MAX_TORQUE_L, (uint8_t*)value, 1);
}
BulletStatus AX12A_ReadStatusReturnLevel(uint8_t servo_number, uint8_t *value)
{
	return AX12A_ReadByte(servo_number, P_RETURN_LEVEL, value);
}
BulletStatus AX12A_ReadAlarmLED(uint8_t servo_number, uint8_t *value)
{
	return AX12A_ReadByte(servo_number, P_ALARM_LED, value);
}
BulletStatus AX12A_ReadAlarmShutdown(uint8_t servo_number, uint8_t *value)
{
	return AX12A_ReadByte(servo_number, P_ALARM_SHUTDOWN, value);
}

//========================================================================
//============================ EEPROM WRITE ==============================
//========================================================================

BulletStatus  AX12A_WriteID(uint8_t new_ID)
{
	uart_message_out[0] = 0xFF;
	uart_message_out[1] = 0xFF;
	uart_message_out[2] = BROADCASTING_ID;
	uart_message_out[3] = 0x04;
	uart_message_out[4] = INST_WRITE;
	uart_message_out[5] = P_ID;
	uart_message_out[6] = new_ID;
	uart_message_out[7] = (uint8_t)~(BROADCASTING_ID+0x4+INST_WRITE+P_ID+new_ID);

	return waitForSendPacket(8);
}
BulletStatus  AX12A_WriteBaudRate(uint8_t new_baud_rate)
{
	uart_message_out[0] = 0xFF;
	uart_message_out[1] = 0xFF;
	uart_message_out[2] = BROADCASTING_ID;
	uart_message_out[3] = 0x04;
	uart_message_out[4] = INST_WRITE;
	uart_message_out[5] = P_BAUD_RATE;
	uart_message_out[6] = new_baud_rate;
	uart_message_out[7] = (uint8_t)~(BROADCASTING_ID+0x4+INST_WRITE+P_BAUD_RATE+new_baud_rate);

	return waitForSendPacket(8);
}
BulletStatus  AX12A_WriteReturnRelayTime(uint8_t servo_number, uint8_t new_return_delay_time)
{
	return AX12A_WriteByte(servo_number, P_RETURN_DELAY_TIME, new_return_delay_time);
}
BulletStatus  AX12A_WriteLowerAngleLimit(uint8_t servo_number, uint16_t lower_angle_limit)
{
	return AX12A_WriteWord(servo_number, P_CW_ANGLE_LIMIT_L, (uint8_t*)&lower_angle_limit, 1);
}
BulletStatus  AX12A_WriteUpperAngleLimit(uint8_t servo_number, uint16_t upper_angle_limit)
{
	return AX12A_WriteWord(servo_number, P_CCW_ANGLE_LIMIT_L, (uint8_t*)&upper_angle_limit, 1);
}
BulletStatus  AX12A_WriteTemperatureLimit(uint8_t servo_number, uint8_t temperature_limit)
{
	return AX12A_WriteByte(servo_number, P_LIMIT_TEMPERATURE, temperature_limit);
}
BulletStatus  AX12A_WriteLowerVolgageLimit(uint8_t servo_number, uint8_t lower_voltage_limit)
{
	return AX12A_WriteByte(servo_number, P_DOWN_LIMIT_VOLTAGE, lower_voltage_limit);
}
BulletStatus  AX12A_WriteUpperVolgageLimit(uint8_t servo_number, uint8_t upper_voltage_limit)
{
	return AX12A_WriteByte(servo_number, P_UP_LIMIT_VOLTAGE, upper_voltage_limit);
}
BulletStatus  AX12A_WriteMaxTorqueEEPROM(uint8_t servo_number, uint16_t max_torque)
{
	return AX12A_WriteWord(servo_number, P_MAX_TORQUE_L, (uint8_t*)&max_torque, 1);
}
BulletStatus  AX12A_WriteStatusReturnLevel(uint8_t servo_number, uint8_t status)
{
	return AX12A_WriteByte(servo_number, P_RETURN_LEVEL, status);
}
BulletStatus  AX12A_WriteAlarmLED(uint8_t servo_number, uint8_t value)
{
	return AX12A_WriteByte(servo_number, P_ALARM_LED, value);
}
BulletStatus  AX12A_WriteAlarmShutdown(uint8_t servo_number, uint8_t value)
{
	return AX12A_WriteByte(servo_number, P_ALARM_SHUTDOWN, value);
}

//========================================================================
//=============================== RAM READ ===============================
//========================================================================

BulletStatus AX12A_ReadTorqueEnable(uint8_t servo_number, uint8_t *value)
{
	return AX12A_ReadByte(servo_number, P_TORQUE_ENABLE, value);
}
BulletStatus AX12A_ReadLED(uint8_t servo_number, uint8_t *value)
{
	return AX12A_ReadByte(servo_number, P_LED, value);
}
BulletStatus AX12A_ReadLowerComplianceMargin(uint8_t servo_number, uint8_t *value)
{
	return AX12A_ReadByte(servo_number, P_CW_COMPLIANCE_MARGIN, value);
}
BulletStatus AX12A_ReadUpperComplianceMargin(uint8_t servo_number, uint8_t *value)
{
	return AX12A_ReadByte(servo_number, P_CCW_COMPLIANCE_MARGIN, value);
}
BulletStatus AX12A_ReadLowerComplianceSlope(uint8_t servo_number, uint8_t *value)
{
	return AX12A_ReadByte(servo_number, P_CW_COMPLIANCE_SLOPE, value);
}
BulletStatus AX12A_ReadUpperComplianceSlope(uint8_t servo_number, uint8_t *value)
{
	return AX12A_ReadByte(servo_number, P_CCW_COMPLIANCE_SLOPE, value);
}
BulletStatus AX12A_ReadGoalPosition(uint8_t servo_number, uint16_t *value)
{
	return AX12A_ReadWord(servo_number, P_GOAL_POSITION_L, (uint8_t*)value, 1);
}
BulletStatus AX12A_ReadMovingSpeed(uint8_t servo_number, uint16_t *value)
{
	return AX12A_ReadWord(servo_number, P_GOAL_SPEED_L, (uint8_t*)value, 1);
}
BulletStatus AX12A_ReadMaxTorqueRAM(uint8_t servo_number, uint16_t *value)
{
	return AX12A_ReadWord(servo_number, P_TORQUE_LIMIT_L, (uint8_t*)value, 1);
}
BulletStatus AX12A_ReadPresentPosition(uint8_t servo_number, uint16_t *value)
{
	return AX12A_ReadWord(servo_number, P_PRESENT_POSITION_L, (uint8_t*)value, 1);
}
BulletStatus AX12A_ReadPresentMovingSpeed(uint8_t servo_number, uint16_t *value)
{
	return AX12A_ReadWord(servo_number, P_PRESENT_SPEED_L, (uint8_t*)value, 1);
}
BulletStatus AX12A_ReadPresentLoad(uint8_t servo_number, uint16_t *value)
{
	return AX12A_ReadWord(servo_number, P_PRESENT_LOAD_L, (uint8_t*)(uint8_t*)value, 1);
}
BulletStatus AX12A_ReadPresentVoltage(uint8_t servo_number, uint8_t *value)
{
	return AX12A_ReadByte(servo_number, P_PRESENT_VOLTAGE, value);
}
BulletStatus AX12A_ReadPresentTemperature(uint8_t servo_number, uint8_t *value)
{
	return AX12A_ReadByte(servo_number, P_PRESENT_TEMPERATURE, value);
}
BulletStatus AX12A_ReadRegisteredInstruction(uint8_t servo_number, uint8_t *value)
{
	return AX12A_ReadByte(servo_number, P_REGISTERED_INSTRUCTION, value);
}
BulletStatus AX12A_ReadMoving(uint8_t servo_number, uint8_t *value)
{
	return AX12A_ReadByte(servo_number, P_MOVING, value);
}
BulletStatus AX12A_ReadLock(uint8_t servo_number, uint8_t *value)
{
	return AX12A_ReadByte(servo_number, P_LOCK, value);
}
BulletStatus AX12A_ReadPunch(uint8_t servo_number, uint16_t *value)
{
	return AX12A_ReadWord(servo_number, P_PUNCH_L, (uint8_t*)value, 1);
}

//========================================================================
//============================== RAM WRITE ===============================
//========================================================================

BulletStatus  AX12A_WriteTorqueEnable(uint8_t servo_number, uint8_t value)
{
	return AX12A_WriteByte(servo_number, P_TORQUE_ENABLE, value);
}
BulletStatus  AX12A_WriteLED(uint8_t servo_number, uint8_t state)
{
	return AX12A_WriteByte(servo_number, P_LED, state);
}
BulletStatus  AX12A_WriteLowerComplianceMargin(uint8_t servo_number, uint8_t value)
{
	return AX12A_WriteByte(servo_number, P_CW_COMPLIANCE_MARGIN, value);
}
BulletStatus  AX12A_WriteUpperComplianceMargin(uint8_t servo_number, uint8_t value)
{
	return AX12A_WriteByte(servo_number, P_CCW_COMPLIANCE_MARGIN, value);
}
BulletStatus  AX12A_WriteLowerComplianceSlope(uint8_t servo_number, uint8_t value)
{
	return AX12A_WriteByte(servo_number, P_CCW_COMPLIANCE_SLOPE, value);
}
BulletStatus  AX12A_WriteUpperComplianceSlope(uint8_t servo_number, uint8_t value)
{
	return AX12A_WriteByte(servo_number, P_CCW_COMPLIANCE_SLOPE, value);
}
BulletStatus  AX12A_WriteGoalPosition(uint8_t servo_number, uint16_t goal_position)
{
	return AX12A_WriteWord(servo_number, P_GOAL_POSITION_L, (uint8_t*)&goal_position, 1);
}
BulletStatus  AX12A_WriteMovingSpeed(uint8_t servo_number, uint16_t moving_speed)
{
	return AX12A_WriteWord(servo_number, P_GOAL_SPEED_L, (uint8_t*)&moving_speed, 1);
}
BulletStatus  AX12A_WriteMaxTorqueRAM(uint8_t servo_number, uint16_t max_torque)
{
	return AX12A_WriteWord(servo_number, P_TORQUE_LIMIT_L, (uint8_t*)&max_torque, 1);
}
BulletStatus  AX12A_WriteLock(uint8_t servo_number, uint8_t value)
{
	return AX12A_WriteByte(servo_number, P_LOCK, value);
}
BulletStatus  AX12A_WritePunch(uint8_t servo_number, uint16_t punch)
{
	return AX12A_WriteWord(servo_number, P_PUNCH_L, (uint8_t*)&punch, 1);
}

// =======================================================================
// ============================ MASS METHOD ==============================
// =======================================================================

BulletStatus AX12A_PingAll(uint8_t *value)
{
	int leg, joint;
	for (leg=1; leg<=6; ++leg)
		for (joint=1; joint<=3; ++joint)
		{
			BulletStatus stat = AX12A_Ping(leg*10+joint, value);
			if (stat != BULLET_SUCCESS) return stat;
		}
	return BULLET_SUCCESS;
}
BulletStatus AX12A_ReadAllPosition(uint8_t *values, uint8_t little_endian)
{
	int leg, joint;
	for (leg=1; leg<=6; ++leg)
		for (joint=1; joint<=3; ++joint)
		{
			BulletStatus stat = AX12A_ReadWord(leg*10+joint, P_PRESENT_POSITION_L, &values[2*((leg-1)*3 + (joint-1))], little_endian);
			if (stat != BULLET_SUCCESS) return stat;
		}
	return BULLET_SUCCESS;
}
BulletStatus AX12A_ReadAllPositionSpeedLoad(uint8_t *values, uint8_t little_endian)
{
	int leg, joint;
	for (leg=1; leg<=6; ++leg)
		for (joint=1; joint<=3; ++joint)
		{
			BulletStatus stat = AX12A_ReadBytes(leg*10+joint, P_PRESENT_POSITION_L, (uint8_t*)(&values[6*((leg-1)*3 + (joint-1))]), 6, little_endian);
			if (stat != BULLET_SUCCESS) return stat;
		}
	return BULLET_SUCCESS;
}
BulletStatus  AX12A_WriteAllGoalPosition(uint8_t *values, uint8_t little_endian)
{
	uint8_t sum=0;																	// Sum for CRC calculation
	uart_message_out[0] = 0xFF;
	uart_message_out[1] = 0xFF;
	uart_message_out[2] = BROADCASTING_ID;				sum+=BROADCASTING_ID;		// Broadcasting id
	uart_message_out[3] = 58;							sum+=58;					// Packet length = 18*2+4 bytes
	uart_message_out[4] = INST_SYNC_WRITE;				sum+=INST_SYNC_WRITE;		// Sync write instr
	uart_message_out[5] = P_GOAL_POSITION_L;			sum+=P_GOAL_POSITION_L;		// Starting address
	uart_message_out[6] = 0x02;							sum+=0x02;					// How many bytes to one serv

	uint8_t leg_number;
	for (leg_number=0; leg_number<6; ++leg_number)
	{
		uint8_t servo_id = 10*(leg_number+1);
		uint8_t i;
		for (i=0;i<3;++i)
		{
			++servo_id;
			uint8_t byte_no = 7 + leg_number*9 + i*3;
			uint8_t index = 2*(3*leg_number + i);

			uart_message_out[byte_no+0] = servo_id;				sum+=servo_id;
			if (little_endian)
			{
				uart_message_out[byte_no+1] = values[index+0];		sum+=values[index+0];
				uart_message_out[byte_no+2] = values[index+1];		sum+=values[index+1];
			} else
			{
				uart_message_out[byte_no+2] = values[index+1];		sum+=values[index+1];
				uart_message_out[byte_no+1] = values[index+0];		sum+=values[index+0];
			}
		}
	}
	uart_message_out[61] = ~(sum);
	return waitForSendPacket(62);
}
BulletStatus  AX12A_WriteAllTorqueEnable(uint8_t value)
{
	uint8_t sum=0;																	// Sum for CRC calculation
	uart_message_out[0] = 0xFF;
	uart_message_out[1] = 0xFF;
	uart_message_out[2] = BROADCASTING_ID;				sum+=BROADCASTING_ID;		// Broadcasting id
	uart_message_out[3] = 40;							sum+=40;					// Packet length = 18*2+4 bytes
	uart_message_out[4] = INST_SYNC_WRITE;				sum+=INST_SYNC_WRITE;		// Sync write instr
	uart_message_out[5] = P_TORQUE_ENABLE;				sum+=P_TORQUE_ENABLE;		// Starting address
	uart_message_out[6] = 0x01;							sum+=0x01;					// How many bytes to one serv

	uint8_t leg_number;
	for (leg_number=0; leg_number<6; ++leg_number)
	{
		uint8_t servo_id = 10*(leg_number+1);
		uint8_t i;
		for (i=0;i<3;++i)
		{
			++servo_id;
			uint8_t byte_no = 7 + leg_number*6 + i*2;
			uart_message_out[byte_no+0] = servo_id;		sum+=servo_id;
			uart_message_out[byte_no+1] = value;		sum+=value;
		}
	}
	uart_message_out[43] = ~(sum);													// CRC

	return waitForSendPacket(44);									// Return succeed
}

//========================================================================
//=========================== HELP FUNCTIONS =============================
//========================================================================

BulletStatus  AX12A_ReadByte (uint8_t servo_id, uint8_t control_table_address, uint8_t *value)
{
	// Fill the output buffer data
	uart_message_out[0] = 0xFF;
	uart_message_out[1] = 0xFF;
	uart_message_out[2] = servo_id;
	uart_message_out[3] = 0x04;
	uart_message_out[4] = INST_READ;
	uart_message_out[5] = control_table_address;
	uart_message_out[6] = 0x01;
	uart_message_out[7] = ~(servo_id+0x04+INST_READ+control_table_address+0x01);

	BulletStatus stat = waitForReturnPacket(8, 8);					// Wait for return message
	if (stat == BULLET_SUCCESS)										// If it succeed
		*value=uart_message_in[read_bytes_no-2];					// Set value
	return stat;
}
BulletStatus  AX12A_ReadWord(uint8_t servo_id, uint8_t control_table_address, uint8_t *value, uint8_t little_endian)
{
	// Fill the output buffer data
	uart_message_out[0] = 0xFF;
	uart_message_out[1] = 0xFF;
	uart_message_out[2] = servo_id;
	uart_message_out[3] = 0x04;
	uart_message_out[4] = INST_READ;
	uart_message_out[5] = control_table_address;
	uart_message_out[6] = 0x02;
	uart_message_out[7] = ~(servo_id+0x04+INST_READ+control_table_address+0x02);

	BulletStatus stat = waitForReturnPacket(8, 9);					// Wait for return message
	if (stat == BULLET_SUCCESS)										// If it succeed
	{
		if(little_endian)
		{
			value[0] = uart_message_in[read_bytes_no-3];			// Set value
			value[1] = uart_message_in[read_bytes_no-2];			// Set value
		} else
		{
			value[0] = uart_message_in[read_bytes_no-2];			// Set value
			value[1] = uart_message_in[read_bytes_no-3];			// Set value
		}
	}
	return stat;
}
BulletStatus  AX12A_WriteByte(uint8_t servo_id, uint8_t control_table_address, uint8_t value)
{
	// Fill the output buffer data
	uart_message_out[0] = 0xFF;
	uart_message_out[1] = 0xFF;
	uart_message_out[2] = servo_id;
	uart_message_out[3] = 0x04;
	uart_message_out[4] = INST_WRITE;
	uart_message_out[5] = control_table_address;
	uart_message_out[6] = value;
	uart_message_out[7] = ~(servo_id+0x04+INST_WRITE+control_table_address+value);

	return waitForSendPacket(8);									// Return succeed
}
BulletStatus  AX12A_WriteWord(uint8_t servo_id, uint8_t control_table_address, uint8_t *values, uint8_t little_endian)
{
	// Fill the output buffer data
	uart_message_out[0] = 0xFF;
	uart_message_out[1] = 0xFF;
	uart_message_out[2] = servo_id;
	uart_message_out[3] = 0x05;
	uart_message_out[4] = INST_WRITE;
	uart_message_out[5] = control_table_address;
	if(little_endian)
	{
		uart_message_out[6] = values[0];							// Set value
		uart_message_out[7] = values[1];							// Set value
	} else
	{
		uart_message_out[6] = values[1];							// Set value
		uart_message_out[7] = values[0];							// Set value
	}
	uart_message_out[8] = ~(servo_id+0x5+INST_WRITE+control_table_address+values[0]+values[1]);

	return waitForSendPacket(9);
}
BulletStatus  AX12A_ReadBytes(uint8_t servo_id, uint8_t control_table_address, uint8_t bytes[], uint8_t size, uint8_t little_endian)
{
	// Fill the output buffer data
	uart_message_out[0] = 0xFF;
	uart_message_out[1] = 0xFF;
	uart_message_out[2] = servo_id;
	uart_message_out[3] = 0x04;
	uart_message_out[4] = INST_READ;
	uart_message_out[5] = control_table_address;
	uart_message_out[6] = size;
	uart_message_out[7] = ~(servo_id+0x04+INST_READ+control_table_address+size);

	BulletStatus stat = waitForReturnPacket(8, 1+6+size);			// Wait for return message
	if (stat == BULLET_SUCCESS)										// If it succeed
	{
		int i;
		for (i=0; i<size/2; i++)
		{
			if(little_endian)
			{
				bytes[2*i+0] = uart_message_in[6+2*i+0];			// Set value
				bytes[2*i+1] = uart_message_in[6+2*i+1];			// Set value
			} else
			{
				bytes[2*i+0] = uart_message_in[6+2*i+1];			// Set value
				bytes[2*i+1] = uart_message_in[6+2*i+0];			// Set value
			}
		}
		if (2*i<size) bytes[2*i+0] = uart_message_in[6+2*i+0];
	}
    return stat;
}
