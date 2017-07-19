//=====================================================================================================
//============================================== HEADERS ==============================================
//=====================================================================================================

#include "bullet_firmware_core.h"
#include "AX12A_library.h"
#include "spi.h"
#include "string.h"

//=====================================================================================================
//========================================== GLOBAL VARIABLES =========================================
//=====================================================================================================

volatile uint8_t allow_next_loop = 0;

volatile BulletSTMMode stm_mode = BULLET_MODE_INITIALIZING;

volatile BulletStatus stm_status = BULLET_SUCCESS;

volatile uint16_t servos_goal_pos[18] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
										 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
										 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};

volatile uint16_t servos_pres_pos_speed_load[54] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
										   	   	    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
										   	   	    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
										   	   	    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
										   	   	    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
										   	   	    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
										   	   	    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
										   	   	    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};

//=====================================================================================================
//========================================== HELP FUNCTIONS ===========================================
//=====================================================================================================

void RCC_Config(void);
void SysTick_Handler();
void processSPIReadAll();
void processSPIWriteAll();
void processSPIChangeMode();
void processSPIPreoperationalMessage();
void processSPIOperationalMessage();

//=====================================================================================================
//=========================================== MAIN ROUTINE ============================================
//=====================================================================================================

int main(void)
{
	//=================================================================================================
	//================================== PERIPHERIALS CONFIGURATION ===================================
	//=================================================================================================

	RCC_Config();

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	#ifdef  VECT_TAB_RAM
		NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
	#else
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	#endif

	SPI_Config();
	AX12A_Config();

	//=================================================================================================
	//===================================== SYSTICK ENABLE 200Hz ======================================
	//=================================================================================================

	SysTick_Config(360000);
	while(allow_next_loop < 200);
	allow_next_loop = 0;

	//=================================================================================================
	//================================== COMMUNICATE WITH ALL SERVOS ==================================
	//=================================================================================================

	BulletStatus status_ping = AX12A_PingAll((uint8_t*)servos_pres_pos_speed_load);
	BulletStatus status_read = AX12A_ReadAllPositionSpeedLoad((uint8_t*)servos_pres_pos_speed_load, 1);
	if (status_ping != BULLET_SUCCESS || status_read != BULLET_SUCCESS) stm_mode = BULLET_MODE_STOPPED;
	else
	{
		AX12A_WriteAllTorqueEnable(0);
		int i=0;
		for (i=0; i<18; i++) servos_goal_pos[i] = servos_pres_pos_speed_load[3*i];
		stm_mode = BULLET_MODE_PREOPERATIONAL;
	}

	//=================================================================================================
	//======================================== INFINITE LOOP ==========================================
	//=================================================================================================

	while(1)
	{
		//=============================================================================================
		//================================== MODE PREOPERATIONAL ======================================
		//=============================================================================================

		if (stm_mode == BULLET_MODE_PREOPERATIONAL)
		{

			//=========================================================================================
			//============================== READ POS SPEED & LOAD ====================================
			//=========================================================================================

			AX12A_ReadAllPositionSpeedLoad((uint8_t*)servos_pres_pos_speed_load, 1);
			int i=0;
			for (i=0; i<18; i++) servos_goal_pos[i] = servos_pres_pos_speed_load[3*i];

			//=========================================================================================
			//============================== PROCESS RPI QUESTIONS ====================================
			//=========================================================================================

			if (main_process_spi)
			{
				processSPIPreoperationalMessage();
				main_process_spi=0;
			}
		}

		//=============================================================================================
		//==================================== MODE OPERATIONAL =======================================
		//=============================================================================================

		if (stm_mode == BULLET_MODE_OPERATIONAL)
		{
			AX12A_ReadAllPositionSpeedLoad((uint8_t*)servos_pres_pos_speed_load, 1);
			AX12A_WriteAllGoalPosition((uint8_t*)servos_goal_pos, 1);

			//=========================================================================================
			//============================== PROCESS RPI QUESTIONS ====================================
			//=========================================================================================

			if (main_process_spi)
			{
				processSPIOperationalMessage();
				main_process_spi = 0;
			}
		}

		//=============================================================================================
		//======================================= LOOP WAIT ===========================================
		//=============================================================================================

		while(!allow_next_loop);
		allow_next_loop = 0;
	}
}

//=====================================================================================================
//========================================== HELP FUNCTIONS ===========================================
//=====================================================================================================

void SysTick_Handler()
{
	++allow_next_loop;
}

void RCC_Config(void)
{
	ErrorStatus HSEStartUpStatus;

	// Reset ustawien RCC
	RCC_DeInit();

	// Wlacz HSE
	RCC_HSEConfig(RCC_HSE_ON);

	// Czekaj za HSE bedzie gotowy
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	if(HSEStartUpStatus == SUCCESS)
	{
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		// zwloka dla pamieci Flash
		FLASH_SetLatency(FLASH_Latency_2);

		// HCLK = SYSCLK
		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		// PCLK2 = HCLK
		RCC_PCLK2Config(RCC_HCLK_Div1);

		// PCLK1 = HCLK/2
		RCC_PCLK1Config(RCC_HCLK_Div2);

		// PLLCLK = 6MHz * 12 = 72 MHz
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_12);

		// Wlacz PLL
		RCC_PLLCmd(ENABLE);

		// Czekaj az PLL poprawnie sie uruchomi
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

		// PLL bedzie zrodlem sygnalu zegarowego
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		// Czekaj az PLL bedzie sygnalem zegarowym systemu
		while(RCC_GetSYSCLKSource() != 0x08);
	}
}

//=====================================================================================================
//=================================== PROCESS SPI PREOPERATIONAL MESSAGE ==============================
//=====================================================================================================

void processSPIReadAll()
{
	// Get instruction
	uint8_t command = spi_message_in[0];
	uint8_t instruction = spi_message_in[1];
	uint8_t i=0;

	if (instruction == 0x00)
	{
		// Get data
		uint8_t crc = spi_message_in[2];
		uint8_t sum=0;

		// CRC Error
		if (crc!=((uint8_t)(255-(command + instruction))))
		{
			spi_message_out[0] = BULLET_ERROR_SPI_CRC;
			for (i=0; i<108; ++i) spi_message_out[i+1] = 0x00;
			spi_message_out[109] = (uint8_t)(255-(BULLET_ERROR_SPI_CRC));
		} else
		{
			// Respond
			spi_message_out[0] = BULLET_SUCCESS;
			for (i=0; i<108; i++)
			{
				uint8_t ch = ((uint8_t*)servos_pres_pos_speed_load)[i];
				spi_message_out[i+1] = ch;
				sum += ch;
			}
			spi_message_out[109] = (uint8_t)(255-(sum));
		}
	} else
	{
		// Respond
		spi_message_out[0] = BULLET_ERROR_MODE_MISMATCH;
		for (i=0; i<108; ++i) spi_message_out[i+1] = 0x00;
		spi_message_out[109] = (uint8_t)(255-(BULLET_ERROR_MODE_MISMATCH));
	}
	DMA1_Channel3->CNDTR = 110;						// Number of data to send
	DMA1_Channel3->CCR |= (DMA_CCR3_EN);			// Enable DMA1 CH3
	SPI1->CR2 |= (SPI_CR2_TXDMAEN);					// Enable DMA TX Req
}
void processSPIWriteAll()
{
	// Get instruction
	uint8_t command = spi_message_in[0];
	uint8_t instruction = spi_message_in[1];

	if (instruction == 0x00)
	{
		// Get data
		uint8_t message[36];
		uint8_t i=0;
		uint8_t crc_in=command+instruction;
		for (i=0; i<36; ++i)
		{
			message[i] = spi_message_in[i+2];
			crc_in += message[i];
		}
		uint8_t crc_mes = spi_message_in[38];

		// CRC Error
		if (crc_mes != ((uint8_t)(255-(crc_in))))
		{
			spi_message_out[0] = BULLET_ERROR_SPI_CRC;
			spi_message_out[1] = (uint8_t)(255-(BULLET_ERROR_SPI_CRC));
		} else
		{
			// Respond
			spi_message_out[0] = BULLET_SUCCESS;
			spi_message_out[1] = (uint8_t)(255-(BULLET_SUCCESS));

			// Store data
			memcpy((void*)servos_goal_pos, (void*)message, 18*sizeof(uint16_t));
		}
	} else
	{
		// Respond
		spi_message_out[0] = BULLET_ERROR_MODE_MISMATCH;
		spi_message_out[1] = (uint8_t)(255-(BULLET_ERROR_MODE_MISMATCH));
	}
	DMA1_Channel3->CNDTR = 2;						// Number of data to send
	DMA1_Channel3->CCR |= (DMA_CCR3_EN);			// Enable DMA1 CH3
	SPI1->CR2 |= (SPI_CR2_TXDMAEN);					// Enable DMA TX Req
}
void processSPIChangeMode()
{
	// Get data
	uint8_t command = spi_message_in[0];
	uint8_t instruction = spi_message_in[1];
	uint8_t new_mode = spi_message_in[2];
	uint8_t crc = spi_message_in[3];

	// CRC Error
	if ((uint8_t)(255-(command + instruction + new_mode)) != crc)
	{
		//W przypadku bledu crc wyslij blad
		spi_message_out[0] = BULLET_ERROR_SPI_CRC;
		spi_message_out[1] = (uint8_t)(255-(BULLET_ERROR_SPI_CRC));
	} else
	{
		// Respond
		spi_message_out[0] = BULLET_SUCCESS;
		spi_message_out[1] = (uint8_t)(255-(BULLET_SUCCESS));

		// Change mode
		stm_mode = new_mode;
	}
	DMA1_Channel3->CNDTR = 2;						// Number of data to send
	DMA1_Channel3->CCR |= (DMA_CCR3_EN);			// Enable DMA1 CH3
	SPI1->CR2 |= (SPI_CR2_TXDMAEN);					// Enable DMA TX Req

	if(stm_mode==BULLET_MODE_PREOPERATIONAL) AX12A_WriteAllTorqueEnable(0);
	else if (stm_mode==BULLET_MODE_OPERATIONAL) AX12A_WriteAllTorqueEnable(1);
}
void processSPIPreoperationalMessage()
{
	// Get command
	uint8_t command = spi_message_in[0];

	if (command==BULLET_COMMAND_PING)
	{
		// Get instruction
		uint8_t instruction = spi_message_in[1];

		//Ping STM
		if (instruction == BULLET_INSTRUCTION_COMMUNICATION)
		{
			// Get data
			uint8_t crc = spi_message_in[2];

			// CRC Error
			if ((uint8_t)(255-(command + instruction)) != crc)
			{
				spi_message_out[0] = BULLET_ERROR_SPI_CRC;
				spi_message_out[1] = 0x00;
				spi_message_out[2] = 0x00;
				spi_message_out[3] = (uint8_t)(255-(BULLET_ERROR_SPI_CRC));
			} else
			{
				// Respond
				spi_message_out[0] = BULLET_SUCCESS;
				spi_message_out[1] = stm_mode;
				spi_message_out[2] = stm_status;
				spi_message_out[3] = (uint8_t)(255-(BULLET_SUCCESS + stm_mode + stm_status));
			}
			DMA1_Channel3->CNDTR = 4;						// Number of data to send
		// Ping specific servo
		} else if (instruction == BULLET_INSTRUCTION_SERVO)
		{
			// Get data
			uint8_t servo_id = spi_message_in[2];
			uint8_t crc = spi_message_in[3];

			// CRC Error
			if ((uint8_t)(255-(command + servo_id + instruction)) != crc)
			{
				spi_message_out[0] = BULLET_ERROR_SPI_CRC;
				spi_message_out[1] = (uint8_t)(255-(BULLET_ERROR_SPI_CRC));
			} else
			{
				// Read AX12A
				uint8_t val;
				AX12A_Ping(servo_id, &val);

				// Respond
				spi_message_out[0] = val;
				spi_message_out[1] = (uint8_t)(255-val);
			}
			DMA1_Channel3->CNDTR = 2;						// Number of data to send
			//Ping all servos
		} else if (instruction == BULLET_INSTRUCTION_SERVOS_ALL)
		{
			// Get data
			uint8_t crc = spi_message_in[2];

			// CRC Error
			if ((uint8_t)(255-(command + instruction)) != crc)
			{
				spi_message_out[0] = BULLET_ERROR_SPI_CRC;
				spi_message_out[1] = (uint8_t)(255-(BULLET_ERROR_SPI_CRC));
			} else
			{
				// Read AX12A
				uint8_t val;
				AX12A_PingAll(&val);

				// Respond
				spi_message_out[0] = val;
				spi_message_out[1] = (uint8_t)(255-val);
			}
			DMA1_Channel3->CNDTR = 2;						// Number of data to send
		} else
		{
			// Respond
			spi_message_out[0] = BULLET_ERROR_MODE_MISMATCH;
			spi_message_out[1] = (uint8_t)(255-(BULLET_ERROR_MODE_MISMATCH));
			DMA1_Channel3->CNDTR = 2;						// Number of data to send
		}
		DMA1_Channel3->CCR |= (DMA_CCR3_EN);			// Enable DMA1 CH3
		SPI1->CR2 |= (SPI_CR2_TXDMAEN);					// Enable DMA TX Req
		return;
	} else if (command==BULLET_COMMAND_READ)
	{
		// Get data
		uint8_t instruction = spi_message_in[1];
		uint8_t servo_id = spi_message_in[2];
		uint8_t crc = spi_message_in[3];

		switch (instruction)
		{
			//EEPROM
			case BULLET_INSTRUCTION_FIRMWARE_VERSION:
			case BULLET_INSTRUCTION_RETURN_DELAY_TIME:
			case BULLET_INSTRUCTION_HIGHEST_LIMIT_TEMPERATURE:
			case BULLET_INSTRUCTION_LOWEST_LIMIT_VOLTAGE:
			case BULLET_INSTRUCTION_HIGHEST_LIMIT_VOLTAGE:
			case BULLET_INSTRUCTION_STATUS_RETURN_LEVEL:
			case BULLET_INSTRUCTION_ALARM_LED:
			case BULLET_INSTRUCTION_ALARM_SHUTDOWN:
			//RAM
			case BULLET_INSTRUCTION_TORQUE_ENABLE:
			case BULLET_INSTRUCTION_LED:
			case BULLET_INSTRUCTION_CW_COMPLIANCE_MARGIN:
			case BULLET_INSTRUCTION_CCW_COMPLIANCE_MARGIN:
			case BULLET_INSTRUCTION_CW_COMPLIANCE_SLOPE:
			case BULLET_INSTRUCTION_CCW_COMPLIANCE_SLOPE:
			case BULLET_INSTRUCTION_PRESENT_VOLTAGE:
			case BULLET_INSTRUCTION_PRESENT_TEMPERATURE:
			case BULLET_INSTRUCTION_REGISTERED_INSTRUCTION:
			case BULLET_INSTRUCTION_MOVING:
			case BULLET_INSTRUCTION_LOCK:
			{
				// CRC Error
				if ((255-(command + instruction + servo_id)) != crc)
				{
					spi_message_out[0] = BULLET_ERROR_SPI_CRC;
					spi_message_out[1] = 0x00;
					spi_message_out[2] = (uint8_t)(255-(BULLET_ERROR_SPI_CRC));
				} else
				{
					// Read AX12A
					uint8_t val;
					AX12A_ReadByte(servo_id, instruction, &val);

					// Respond
					spi_message_out[0] = BULLET_SUCCESS;
					spi_message_out[1] = val;
					spi_message_out[2] = (uint8_t)(255-(BULLET_SUCCESS+val));
				}
				DMA1_Channel3->CNDTR = 3;						// Number of data to send
				break;
			}

			//EEPROM
			case BULLET_INSTRUCTION_MODEL_NUMBER:
			case BULLET_INSTRUCTION_CW_ANGLE_LIMIT:
			case BULLET_INSTRUCTION_CCW_ANGLE_LIMIT:
			case BULLET_INSTRUCTION_MAX_TORQUE:
			//RAM
			case BULLET_INSTRUCTION_GOAL_POSITION:
			case BULLET_INSTRUCTION_MOVING_SPEED:
			case BULLET_INSTRUCTION_TORQUE_LIMIT:
			case BULLET_INSTRUCTION_PRESENT_POSITION:
			case BULLET_INSTRUCTION_PRESENT_SPEED:
			case BULLET_INSTRUCTION_PRESENT_LOAD:
			case BULLET_INSTRUCTION_PUNCH:
			{
				// CRC Error
				if ((255-(command + instruction + servo_id)) != crc)
				{
					spi_message_out[0] = BULLET_ERROR_SPI_CRC;
					spi_message_out[1] = 0x00;
					spi_message_out[2] = 0x00;
					spi_message_out[3] = (uint8_t)(255-(BULLET_ERROR_SPI_CRC));
				} else
				{
					// Read AX12A
					uint8_t val[2];
					AX12A_ReadWord(servo_id, instruction, val, (uint8_t)1);

					// Respond
					spi_message_out[0] = BULLET_SUCCESS;
					spi_message_out[1] = val[0];
					spi_message_out[2] = val[1];
					spi_message_out[3] = (uint8_t)(255-(BULLET_SUCCESS+val[0]+val[1]));
				}
				DMA1_Channel3->CNDTR = 4;						// Number of data to send
				break;
			}
		}
		DMA1_Channel3->CCR |= (DMA_CCR3_EN);			// Enable DMA1 CH3
		SPI1->CR2 |= (SPI_CR2_TXDMAEN);					// Enable DMA TX Req
		return;
	} else if (command==BULLET_COMMAND_WRITE)
	{
		// Get instruction
		uint8_t instruction = spi_message_in[1];

		switch (instruction)
		{
			// EEPROM
			case BULLET_INSTRUCTION_ID:
			case BULLET_INSTRUCTION_BAUDRATE:
			case BULLET_INSTRUCTION_RETURN_DELAY_TIME:
			case BULLET_INSTRUCTION_HIGHEST_LIMIT_TEMPERATURE:
			case BULLET_INSTRUCTION_LOWEST_LIMIT_VOLTAGE:
			case BULLET_INSTRUCTION_HIGHEST_LIMIT_VOLTAGE:
			case BULLET_INSTRUCTION_STATUS_RETURN_LEVEL:
			case BULLET_INSTRUCTION_ALARM_LED:
			case BULLET_INSTRUCTION_ALARM_SHUTDOWN:
			//RAM
			case BULLET_INSTRUCTION_TORQUE_ENABLE:
			case BULLET_INSTRUCTION_LED:
			case BULLET_INSTRUCTION_CW_COMPLIANCE_MARGIN:
			case BULLET_INSTRUCTION_CCW_COMPLIANCE_MARGIN:
			case BULLET_INSTRUCTION_CW_COMPLIANCE_SLOPE:
			case BULLET_INSTRUCTION_CCW_COMPLIANCE_SLOPE:
			case BULLET_INSTRUCTION_PRESENT_VOLTAGE:
			case BULLET_INSTRUCTION_PRESENT_TEMPERATURE:
			case BULLET_INSTRUCTION_REGISTERED_INSTRUCTION:
			case BULLET_INSTRUCTION_MOVING:
			case BULLET_INSTRUCTION_LOCK:
			{
				// Get data
				uint8_t servo_id = spi_message_in[2];
				uint8_t new_value = spi_message_in[3];
				uint8_t crc = spi_message_in[4];

				// CRC Error
				if (crc!=((uint8_t)(255-(command + instruction + servo_id + new_value))))
				{
					spi_message_out[0] = BULLET_ERROR_SPI_CRC;
					spi_message_out[1] = (uint8_t)(255-(BULLET_ERROR_SPI_CRC));
				} else
				{
					// Write AX12A
					BulletStatus stat = AX12A_WriteByte(servo_id, instruction, new_value);

					// Respond
					spi_message_out[0] = stat;
					spi_message_out[1] = (uint8_t)(255-(stat));
				}
				DMA1_Channel3->CNDTR = 2;						// Number of data to send
				break;
			}

			//EEPROM
			case BULLET_INSTRUCTION_MODEL_NUMBER:
			case BULLET_INSTRUCTION_CW_ANGLE_LIMIT:
			case BULLET_INSTRUCTION_CCW_ANGLE_LIMIT:
			case BULLET_INSTRUCTION_MAX_TORQUE:
			//RAM
			case BULLET_INSTRUCTION_GOAL_POSITION:
			case BULLET_INSTRUCTION_MOVING_SPEED:
			case BULLET_INSTRUCTION_TORQUE_LIMIT:
			case BULLET_INSTRUCTION_PRESENT_POSITION:
			case BULLET_INSTRUCTION_PRESENT_SPEED:
			case BULLET_INSTRUCTION_PRESENT_LOAD:
			case BULLET_INSTRUCTION_PUNCH:
			{
				// Get data
				uint8_t servo_id = spi_message_in[2];
				uint8_t new_value[2];
				new_value[0] = spi_message_in[3];
				new_value[1] = spi_message_in[4];
				uint8_t crc = spi_message_in[5];

				// CRC Error
				if (crc!=((uint8_t)(255-(command + instruction+servo_id+new_value[0]+new_value[1]))))
				{
					spi_message_out[0] = BULLET_ERROR_SPI_CRC;
					spi_message_out[1] = (uint8_t)(255-(BULLET_ERROR_SPI_CRC));
				} else
				{
					// Write AX12A
					BulletStatus stat = AX12A_WriteWord(servo_id, instruction, new_value, 1);

					// Respond
					spi_message_out[0] = stat;
					spi_message_out[1] = (uint8_t)(255-(stat));
				}
				DMA1_Channel3->CNDTR = 2;						// Number of data to send
				break;
			}
			default:
			{
				// Respond
				spi_message_out[0] = BULLET_ERROR_MODE_MISMATCH;
				spi_message_out[1] = (uint8_t)(255-(BULLET_ERROR_MODE_MISMATCH));
				DMA1_Channel3->CNDTR = 2;						// Number of data to send
			}
		}
		DMA1_Channel3->CCR |= (DMA_CCR3_EN);			// Enable DMA1 CH3
		SPI1->CR2 |= (SPI_CR2_TXDMAEN);					// Enable DMA TX Req
		return;
	} else if (command == BULLET_COMMAND_READ_ALL) processSPIReadAll();
	else if (command == BULLET_COMMAND_WRITE_ALL) processSPIWriteAll();
	else if (command == BULLET_COMMAND_CHANGE_MODE) processSPIChangeMode();
}

void processSPIOperationalMessage(void)
{
	// Get command
	uint8_t command = spi_message_in[0];

	// PING
	if (command==BULLET_COMMAND_PING)
	{
		// Get instruction
		uint8_t instruction = spi_message_in[1];

		//Ping STM
		if (instruction == 0x00)
		{
			// Get data
			uint8_t crc = spi_message_in[2];

			// CRC Error
			if ((uint8_t)(255-(command + instruction)) != crc)
			{
				spi_message_out[0] = BULLET_ERROR_SPI_CRC;
				spi_message_out[1] = 0x01;
				spi_message_out[2] = 0x02;
				spi_message_out[3] = (uint8_t)(255-(BULLET_ERROR_SPI_CRC));
			} else
			{
				// Respond
				spi_message_out[0] = BULLET_SUCCESS;
				spi_message_out[1] = stm_mode;
				spi_message_out[2] = stm_status;
				spi_message_out[3] = (uint8_t)(255-(BULLET_SUCCESS + stm_mode + stm_status));
			}
		// CANNOT HANDLE THIS INSTRUCTION IN THIS MODE
		} else
		{
			// Respond
			spi_message_out[0] = BULLET_ERROR_MODE_MISMATCH;
			spi_message_out[1] = 0x01;
			spi_message_out[2] = 0x02;
			spi_message_out[3] = (uint8_t)(255-(BULLET_ERROR_MODE_MISMATCH));
		}
		DMA1_Channel3->CNDTR = 4;						// Number of data to send
		DMA1_Channel3->CCR |= (DMA_CCR3_EN);			// Enable DMA1 CH3
		SPI1->CR2 |= (SPI_CR2_TXDMAEN);					// Enable DMA TX Req
		return;
	} else if (command == BULLET_COMMAND_READ_ALL) processSPIReadAll();
	else if (command == BULLET_COMMAND_WRITE_ALL) processSPIWriteAll();
	else if (command == BULLET_COMMAND_CHANGE_MODE) processSPIChangeMode();
}
