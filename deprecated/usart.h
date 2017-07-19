/**
 *  @file		usart.h
 *  @author     DanielKoguciuk <daniel.koguciuk@gmail.com>
 *  @date       17.11.2013
 *  @version    1.1
 *
 *  @brief		Biblioteka inicjujaca peryferium UART w trybie 1Mbodow. Zaimplementowano
 *  			funkcje wysylajace gotowy ciag znakow do wyswietlenia w terminalu. Calosc
 *  			spieta jest poprzez kolejki FIFO nadawcza i odbiorcza.
 */

#ifndef USART_H
#define USART_H

// ======================================================================================
// ===================================== HEADERS ========================================
// ======================================================================================

#include "fifo.h"

#include "stm_lib/inc/stm32f10x_usart.h"
#include "stm_lib/inc/stm32f10x_gpio.h"
#include "stm_lib/inc/stm32f10x_rcc.h"
#include "stm_lib/inc/misc.h"

// ======================================================================================
// ===================================== DEFINES ========================================
// ======================================================================================

/**
 * 	@brief		USART periph number.
 */
#define USART_PC				USART1

/**
 * 	@brief		Port definition for USART communication with PC.
 */
#define USART_PC_PORT			GPIOA

/**
 *	@brief		Transmit pin definition for USART communication with PC.
 */
#define USART_PC_PIN_TX			GPIO_Pin_9

/**
 *	@brief		Receive pin definition for USART communication with PC.
 */
#define USART_PC_PIN_RX			GPIO_Pin_10

/**
 * 	@brief		Clock definition for USART.
 */
#define USART_PC_CLOCK			RCC_APB2Periph_USART1

/**
 * 	@brief		Clock definition for USART port.
 */
#define USART_PC_PORT_CLOCK		RCC_APB2Periph_GPIOA

/**
 * 	@brief		USART IRQN.
 */
#define USART_PC_IRQ			USART1_IRQn

// ======================================================================================
// ================================== FIFO VARIABLES ====================================
// ======================================================================================

/**
 * 	@brief		UART receive buffer declaration.
 */
extern FIFO_type USART_PC_RX_Buffer;

/**
 * 	@brief		UART transmit buffer declaration.
 */
extern FIFO_type USART_PC_TX_Buffer;

// ======================================================================================
// ================================= PUBLIC FUNCTIONS ===================================
// ======================================================================================

void USART_PC_Config();

void usartSendString(char* string);
void usartSendInt(uint16_t number);
void usartSendFloat(float number);

void usartSendInt8(int8_t number);
void usartSendInt16(int16_t number);

// ======================================================================================
// ================================= PRIVATE FUNCTIONS ==================================
// ======================================================================================

void USART1_IRQHandler(void);

#endif // USART_H
