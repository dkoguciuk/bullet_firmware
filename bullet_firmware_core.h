/**
 *  @file
 *  @author     MichalSasim <sasim.michal@gmail.com>
 *  @date       18.04.2014r.
 *  @version    1.0
 *
 *  @brief		Plik systemowy, sluzacy do definiowania pod³¹czenia poszczególnych peryferiów
 *              do odpowienich pinów mikrokontrolera.
 */

#ifndef BULLET_FIRMWARE_CORE_H
#define BULLET_FIRMWARE_CORE_H

#include "bullet_datatypes.h"
#include <stm_lib/inc/misc.h>
#include <stm_lib/inc/stm32f10x_dma.h>
#include <stm_lib/inc/stm32f10x_rcc.h>
#include <stm_lib/inc/stm32f10x_gpio.h>
#include <stm_lib/inc/stm32f10x_flash.h>
#include <stm_lib/inc/stm32f10x_tim.h>
#include <stm_lib/inc/stm32f10x_usart.h>
#include <stm_lib/inc/stm32f10x_spi.h>

extern volatile BulletSTMMode stm_mode;
extern volatile BulletStatus stm_status;

extern volatile uint16_t servos_goal_pos[18];
extern volatile uint16_t servos_pres_pos_speed_load[54];
extern volatile uint8_t main_process_spi;

extern volatile uint8_t spi_message_in[39];
extern volatile uint8_t spi_message_out[110];

extern volatile uint8_t uart_message_in[110];
extern volatile uint8_t uart_message_out[110];

//--------------------------------//
//        Serwo analogowe         //
//--------------------------------//
/**
 * @brief		Definicja portu podlaczenia serwa analogowego
 */
#define SERVO_Port   GPIOA

/**
 * @brief		Definicja pinu podlaczenia serwa analogowego
 */
#define SERVO_Pin    GPIO_Pin_8



//--------------------------------//
//           Switch'e             //
//--------------------------------//
/**
 * 	@brief		Definicja portu, do którego pod³¹czone s¹ microswitche.
 */
#define SWITCH_Port     GPIOB

/**
 * 	@brief		Definicja pinu, do którego pod³¹czony jest switch pierwszej nogi.
 */
#define SWITCH1_Pin     GPIO_Pin_0

/**
 * 	@brief		Definicja pinu, do którego pod³¹czony jest switch drugiej nogi.
 */
#define SWITCH2_Pin     GPIO_Pin_1

/**
 * 	@brief		Definicja pinu, do którego pod³¹czony jest switch trzeciej nogi.
 */
#define SWITCH3_Pin     GPIO_Pin_2

/**
 * 	@brief		Definicja pinu, do którego pod³¹czony jest switch czwartej nogi.
 */
#define SWITCH4_Pin     GPIO_Pin_3

/**
 * 	@brief		Definicja pinu, do którego pod³¹czony jest switch pi¹tej nogi.
 */
#define SWITCH5_Pin     GPIO_Pin_4

/**
 * 	@brief		Definicja pinu, do którego pod³¹czony jest switch szóstej nogi.
 */
#define SWITCH6_Pin     GPIO_Pin_5



//--------------------------------//
//             LED'y              //
//--------------------------------//
/**
 * 	@brief		Definicja portu, do którego pod³¹czone s¹ diody LED.
 */
#define LED_Port     GPIOB

/**
 * 	@brief		Definicja pinu, do którego pod³¹czony jest LED pierwszej nogi.
 */
#define LED1_Pin     GPIO_Pin_10

/**
 * 	@brief		Definicja pinu, do którego pod³¹czony jest LED drugiej nogi.
 */
#define LED2_Pin     GPIO_Pin_11

/**
 * 	@brief		Definicja pinu, do którego pod³¹czony jest LED trzeciej nogi.
 */
#define LED3_Pin     GPIO_Pin_12

/**
 * 	@brief		Definicja pinu, do którego pod³¹czony jest LED czwartej nogi.
 */
#define LED4_Pin     GPIO_Pin_13

/**
 * 	@brief		Definicja pinu, do którego pod³¹czony jest LED pi¹tej nogi.
 */
#define LED5_Pin     GPIO_Pin_14

/**
 * 	@brief		Definicja pinu, do którego pod³¹czony jest LED szóstej nogi.
 */
#define LED6_Pin     GPIO_Pin_15







#endif /* BULLET_FIRMWARE_CORE_H */
