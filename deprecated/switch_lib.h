/**
 *  @file
 *  @author     MichalSasim <sasim.michal@gmail.com>
 *  @date       18.04.2013
 *  @version    1.1
 *
 *  @brief		Biblioteka umozliwia odczyt stanu microswitchy. Kazda noga Heksapoda zaopatrzona jest
 *              w jeden microswitch. Funkcja odczuj�ca ich stan zaimplementowana jest dla przypadku,
 *              w kt�rym microswitche pod��czone s� do pin�w 8, 9, 10, 11, 12, 13 portu GPIOB.
 *              W glownym programie nalezy wywolac funkcje SWITCH_Config().
 */

#ifndef SWITCH_LIB_H_
#define SWITCH_LIB_H_

#include <stddef.h>
#include "stm32f10x.h"
#include "system.h"

/**
 * 	@brief		Funkcja konfiguruj�ca mikrokontroler, aby obs�ugiwa� microswitche.
 */
void SWITCH_Config(void);

/**
 * 	@brief		Funkcja konfiguruj�ca niezb�dne zegary.
 */
void SWITCH_RCCConfig(void);

/**
 * 	@brief		Funkcja konfiguruj�ca wejscia/wyjscia.
 */
void SWITCH_GPIOConfig(void);

/**
 * 	@brief		Funkcja odczytuj�ca stan mikroswitchy. Zwraca bajt, w kt�rym bit 0 to pierwsza noga,
 * 	            a bit 5 to sz�sta noga. Bity 6 i 7 s� zawsze zerowe i nie s� wykorzystywane.
 */
uint8_t SWITCH_ReadPositions(void);

#endif /* SWITCH_LIB_H_ */
