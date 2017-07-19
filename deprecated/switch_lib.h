/**
 *  @file
 *  @author     MichalSasim <sasim.michal@gmail.com>
 *  @date       18.04.2013
 *  @version    1.1
 *
 *  @brief		Biblioteka umozliwia odczyt stanu microswitchy. Kazda noga Heksapoda zaopatrzona jest
 *              w jeden microswitch. Funkcja odczuj¹ca ich stan zaimplementowana jest dla przypadku,
 *              w którym microswitche pod³¹czone s¹ do pinów 8, 9, 10, 11, 12, 13 portu GPIOB.
 *              W glownym programie nalezy wywolac funkcje SWITCH_Config().
 */

#ifndef SWITCH_LIB_H_
#define SWITCH_LIB_H_

#include <stddef.h>
#include "stm32f10x.h"
#include "system.h"

/**
 * 	@brief		Funkcja konfiguruj¹ca mikrokontroler, aby obs³ugiwaæ microswitche.
 */
void SWITCH_Config(void);

/**
 * 	@brief		Funkcja konfiguruj¹ca niezbêdne zegary.
 */
void SWITCH_RCCConfig(void);

/**
 * 	@brief		Funkcja konfiguruj¹ca wejscia/wyjscia.
 */
void SWITCH_GPIOConfig(void);

/**
 * 	@brief		Funkcja odczytuj¹ca stan mikroswitchy. Zwraca bajt, w którym bit 0 to pierwsza noga,
 * 	            a bit 5 to szósta noga. Bity 6 i 7 s¹ zawsze zerowe i nie s¹ wykorzystywane.
 */
uint8_t SWITCH_ReadPositions(void);

#endif /* SWITCH_LIB_H_ */
